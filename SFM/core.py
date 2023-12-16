######### no loggin warning on av ############
import av.logging
restore_default_callback = lambda *args: args
av.logging.restore_default_callback = restore_default_callback
av.logging.set_level(av.logging.ERROR)
##############################################
# tools 
import logging
import ssl
import uuid
import time
import json
import threading
import os
import shutil
from pathlib import Path
# webapp
import asyncio
from aiohttp import web
# aiortc
from av import VideoFrame
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay
# sfm module
import numpy as np
from SFM import pysfm
from SFM.mvs import MVSPipe
######################################################################################################################################################

FBOW_PATH = str(Path(__file__).parent/'source/vocab/orb_mur.fbow')
CONF_PATH = str(Path(__file__).parent/'config.yaml')
MVS_PIPE = MVSPipe('/usr/local/bin/OpenMVS')

######################################################################################################################################################
# encode numpy.ndarray to json
class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)

######################################################################################################################################################
# base track class
class BaseTrack(MediaStreamTrack):
    kind = 'video'
    session = None
    track = None
    released = False
    base_dir = None
    response = VideoFrame.from_ndarray(np.zeros((1, 1, 3), np.uint8), format='rgb24')
    on_release_callback = None
    on_terminate_callback = None

    ### init
    def __init__(self, uid, pc, usr_dir):
        super().__init__()  # don't forget this!
        self.uid = uid
        self.base_dir = Path(usr_dir) / str(self.uid)
        # init webrtc
        self.webrtc_init(pc)
        # init data channel 
        self.data_channel = pc.createDataChannel('status')
        # set channel callback
        def on_message(msg):
            if msg == 'release':  self.release()
            elif msg == 'cancel': self.terminate()
        self.data_channel.add_listener('message', on_message) 

    ### init webrtc
    def webrtc_init(self, pc):
        # track images
        @pc.on('track')
        def on_track(track):
            self.track = MediaRelay().subscribe(track)
            pc.addTrack(self)
            @track.on('ended')
            async def on_ended():
                self.terminate()
        # data channel messages
        @pc.on('datachannel')
        def on_datachannel(channel):
            pass

    ### add track source
    def set_track(self,track):
        self.track = track

    ### complete scanning
    def release(self):
        if self.released: return
        self.released = True
        data = self.session.release()
        if self.on_release_callback is not None:
            self.on_release_callback(data)

    ### cancel sacnning
    def terminate(self):
        if self.released: return
        self.released = True
        self.session.cancel()
        if self.on_terminate_callback is not None:
            self.on_terminate_callback()

    # recv frame
    async def recv(self):
        frame = await self.track.recv()
        img = frame.to_ndarray(format='bgr24')
        # new tracking
        self.session.add_track(img)

        # get track status
        position = self.session.get_position_three().flatten()
        state = self.session.tracking_state()
        data = np.hstack((state, position))

        # send status
        if not self.released:
            try: self.data_channel.send(json.dumps(data, cls=NumpyArrayEncoder))
            except Exception as e: logging.warn(e)

        # return empty response
        self.response.pts = frame.pts
        self.response.time_base = frame.time_base
        return self.response


######################################################################################################################################################
# Capture Track
class CaptureTrack(BaseTrack):
    after_release = None
    proto_thread = None

    def __init__(self, uid, pc, usr_dir):
        super().__init__(uid, pc, usr_dir)
        self.map_path = str(self.base_dir/'feature.bin')
        self.scene_path = str(self.base_dir/'scene/scene.mvs')
        self.raw_img_dir = str(self.base_dir/'images')
        # set callback
        self.on_release_callback = self.on_success
        self.on_terminate_callback = self.on_cancel
        # set protobuf channel
        self.proto_channel = pc.createDataChannel('protobuf')
        self.proto_thread = threading.Thread(target=self.async_proto_thread)

    def init_dir(self):
        if os.path.exists(self.base_dir):
            shutil.rmtree(self.base_dir)
        os.mkdir(self.base_dir)
        os.mkdir(self.raw_img_dir)
        os.mkdir(str(self.base_dir/'scene'))
        # write basic info
        with open(str(self.base_dir/'info.json'), 'w') as f:
            json.dump({'id': str(self.uid), 
                'time': time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            }, f)
    
    def create(self, imw = 640, imh = 480, line=False):
        self.init_dir()
        config = pysfm.Config(CONF_PATH).vocab(FBOW_PATH)
        config.model(imw, imh).line_track(line)
        config.serialize(self.map_path, self.scene_path, self.raw_img_dir)
        self.session = pysfm.Session(config)  
        self.proto_thread.start()

    def on_success(self, data):
        MVS_PIPE.add_task(self.scene_path)
        if self.after_release is not None:
            self.after_release(self.uid, data)
    
    def on_cancel(self):
        logging.warn('files about session %s will be removed', self.uid)
        # remove folder
        if os.path.exists(self.base_dir):
            shutil.rmtree(self.base_dir)
    
    def set_after_release(self, callback):
        self.after_release = callback

    def async_proto_thread(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.async_send_proto())
        loop.close()

    ### map protobuf 
    async def async_send_proto(self):
        while not self.released:
            if self.session is None: continue
            buf = self.session.get_map_protobuf()
            if len(buf) > 0:
                try: self.proto_channel.send(buf)
                except Exception as e: logging.warn(e)
        logging.info('protobuf channel closed')
      

######################################################################################################################################################
# Review Track
class ReviewTrack(BaseTrack):
    def __init__(self, uid, pc, usr_dir):
        super().__init__(pc, uid, usr_dir)
        self.map_path = str(self.base_dir/'feature.bin')
    
    def create(self, imw = 640, imh = 480, line=False):
        config = pysfm.Config(CONF_PATH).vocab(FBOW_PATH)
        config.model(imw, imh).line_track(line)
        config.database(self.map_path)
        self.session = pysfm.Session(config)
