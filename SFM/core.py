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

BlankFrame = VideoFrame.from_ndarray(np.zeros((1, 1, 3), np.uint8), format='rgb24')
FBOW_PATH = str(Path(__file__).parent/'source/vocab/orb_mur.fbow')

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
    map_thread = None

    ### init
    def __init__(self, uid, pc, usr_dir):
        super().__init__()  # don't forget this!
        self.uid = uid
        self.base_dir = Path(usr_dir) / str(self.uid)
        self.sig_channel = pc.createDataChannel('signal')
        self.sta_channel = pc.createDataChannel('status')
        self.buf_channel = pc.createDataChannel('mapmsg')

        self.map_thread = threading.Thread(target=self.send_map_proto)
    
    ### map protobuf 
    def send_map_proto(self):
        if self.released or self.session is None: return
        buf = self.session.get_map_protobuf()
        if len(buf) > 0:
            try: self.buf_channel.send(buf)
            except Exception as e: logger.warn(e)


    ### add track source
    def set_track(self,track):
        self.track = track

    ### create websfm session
    def create_session(self):
        pass

    ### complete scanning
    def release(self):
        self.released = True
        pass 

    ### cancel sacnning
    def terminate(self):
        pass

    # recv frame
    async def recv(self):
        global BlankFrame

        frame = await self.track.recv()
        img = frame.to_ndarray(format='bgr24')
        new_frame = BlankFrame

        # new tracking
        self.session.add_track(img)

        # push position
        position = self.session.get_position_three()
        features = self.session.get_feature_points()
        state = self.session.tracking_state()
        data = {
          'position': position,
          'features': features,
          'state': state
        }

        # send status
        if not self.released:
            try: self.sta_channel.send(json.dumps(data, cls=NumpyArrayEncoder))
            except Exception as e: logger.warn(e)

        # push frames
        new_frame = BlankFrame
        new_frame.pts = frame.pts
        new_frame.time_base = frame.time_base
        return new_frame

######################################################################################################################################################

class CreateTrack(BaseTrack):
    def __init__(self, uid, pc, usr_dir, callback=None):
        super().__init__(pc, uid, usr_dir)
        self.map_path = str(self.base_dir/'feature.bin')
        self.scene_path = str(self.base_dir/'scene/scene.mvs')
        self.raw_img_path = str(self.base_dir/'images')

        # set channel
        def on_message(msg):
            if msg == 'release':  self.release(callback)
            elif msg == 'cancel': self.terminate()
        self.sig_channel.add_listener('message', on_message) 
    
    def create(self, conf_path, line=False):
        config = pysfm.Config(str(conf_path))
        config.vocab(FBOW_PATH).model(640, 480)
        config.line_track(line)
        config.serialize(self.map_path, self.scene_path, self.raw_img_path)
        self.session = pysfm.Session(config)
        

    def release(self, callback=None):
        if self.released: return
        super.release()
        data = self.session.release()

        if callback is not None:
            callback(self.uid, data)
        
        # create new scene
        MVS_PIPE.add_task(self.scene_path)  
    
    ### cancel sacnning
    def terminate(self):
        if self.released: return
        super.release()
        self.session.cancel()

        # remove terminate scene
        logger.warn('files about session %s will be removed', self.uid)
        # remove folder
        if os.path.exists(self.folder):
            shutil.rmtree(self.folder)

######################################################################################################################################################

class ReviewTrack(BaseTrack):
    def __init__(self, uid, pc, usr_dir):
        super().__init__(pc, uid, usr_dir)
        self.map_path = str(self.base_dir/'feature.bin')
        
        # set channel
        def on_message(msg):
            if msg == 'release':  self.release()
            elif msg == 'cancel': self.terminate()
        self.sig_channel.add_listener('message', on_message) 
    
    def create(self, conf_path, line=False):
        config = pysfm.Config(str(conf_path))
        config.vocab(FBOW_PATH).model(640, 480)
        config.line_track(line)
        config.database(self.map_path)
        self.session = pysfm.Session(config)
        

    def release(self):
        if self.released: return
        super.release()
        data = self.session.release()
    
    ### cancel sacnning
    def terminate(self):
        if self.released: return
        super.release()
        self.session.cancel()