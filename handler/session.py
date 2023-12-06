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

# io tools
from handler.tools import write_info, read_info

MVS_PIPE = MVSPipe(Path('/usr/local/bin/OpenMVS'))

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'
FBOW_PATH = str(ROOTDIR/'SFM/vocab/orb_mur.fbow')


logger = logging.getLogger('WebSFM')
pcs = set()
relay = MediaRelay()

######################################################################################################################################################
def init_dir(uid):
  folder = str(USR_DIR/str(uid))
  if os.path.exists(folder):
    shutil.rmtree(folder)
  os.mkdir(folder)
  os.mkdir(str(USR_DIR/str(uid)/'images'))
  os.mkdir(str(USR_DIR/str(uid)/'scene'))
  # log time
  time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
  write_info(str(uid), {'id': str(uid), 'time': time_str})

######################################################################################################################################################
# encode numpy.ndarray to json
class NumpyArrayEncoder(json.JSONEncoder):
  def default(self, obj):
    if isinstance(obj, np.ndarray):
      return obj.tolist()
    return JSONEncoder.default(self, obj)


######################################################################################################################################################
BlankFrame = VideoFrame.from_ndarray(np.zeros((480, 640, 3), np.uint8), format='rgb24')
# webrtc frame track handler
class SfmTrack(MediaStreamTrack):
  kind = 'video'

  def __init__(self, session_id, create_mode, channels):
    super().__init__()  # don't forget this!
    self.session_id = session_id
    self.track = None
    self.channels = channels
    self.frame2d = True
    self.create_mode = create_mode
    self.released = False
    self.folder = USR_DIR/str(self.session_id)

    # init session
    self.create_session()

    # set channel
    def on_message(msg):
      if msg == 'release':  self.release()
      elif msg == 'cancel': self.terminate()
    self.channels['signal'].add_listener('message', on_message) 

  def set_track(self,track):
    self.track = track

  # create websfm session
  def create_session(self):
    # create session
    self.session = pysfm.Session(FBOW_PATH, 640, 480, True, str(self.folder/'images'))
    self.session.enable_viewer(False)

    if self.create_mode:
      init_dir(self.session_id)
      self.session.save_map(True, str(self.folder/'map.yaml'))
      self.session.save_mvs(True, str(self.folder/'scene/scene.mvs'))
    else:
      self.session.load_map(True, str(self.folder/'map.yaml'))
  
  # complete scanning
  def release(self):
    if self.released: return
    self.released = True
    self.session.release()
    if self.create_mode: MVS_PIPE.add_task(str(self.folder/'scene/scene.mvs'))   

  # cancel sacnning
  def terminate(self):
    logger.warn('session terminated, files about session %s will be removed', self.session_id)
    self.session.cancel()
    # remove folder
    folder = str(USR_DIR/str(self.session_id))
    if os.path.exists(folder):
      shutil.rmtree(folder)


  # recv frame
  async def recv(self):
    global BlankFrame
    frame = await self.track.recv()
    img = frame.to_ndarray(format='bgr24')
    new_frame = BlankFrame

    # new tracking
    self.session.add_track(img)

    # get map frame
    # img = self.session.get_orb_visual() if self.frame2d else self.session.get_map_visual()
    img = self.session.get_map_visual()
    if img.size > 0: new_frame = VideoFrame.from_ndarray(img, format='bgr24')
    # push position
    position = self.session.get_position_gl()
    features = self.session.get_feature_points()
    state = self.session.tracking_state()

    # send status
    if not self.released:
      try: self.channels['position'].send(json.dumps(position, cls=NumpyArrayEncoder))
      except Exception as e: logger.warn(e)

      try: self.channels['features'].send(json.dumps(features, cls=NumpyArrayEncoder))
      except Exception as e: logger.warn(e)

      try: self.channels['state'].send(json.dumps(state, cls=NumpyArrayEncoder))
      except Exception as e: logger.warn(e)

    # push frames
    new_frame.pts = frame.pts
    new_frame.time_base = frame.time_base
    return new_frame


######################################################################################################################################################
# webrtc session handler
async def session_handler(request):
  params = await request.json()

  if params['create_mode'] and MVS_PIPE.tasks.qsize() > 2:
    return web.json_response({'status': -1, 'msg' : 'service busy'})

  offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

  pc = RTCPeerConnection() 
  pcs.add(pc)

  session_id = str(uuid.uuid1())
  pc_id = 'WebRTC-Session(%s)' % session_id
  
  def log_info(msg, *args):
    logger.info(pc_id + ' ' + msg, *args)

  # create data channel to send position and state
  channels = {
    'position' : pc.createDataChannel('position'),
    'features' : pc.createDataChannel('features'),
    'state' : pc.createDataChannel('state'),
    'signal' : pc.createDataChannel('signal')
  }

  sfm_track = SfmTrack(session_id, params['create_mode'], channels)

  # track images
  @pc.on('track')
  def on_track(track):
    sfm_track.set_track(relay.subscribe(track))
    pc.addTrack(sfm_track)
    @track.on('ended')
    async def on_ended():
      sfm_track.terminate()

  @pc.on('connectionstatechange')
  async def on_connectionstatechange():
    log_info('Connection state is %s', pc.connectionState)
    if pc.connectionState == 'failed':
      sfm_track.terminate()
      await pc.close()
      pcs.discard(pc)

  # data channel messages
  @pc.on('datachannel')
  def on_datachannel(channel):
    pass

  # handle offer
  await pc.setRemoteDescription(offer)

  # send answer
  answer = await pc.createAnswer()
  await pc.setLocalDescription(answer)
  
  return web.json_response({
    'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type
  })

# close all rtc connection when shut down
async def on_shutdown(app):
  MVS_PIPE.release()
  # close peer connections
  coros = [pc.close() for pc in pcs]
  await asyncio.gather(*coros)
  pcs.clear()