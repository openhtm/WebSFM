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
from SFM.mvs import MVS

MVS_BIN = Path('/usr/local/bin/OpenMVS')

ROOTDIR = Path(__file__).parent.parent
DATA_DIR = ROOTDIR/'static/usr'

FBOW_DIR = str(ROOTDIR/'SFM/vocab/orb_mur.fbow')
BASE_DIR = str(DATA_DIR/'test')
FEATURE_PATH = str(DATA_DIR/'test/map.yaml')
IMAGE_DIR = str(DATA_DIR/'test/images')
SCENE_DIR = str(DATA_DIR/'test/scene')
SCENE_PATH = str(DATA_DIR/'test/scene/scene.mvs')


logger = logging.getLogger('WebSFM')
pcs = set()
relay = MediaRelay()

######################################################################################################################################################
def reset_dir():
  if os.path.exists(BASE_DIR):
    shutil.rmtree(BASE_DIR)
  os.mkdir(BASE_DIR)
  os.mkdir(IMAGE_DIR)
  os.mkdir(SCENE_DIR)

######################################################################################################################################################
# encode numpy.ndarray to json
class NumpyArrayEncoder(json.JSONEncoder):
  def default(self, obj):
    if isinstance(obj, np.ndarray):
      return obj.tolist()
    return JSONEncoder.default(self, obj)


######################################################################################################################################################
# webrtc frame track handler
class SlamTrack(MediaStreamTrack):
  kind = 'video'
  frame = 'map'

  def __init__(self, track, session, channels, params):
    super().__init__()  # don't forget this!
    self.track = track
    self.session = session
    self.channels = channels
    self.params = params
    self.blank = VideoFrame.from_ndarray(np.zeros((480, 640, 3), np.uint8), format='rgb24')

    # set channel
    def on_message(msg):
      if msg == 'release':
        self.session.release()
        if self.params['mode'] == 'slam_save':
          logger.info('start mvs reconstruct')
          MVS(MVS_BIN, BASE_DIR).generate(SCENE_PATH)

      elif msg == 'switch':
        self.frame = 'map' if self.frame == 'orb' else 'orb'

    self.channels['signal'].add_listener('message', on_message) 


  async def recv(self):
    frame = await self.track.recv()
    img = frame.to_ndarray(format='bgr24')
    new_frame = self.blank

    # new tracking
    self.session.add_track(img)

    # get map frame
    img = self.session.get_map_visual() if self.frame == 'map' else self.session.get_orb_visual()
    if img.size > 0: new_frame = VideoFrame.from_ndarray(img, format='bgr24')
    # push position
    position = self.session.get_position()
    state = self.session.tracking_state()
    
    try:
      self.channels['position'].send(json.dumps(position, cls=NumpyArrayEncoder))
      self.channels['state'].send(json.dumps(state, cls=NumpyArrayEncoder))
    except Exception as e:
      logger.error(str(e))

    # push frames
    new_frame.pts = frame.pts
    new_frame.time_base = frame.time_base
    return new_frame


######################################################################################################################################################
# webrtc session handler
async def session_handler(request):
  params = await request.json()
  offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

  pc = RTCPeerConnection() 
  pc_id = 'WebRTC-Session(%s)' % uuid.uuid4()
  pcs.add(pc)

  def log_info(msg, *args):
    logger.info(pc_id + ' ' + msg, *args)
  
  # create slam session ###########################################################################
  if params['mode'] == 'slam_save': 
    reset_dir()
    
  session = pysfm.Session(FBOW_DIR, 640, 480, True, IMAGE_DIR)
  session.enable_viewer()
  
  if params['mode'] == 'slam_save': 
    # save orb feature map
    session.save_map(True, FEATURE_PATH)
    # save mvs scene
    session.save_mvs(True, SCENE_PATH)

  elif params['mode'] == 'tracking': 
    session.load_map(True, FEATURE_PATH)
  
  ################################################################################################


  # create data channel to send position and state
  channels = {
    'position' : pc.createDataChannel('position'),
    'state' : pc.createDataChannel('state'),
    'signal' : pc.createDataChannel('signal')
  }
  channels['signal'].add_listener('message', lambda msg: print(msg))
  # set channel

  @pc.on('connectionstatechange')
  async def on_connectionstatechange():
    log_info('Connection state is %s', pc.connectionState)
    if pc.connectionState == 'failed':
      session.cancel()
      await pc.close()
      pcs.discard(pc)

  # data channel messages
  @pc.on('datachannel')
  def on_datachannel(channel):
    # print('new channel'channel.label)
    pass

  # track images
  @pc.on('track')
  def on_track(track):
    log_info('Track %s received', track.kind)

    if track.kind == 'video':
      pc.addTrack(SlamTrack(relay.subscribe(track), session, channels, params))

    @track.on('ended')
    async def on_ended():
      log_info('Track %s ended', track.kind)
      session.release()

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
  # close peer connections
  coros = [pc.close() for pc in pcs]
  await asyncio.gather(*coros)
  pcs.clear()