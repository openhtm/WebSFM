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


# io tools
from handler.tools import write_info, read_info

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'


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
# webrtc session handler
async def session_handler(request):
    params = await request.json()
    create_mode, uid = params.get('create_mode', None), params.get('uid', None)

    if create_mode is None or (not create_mode and uid is None):
        return web.json_response({'status': False, 'msg' : 'invalid params'})

    if create_mode and MVS_PIPE.tasks.qsize() > 2:
        return web.json_response({'status': False, 'msg' : 'service busy'})

    if not create_mode and not os.path.exists(str(USR_DIR/str(uid)/'feature.bin')):
        return web.json_response({'status': False, 'msg' : 'no such scene'})

    offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

    pc = RTCPeerConnection() 
    pcs.add(pc)

    session_id = str(uuid.uuid1()) if uid is None else uid
    pc_id = 'WebRTC-Session(%s)' % session_id
  
    def log_info(msg, *args):
        logger.info(pc_id + ' ' + msg, *args)

    # create data channel to send position and state
    sfm_track = None
    
    if create_mode:
        def callback(uid, data):
            info = read_info(uid)
            info['nkfs'], info['nmps'] = int(data[0]), int(data[1])
            write_info(uid, info)
        init_dir(session_id)
        sfm_track = CreateTrack(session_id, pc, USR_DIR, callback)
    else:
        sfm_track = ReviewTrack(session_id, pc, USR_DIR)

    # track images
    @pc.on('track')
    def on_track(track):
        sfm_track.set_track(relay.subscribe(track))
        pc.addTrack(sfm_track)
        @track.on('ended')
        async def on_ended():
            sfm_track.release()

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
        'status' : True,
        'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type
    })

# close all rtc connection when shut down
async def on_shutdown(app):
    MVS_PIPE.release()
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()