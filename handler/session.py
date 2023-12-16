# tools 
import logging
import ssl
import uuid
import json
from pathlib import Path
# webapp
import asyncio
from aiohttp import web
# aiortc
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
# io tools
from handler.tools import write_info, read_info, append_info
# sfm module
from SFM.core import CaptureTrack, ReviewTrack, MVS_PIPE

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'

pcs = set()

######################################################################################################################################################
# webrtc session handler
async def session_handler(request):
    params = await request.json()
    mode, uid = params.get('mode', None), params.get('uid', None)

    offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])
    pc = RTCPeerConnection() 
    pcs.add(pc)

    session_id = str(uuid.uuid1()) if mode == 'capture' else uid

    # create data channel to send position and state
    TrackSession = None
    
    if mode == 'capture':
        def callback(uid, data): 
            append_info(uid, {'nkfs' : int(data[0]), 'nmps' : int(data[1])})
        TrackSession = CaptureTrack(session_id, pc, USR_DIR)
        TrackSession.set_after_release(callback)
    
    elif mode == 'review':
        if uid is None:  return web.json_response({'status': False, 'msg' : 'no such scene'})
        TrackSession = ReviewTrack(session_id, pc, USR_DIR)
    
    else:
        return web.json_response({'status': False, 'msg' : 'invalid mode'})
    
    TrackSession.create()

    @pc.on('connectionstatechange')
    async def on_connectionstatechange():
        if pc.connectionState == 'failed':
            logging.warning('Connection %s failed', session_id)
            TrackSession.terminate()
            await pc.close()
            pcs.discard(pc)

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