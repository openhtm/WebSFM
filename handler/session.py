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
# capture session handler
async def session_capture_handler(request):
    params = await request.json()
    pc = RTCPeerConnection()

    imwidth, imheight = params.get('width', 640), params.get('height', 480)
    session_id = str(uuid.uuid1())
    
    # create data channel to send position and state
    TrackSession = CaptureTrack(session_id, pc, USR_DIR)
    TrackSession.create(imwidth, imheight)

    return await session_handler(params, pc, TrackSession)

# review session handler
async def session_review_handler(request):
    params = await request.json()
    pc = RTCPeerConnection()

    imwidth, imheight = params.get('width', 640), params.get('height', 480)
    target_id = params.get('target', None)
    if target_id is None:  return web.json_response({'status': False, 'msg' : 'no such scene'})

    TrackSession = ReviewTrack(target_id, pc, USR_DIR)
    TrackSession.create(imwidth, imheight)

    return await session_handler(params, pc, TrackSession)

# basic session handler
async def session_handler(params, pc, session):
    offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])
    pcs.add(pc)

    @pc.on('connectionstatechange')
    async def on_connectionstatechange():
        if pc.connectionState == 'failed':
            logging.warning('Connection %s failed', session.uid)
            session.terminate()
            await pc.close()
            pcs.discard(pc)

    # handle offer
    await pc.setRemoteDescription(offer)
    # send answer
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
  
    return web.json_response({
        'status' : True, 'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type
    })

# close all rtc connection when shut down
async def on_shutdown(app):
    MVS_PIPE.release()
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()