# tools 
import logging
import json
import os
import shutil
from pathlib import Path
# webapp
import asyncio
from aiohttp import web
# io tools
from handler.tools import write_info, read_info, read_yaml, read_status

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'

logger = logging.getLogger('Retrieve')

async def query_scenes(request):
  data = {}
  for folder in os.listdir(str(USR_DIR)):
    if os.path.isdir(str(USR_DIR/folder)):
      # basic infomation
      basic = read_info(folder)
      
      # orb features
      try:
        mapinfo = read_yaml(folder)
        basic['nmps'] = mapinfo['nMappoints']
        basic['nkfs'] = mapinfo['nKeyframes']
      except Exception as e:
        pass
      
      # mvs status
      status = read_status(folder)
      basic['status'] = status

      # add to dict
      data[folder] = basic
  
  return web.json_response(data)

async def remove_scene(request):
  json_data = await request.json()
  if 'uid' not in json_data:
    return web.json_response({
      'status': False,
      'msg': 'wrong format'
    })

  uid = json_data['uid']
  folder = str(USR_DIR/str(uid))
  if os.path.exists(folder):
    shutil.rmtree(folder)
    return web.json_response({
      'status': True,
      'msg': 'success'
    })

  else:
    return web.json_response({
      'status': False,
      'msg': 'no such file'
    })