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

# query all scenes
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

      if 'grid' in basic: basic['status'] = 4

      # add to dict
      data[folder] = basic
  
  return web.json_response(data)


# query detail
async def query_info(request):
  params = request.query
  uid = params.get('uid', None)
  data = read_info(uid)
  return web.json_response(data)


# remove unique scene
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


# define scenen base
async def define_base(request):
  json_data = await request.json()
  if 'uid' not in json_data:
    return web.json_response({
      'status': False,
      'msg': 'wrong format'
    })
    
  try:
    uid = json_data['uid']
    size = json_data['size']
    position = json_data['position']
    rotation = json_data['rotation']

    info = read_info(uid)
    info['define'] = {
      'size': size,
      'position': position,
      'rotation': rotation
    }

    write_info(uid, info)
    return web.json_response({
      'status': True,
      'msg': 'success'
    })

  except Exception as e:
    return web.json_response({
      'status': False,
      'msg': str(e)
    })


#define grid
async def define_grid(request):
  json_data = await request.json()
  if 'uid' not in json_data:
    return web.json_response({
      'status': False,
      'msg': 'wrong format'
    })
    
  try:
    uid = json_data['uid']
    division = json_data['division']
    mindist = json_data['mindist']
    maxdist = json_data['maxdist']
    data = json_data['array']

    info = read_info(uid)
    info['grid'] = {
      'division': division,
      'mindist': mindist,
      'maxdist': maxdist,
      'array' : data
    }

    write_info(uid, info)
    return web.json_response({
      'status': True,
      'msg': 'success'
    })

  except Exception as e:
    return web.json_response({
      'status': False,
      'msg': str(e)
    })

#define landmarks
async def define_landmark(request):
  json_data = await request.json()
  if 'uid' not in json_data:
    return web.json_response({
      'status': False,
      'msg': 'wrong format'
    })
    
  try:
    uid = json_data['uid']
    landmarks = json_data['landmarks']

    info = read_info(uid)
    info['landmarks'] = landmarks
    
    write_info(uid, info)
    return web.json_response({
      'status': True,
      'msg': 'success'
    })

  except Exception as e:
    return web.json_response({
      'status': False,
      'msg': str(e)
    })