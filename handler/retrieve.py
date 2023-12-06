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
from handler.tools import write_info, read_info, read_yaml

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'

logger = logging.getLogger('Retrieve')

async def query_scenes(request):
  data = {}
  for folder in os.listdir(str(USR_DIR)):
    if os.path.isdir(str(USR_DIR/folder)):
      basic = read_info(folder)
      try:
        mapinfo = read_yaml(folder)
        basic['nmps'] = mapinfo['nMappoints']
        basic['nkfs'] = mapinfo['nKeyframes']
      except Exception as e:
        pass
      data[folder] = basic
  
  return web.json_response(data)