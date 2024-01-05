# tools 
import json
import yaml
import os
import shutil
from pathlib import Path

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'

def write_info(uid, data):
  if uid is not None:
    with open(str(USR_DIR/str(uid)/'map.json'), 'w') as f:
      json.dump(data, f)

def append_info(uid, data):
  if uid is not None:
    info = read_info(uid)
    for key in data.keys():
      info[key] = data[key]
    write_info(uid, info)

def read_info(uid):
  if uid is not None and os.path.exists(str(USR_DIR/str(uid)/'map.json')):
    with open(str(USR_DIR/str(uid)/'map.json'), 'r') as f:
      return json.load(f)
  # not found
  return {}

def get_first_image(uid):
  folder = str(USR_DIR/str(uid)/'images')
  if uid is not None and os.path.exists(folder):
    all_files = os.listdir(folder)
    for file_name in all_files:
        if file_name.lower().endswith(('.png')):
            return os.path.join('/static/usr', uid, 'images', file_name)
    return os.listdir(str(USR_DIR/str(uid)/'images'))
  # not found
  return ''

def read_status(uid):
  if uid is not None and os.path.exists(str(USR_DIR/str(uid)/'scene/status.log')):
    with open(str(USR_DIR/str(uid)/'scene/status.log'), 'r') as f:
      status = f.readline()
      return int(status)
  # not found
  return -1