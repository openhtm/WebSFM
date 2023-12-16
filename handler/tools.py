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
    with open(str(USR_DIR/str(uid)/'info.json'), 'w') as f:
      json.dump(data, f)

def append_info(uid, data):
  if uid is not None:
    info = read_info(uid)
    for key in data.keys():
      info[key] = data[key]
    write_info(uid, info)

def read_info(uid):
  if uid is not None and os.path.exists(str(USR_DIR/str(uid)/'info.json')):
    with open(str(USR_DIR/str(uid)/'info.json'), 'r') as f:
      return json.load(f)
  else:
    return {}

def read_yaml(uid):
  if uid is not None and os.path.exists(str(USR_DIR/str(uid)/'map.yaml')):
    with open(str(USR_DIR/str(uid)/'map.yaml'), 'r') as f:
      f.readline()
      return yaml.safe_load(f)
  else:
    return {}

def read_status(uid):
  if uid is not None and os.path.exists(str(USR_DIR/str(uid)/'scene/status.log')):
    with open(str(USR_DIR/str(uid)/'scene/status.log'), 'r') as f:
      status = f.readline()
      return int(status)
  else:
    return -1