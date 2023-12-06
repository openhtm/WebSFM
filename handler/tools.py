# tools 
import json
import yaml
import os
import shutil
from pathlib import Path

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'

def write_info(id, data):
  with open(str(USR_DIR/str(id)/'info.json'), 'w') as f:
    json.dump(data, f)
    f.close()

def read_info(id):
  if os.path.exists(str(USR_DIR/str(id)/'info.json')):
    with open(str(USR_DIR/str(id)/'info.json'), 'r') as f:
      return json.load(f)
  else:
    return {}

def read_yaml(id):
  if os.path.exists(str(USR_DIR/str(id)/'map.yaml')):
    with open(str(USR_DIR/str(id)/'map.yaml'), 'r') as f:
      f.readline()
      return yaml.safe_load(f)
  else:
    return {}