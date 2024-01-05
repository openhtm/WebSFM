from pathlib import Path
import subprocess
from multiprocessing import Process, Queue
import os
import json
import logging
import shutil

from SFM.mvs import MVS
from SFM.struct import PointCloud, Structure

logger = logging.getLogger('TaskPipe')

ROOTDIR = Path(__file__).parent.parent
USR_DIR = ROOTDIR/'static/usr'
MVS_BIN_PATH = '/usr/local/bin/OpenMVS'

######################################################################################################################################################
# tasks
class TaskPipe(Process):
  tasks = Queue()

  def __init__(self):
    super().__init__()

  def run(self):
    logger.info('Pipe is running, waiting for new task.')

    while True:
      task = self.tasks.get()
      if task is None:
        break
      logger.info('new task received')
      task.generate().segment().calculate()
      logger.info('task finished')
  
  def add_task(self, base_dir):
    task = Task(base_dir)
    self.tasks.put(task)

class Task:
  base_dir = None
  point_cloud = None

  def __init__(self, base_dir):
    self.base_dir = Path(base_dir).resolve()
  
  def generate(self):
    # generate pointcloud
    mvs = MVS(MVS_BIN_PATH, self.base_dir/'scene')
    mvs.generate()
    return self

  def segment(self):
    logger.info('segmenting')
    # generate structure
    ply_path = self.base_dir/'scene/dense.ply'
    point_cloud = PointCloud().load_ply(ply_path).down_sample()
    self.point_cloud = point_cloud
    plane_model = point_cloud.segment(6)
    # save plane
    filepath = self.base_dir/'map.json'
    data = {}
    with open(str(filepath), 'r') as f:
      data = json.load(f)
    data['plane_model'] = plane_model
    with open(str(filepath), 'w') as f:
      json.dump(data, f)
    return self
  
  def switch_plane(self):
    # dump
    filepath = str(self.base_dir/'map.json')
    plane_model = []
    data = {}
    with open(filepath, 'r') as f:
      data = json.load(f)
      plane_model = data['plane_model']
    # switch
    p = plane_model[0]
    new_plane_model = plane_model[1:]
    new_plane_model.append(p)
    # save
    data['plane_model'] = new_plane_model
    with open(filepath, 'w') as f:
      json.dump(data, f)
    return self
  
  def calculate(self):
    logger.info('calculating grid map')
    # load pcd
    point_cloud = self.point_cloud if self.point_cloud is not None else PointCloud().load_ply(self.base_dir/'scene/dense.ply').down_sample()
    # dump
    filepath = str(self.base_dir/'map.json')
    plane_model = []
    with open(filepath, 'r') as f:
      data = json.load(f)
      plane_model = data['plane_model']
    Structure(point_cloud.ply, plane_model[0]).flatten().dump(filepath)

TASK_PIPE = TaskPipe()