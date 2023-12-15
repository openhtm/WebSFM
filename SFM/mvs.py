from pathlib import Path
import subprocess
from multiprocessing import Process, Queue
import os
import logging
import shutil

logger = logging.getLogger('MVSPipe')

class MVSPipe(Process):
  tasks = Queue()
  exit_required = False

  def __init__(self, mvs_path):
    super().__init__()
    self.mvs_path = Path(mvs_path)
  
  def run(self):
    logger.info('Pipe is running, waiting for new task.')

    while not self.exit_required:
      task = self.tasks.get()
      if task is None:
        break

      logger.info('new task received')
      if isinstance(task, MVS):
        logger.info(f'running mvs task {task.scene}')
        task.generate()

  def add_task(self, scene):
    if self.exit_required:
      logger.error('mvs pipe is closed')
      return
    
    task = MVS(self.mvs_path, scene)
    self.tasks.put(task)

  def release(self):
    logger.info('Pipe will be release')
    self.tasks.put(None)
    self.exit_required = True

  def terminate(self):
    logger.info('Pipe terminated')
    self.exit_required = True
    super().terminate()


class MVS:
  def __init__(self, mvs_path, scene):
    self.mvs_path = Path(mvs_path)
    
    self.cmd_densify = str(self.mvs_path/'DensifyPointCloud')
    self.cmd_reconstruct = str(self.mvs_path/'ReconstructMesh')
    self.cmd_texture = str(self.mvs_path/'TextureMesh')

    self.code = 0
    self.scene = Path(scene)

    self.log_path = Path(scene).parent / 'status.log'
    self.log(0)

  def log(self, code):
    self.code = code
    with open(self.log_path, 'w') as f:
      f.write(str(code))

  def command(self, run_, input_, output_):
    input_ = str(input_)
    output_ = str(output_)
    
    cmd = f'{run_} -i {input_} -o {output_}'
    
    p = subprocess.Popen(cmd, shell=True, encoding="utf-8",
      stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )

    p.wait()

    if p.returncode != 0:
      self.log(-1)
      return p.returncode, str(p.stderr.read())
    
    return p.returncode, str(p.stdout.read())

  def densify_pcl(self, input_, output_):
    if self.code == -1:
      return False

    err, msg = self.command(self.cmd_densify, input_, output_)
    if not err:
      logger.info('successfully densified')
      self.log(1)
    else:
      logger.error('error occured while densifying')
      self.log(-1)
    return err == 0
  
  # reconstruct mesh from dense pointclouds
  def reconstruct_mesh(self, input_, output_):
    if self.code == -1:
      return False

    err, msg = self.command(self.cmd_reconstruct, input_, output_)
    if not err:
      logger.info('successfully reconstructed')
      self.log(2)
    else:
      logger.error('error occured while reconstructing')
      self.log(-2)
    return err == 0
  
  # texture mesh from raw mesh
  def texture_mesh(self, input_, output_):
    if self.code == -1:
      return False

    err, msg = self.command(self.cmd_texture, input_, output_)
    if not err:
      logger.info('successfully textured')
      self.log(3)
    else:
      logger.error('error occured while texturing')
      self.log(-3)
    return err == 0


  # main generate function
  def generate(self, scene_path = None, cache = False):
    scene = scene_path if scene_path else self.scene

    base = Path(scene).parent
    tmp = base/'tmp'

    if os.path.exists(tmp):
      shutil.rmtree(tmp)
    
    os.mkdir(tmp)
    os.chdir(tmp)

    dense = base/'dense.mvs'
    mesh = base/'mesh.mvs'
    texture = base/'texture.mvs'

    logger.info(f'generating for {scene}')

    res = self.densify_pcl(scene, dense) and self.reconstruct_mesh(dense, mesh) and self.texture_mesh(mesh, texture)    
    
    if not cache:
      for filename in os.listdir(tmp):
        if filename.endswith('.dmap'):
          os.remove(tmp/filename)

    return self.code

    


