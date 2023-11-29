from pathlib import Path
import subprocess
import os
import shutil

class MVS:
  def __init__(self, mvs_path, log_dir):
    self.mvs_path = Path(mvs_path)
    
    self.cmd_densify = str(mvs_path/'DensifyPointCloud')
    self.cmd_reconstruct = str(mvs_path/'ReconstructMesh')
    self.cmd_texture = str(mvs_path/'TextureMesh')

    self.code = 0

    self.log_path = Path(log_dir) / 'mvs.log'
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
    code, msg = self.command(self.cmd_densify, input_, output_)
    if code == 0:
      print('successfully densified')
      self.log(1)
    return code == 0
  
  def reconstruct_mesh(self, input_, output_):
    code, msg = self.command(self.cmd_reconstruct, input_, output_)
    if code == 0:
      print('successfully reconstructed')
      self.log(2)
    return code == 0
  
  def texture_mesh(self, input_, output_):
    code, msg = self.command(self.cmd_texture, input_, output_)
    if code == 0:
      print('successfully textured')
      self.log(3)
    return code == 0

  def generate(self, scene):
    base = Path(scene).parent
    tmp = base/'tmp'

    if os.path.exists(tmp):
      shutil.rmtree(tmp)
    
    os.mkdir(tmp)
    os.chdir(tmp)

    dense = base/'dense.mvs'
    mesh = base/'mesh.mvs'
    texture = base/'texture.mvs'

    if self.densify_pcl(scene, dense):
      if self.reconstruct_mesh(dense, mesh):
        if self.texture_mesh(mesh, texture):
          pass

    return self.code

    


