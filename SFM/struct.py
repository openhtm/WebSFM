from pathlib import Path
import subprocess
from multiprocessing import Process, Queue
import os
import logging
import shutil
import json
import open3d as o3d
import numpy as np

logger = logging.getLogger('Structure')

class Structure:
  ply = None
  plane_model = None
  transform_twp = None
  x_range = []
  z_range = []
  y_max = []
  grid = [[]]

  def __init__(self, ply, plane):
    self.ply = ply
    self.plane_model = plane
  
  def calculate_transform(self):
    [a, b, c, d] = self.plane_model
    # plane-based coordinate
    center = [0, 0, -d/c]
    vec1_end = [1, 0, (-d-a)/c]
    vec1 = np.array(vec1_end) - np.array(center) # x
    vec1 = vec1 / np.linalg.norm(vec1)
    vec2 = np.array([a, b, c])
    vec2 = vec2 / np.linalg.norm(vec2) # y
    # cross
    vec3 = np.cross(vec1, vec2) # z
    vec3 = vec3 / np.linalg.norm(vec3)
    # transform = [X, Y, Z, center] (Twp)
    transform = np.eye(4)
    transform[:3, 0] = vec1
    transform[:3, 1] = vec2
    transform[:3, 2] = vec3
    transform[:3, 3] = center
    self.transform_twp = transform
    self.transform_tpw = np.linalg.inv(transform)
    return self

  # flatten to grid
  def flatten(self):
    self.calculate_transform()
    # multiply by tpw
    pcd = self.ply.transform(self.transform_tpw)
    # get points
    points = np.asarray(pcd.points)
    # filter
    points = points[points[:, 1] > 0]
    mean_y = np.mean(points[:, 1])
    points = points[points[:, 1] > 0.05]
    points = points[points[:, 1] < (mean_y + 0.1)]
    # # flat
    flatted = points[:, [0, 2]]
    min_x, max_x = np.min(flatted[:, 0]), np.max(flatted[:, 0])
    min_z, max_z = np.min(flatted[:, 1]), np.max(flatted[:, 1])
    # gridding
    offset_points = flatted - [min_x, min_z]
    # division = 96
    grid_size = 0.05
    div_x = int((max_x - min_x) / grid_size) + 1
    div_z = int((max_z - min_z) / grid_size) + 1
    self.grid = np.ones((div_z, div_x))
    # set grid
    for p in offset_points:
      x = int(p[0] / grid_size)
      z = int(p[1] / grid_size)
      self.grid[z][x] = 0

    self.x_range = [min_x, max_x]
    self.z_range = [min_z, max_z]
    self.y_max = mean_y * 2
    return self
  
  # dump to file
  def dump(self, filepath):
    filepath = str(filepath)
    data = {}
    with open(str(filepath), 'r') as f:
      data = json.load(f)
    data['x_range'] = self.x_range
    data['z_range'] = self.z_range
    data['y_max'] = self.y_max
    data['Twp'] = self.transform_twp.tolist()
    data['Tpw'] = self.transform_tpw.tolist()
    data['grid'] = self.grid.tolist()
    with open(str(filepath), 'w') as f:
      json.dump(data, f)


class PointCloud:
  ply = None

  def load_ply(self, path):
    self.ply = o3d.io.read_point_cloud(str(path))
    return self
  
  def down_sample(self, voxel_size=0.01):
    self.ply = self.ply.voxel_down_sample(voxel_size=voxel_size)
    return self
  
  def segment_planes(self, segment_num=1, visualize=False):
    plane_models = []
    inlier_points = []
    input_ply = self.ply
    for i in range(0, segment_num):
      plane_model, inliers = input_ply.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
      plane_models.append(plane_model)
      # get inlier points if visualize
      if visualize: inlier_points.append(input_ply.select_by_index(inliers))
      input_ply = input_ply.select_by_index(inliers, invert=True)
    # visualize segments
    if visualize:
      for e in inlier_points:
        e.paint_uniform_color(np.random.rand(3))
      o3d.visualization.draw_geometries(inlier_points)
    
    return plane_models, inlier_points
    
  # generate structures
  def segment(self, segment_num):
    plane_model, _ = self.segment_planes(segment_num)
    return [m.tolist() for m in plane_model]


# if __name__ == '__main__':
#   uid = 'd8d2f3d2-a28c-11ee-9a3c-23029bf31f2a'
#   s = Structure().load_ply(USR_DIR/str(uid)/'scene/dense.ply')
#   models, inliers = s.segment_planes(4, True)
#   # draw
#   # geo = [s.ply]
#   geo = []
#   for i in range(0, len(models)):
#     # random color
#     inliers[i].paint_uniform_color(np.random.rand(3))
#     geo.append(inliers[i])
#   o3d.visualization.draw_geometries(geo)


# ROOTDIR = Path(__file__).parent.parent
# USR_DIR = ROOTDIR/'static/usr'

# class StructurePipe(Process):
#   tasks = Queue()
#   exit_required = False

#   def __init__(self):
#     super().__init__()
  
#   def run(self):
#     logger.info('Pipe is running, waiting for new task.')

#     while not self.exit_required:
#       task = self.tasks.get()
#       if task is None:
#         break

#       logger.info('new task received')
#       if isinstance(task, Structure):
#         logger.info(f'running structure task {task.uid}')
#         task.generate()

#   def add_task(self, uid):
#     if self.exit_required:
#       logger.error('structure pipe is closed')
#       return
    
#     task = Structure(uid)
#     self.tasks.put(task)

#   def release(self):
#     logger.info('Pipe will be release')
#     self.tasks.put(None)
#     self.exit_required = True


# class Structuret:
#   pcd = None
#   plane_model = None

#   def __init__(self, uid):
#     self.uid = uid
#     self.pcd_file = str(Path(USR_DIR/str(uid)/'scene/dense.ply'))
#     if os.path.exists(self.pcd_file):
#       self.pcd = o3d.io.read_point_cloud(str(self.pcd_file))
#       # downsample
#       # self.pcd = self.pcd.voxel_down_sample(voxel_size=0.01)
#     else:
#       raise Exception(f'pcd file {self.pcd_file} not found')
    
#   def test(self):
#     self.plane_model = [-0.007388115688878947, 0.9825640044119996, 0.18577780540318545, 0.2576701224303377]
#     append_info(self.uid, {'plane': self.plane_model})

#   def segment(self):
#     # segment plane
#     plane_model, inliers = self.pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
#     self.plane_model = plane_model
#     [a, b, c, d] = self.plane_model # ax + by + cz + d = 0
#     print(f'plane model: {a}x + {b}y + {c}z + {d} = 0')

#     append_info(self.uid, {'plane': self.plane_model})
#     # [a, b, c, d] = self.plane_model # ax + by + cz + d = 0
#     # print(f'plane model: {a}x + {b}y + {c}z + {d} = 0')
#     # set inliers
#     # inlier_cloud = self.pcd.select_by_index(inliers)
#     # inlier_cloud.paint_uniform_color([1.0, 0, 0])
#     # self.inliers = inlier_cloud
#     # o3d.visualization.draw_geometries([self.pcd, self.inliers])

#   def calculate(self):
#     [a, b, c, d] = self.plane_model # ax + by + cz + d = 0
#     # plane-based coordinate
#     center = [0, 0, -d/c]
#     vec1_end = [1, 0, (-d-a)/c]
#     vec1 = np.array(vec1_end) - np.array(center) # x
#     vec1 = vec1 / np.linalg.norm(vec1)
#     vec2 = np.array([a, b, c])
#     vec2 = vec2 / np.linalg.norm(vec2) # y
#     # cross
#     vec3 = np.cross(vec1, vec2) # z
#     vec3 = vec3 / np.linalg.norm(vec3)
#     # Twp = [X, Y, Z, center]
#     Twp = np.eye(4)
#     Twp[:3, 0] = vec1
#     Twp[:3, 1] = vec2
#     Twp[:3, 2] = vec3
#     Twp[:3, 3] = center
#     self.Twp = Twp
#     self.Tpw = np.linalg.inv(Twp)
#     append_info(self.uid, {'Twp': self.Twp.tolist(), 'Tpw': self.Tpw.tolist()})
    
#   def flatten(self):
#     # multiply by tpw
#     pcd = self.pcd.transform(self.Tpw)
#     # get points
#     points = np.asarray(pcd.points)
#     # filter
#     points = points[points[:, 1] > 0]
#     mean_y = np.mean(points[:, 1])
#     # points = points[abs(points[:, 2]) > min_]
#     # points = points[abs(points[:, 2]) < max_]
#     # if not negative_:
#     #   points = points[points[:, 2] > 0]
#     points = points[points[:, 1] > (mean_y - 0.1)]
#     points = points[points[:, 1] < (mean_y + 0.1)]
#     # # flat
#     flatted = points[:, [0, 2]]
#     min_x, max_x = np.min(flatted[:, 0]), np.max(flatted[:, 0])
#     min_z, max_z = np.min(flatted[:, 1]), np.max(flatted[:, 1])
#     # gridding
#     offset_points = flatted - [min_x, min_z]
#     # division = 96
#     grid_size = 0.05
#     div_x = int((max_x - min_x) / grid_size) + 1
#     div_z = int((max_z - min_z) / grid_size) + 1
#     grid = np.ones((div_z, div_x))
#     # set grid
#     for p in offset_points:
#       x = int(p[0] / grid_size)
#       z = int(p[1] / grid_size)
#       grid[z][x] = 0
    
#     # save
#     append_info(self.uid, {'x_range':[min_x, max_x], 'z_range':[min_z, max_z], 'y_max':mean_y * 2, 'grid': grid.tolist()})

#     # plt.imshow(grid, cmap='viridis', origin='lower', extent=[-10, 10, -10, 10])
#     # plt.colorbar(label='Point Density')  # 添加颜色条，用于表示点密度
#     # plt.title('Grid Map')
#     # plt.xlabel('X')
#     # plt.ylabel('Y')
#     # plt.show()

#     # box = o3d.geometry.TriangleMesh.create_box(depth=5, width=5, height=0.01)
#     # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
#     # o3d.visualization.draw_geometries([self.pcd, coordinate_frame])
  
#   def generate(self):
#     self.segment()
#     self.calculate()
#     self.flatten()

# STRUCTURE_PIPE = StructurePipe()
# if __name__ == '__main__':
#   uid = 'd8d2f3d2-a28c-11ee-9a3c-23029bf31f2a'
#   s = Structure(uid)
#   s.test()
#   #s.segment()
#   s.calculate()
#   s.flatten()