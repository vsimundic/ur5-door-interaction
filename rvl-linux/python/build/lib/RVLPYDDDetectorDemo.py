import numpy as np
import os

import RVLPYDDDetector as rvl

print('Starting...')

detector = rvl.PYDDDetector()
detector.create('/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg')
scene_sequence_dir_name = '/home/RVLuser/rvl-linux/data/01_door_human_2022-11-23-10-02-41'
ply_dir_name = 'PLY_seg'
print('Loading mesh sequence %s' % scene_sequence_dir_name)
first_mesh_id = 35
last_mesh_id = 67

for mesh_id in range(first_mesh_id, last_mesh_id):
    mesh_file_name = f'{mesh_id:04d}' + '.ply'
    print(mesh_file_name)
    mesh_path = os.path.join(scene_sequence_dir_name, ply_dir_name, mesh_file_name)
    print(mesh_path)
    detector.add_mesh(mesh_path)

num_meshes = last_mesh_id - first_mesh_id + 1

rgb_dir_name = 'rgb'
print('Loading rgb sequence')
for mesh_id in range(first_mesh_id, last_mesh_id + 1):
    rgb_file_name = f'{mesh_id:04d}' + '.png'
    print(rgb_file_name)
    rgb_path = mesh_path = os.path.join(scene_sequence_dir_name, rgb_dir_name, mesh_file_name)
    detector.add_rgb(rgb_path)

# hyp_file_name = os.path.join(scene_sequence_dir_name, 'hyps.txt')
# detector.set_hyp_file_name(hyp_file_name)
ao = detector.detect()
print(ao)

print('completed.')