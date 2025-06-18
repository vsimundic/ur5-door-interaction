import rospy
import numpy as np
import os
import pickle   

trajectories_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/gt_cabinets/cabinet_gt_trajectories.pkl'
T_A_W_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/gt_cabinets/cabinet_gt_poses.npy'

door_thickness = 0.018
width = 0.395
height = 0.495

T_A_W_cabinets = np.load(T_A_W_path)
T_A_W_cabinets = T_A_W_cabinets.reshape(-1, 4, 4)

with open(trajectories_path, 'rb') as f:
    trajectories_per_pose = pickle.load(f)

cabinet_idx = 2

# rospy.init_node('set_cabinet_on_pose', anonymous=True)

# robot = UR5Controller()

T_A_W = T_A_W_cabinets[cabinet_idx]

T_corner1_A = np.eye(4)
T_corner1_A[:3, 3] = np.array([-door_thickness*0.5, 0.0, height*0.5])

T_corner2_A = np.eye(4)
T_corner2_A[:3, 3] = np.array([-door_thickness*0.5, -width, height*0.5])

T_corner1_W = T_A_W @ T_corner1_A
T_corner1_W[:3, 3] *= 1000. # convert to mm
T_corner2_W = T_A_W @ T_corner2_A
T_corner2_W[:3, 3] *= 1000. # convert to mm

print('Topleft corner: %.2f %.2f %.2f' % (-T_corner1_W[0, 3], -T_corner1_W[1, 3], T_corner1_W[2, 3]))

print('Topright corner: %.2f %.2f %.2f' % (-T_corner2_W[0, 3], -T_corner2_W[1, 3], T_corner2_W[2, 3]))