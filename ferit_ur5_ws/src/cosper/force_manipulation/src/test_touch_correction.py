import os
import numpy as np
import json

static_depth = a = 0.4
static_side_width = c = 0.005

T_0_W = np.eye(4)
T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')

base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'
door_model_path = os.path.join(base_dir, 'models/doorModel.json')
# Detected model
with open(door_model_path, 'r') as f:
    data = json.load(f)

R = np.array(data["R"])
t = np.array(data["t"])
s = np.array(data["s"])
r = np.array(data["r"])
axis_pos = data["openingDirection"]
T_A_C = np.eye(4)
T_A_C[:3, :3] = R
T_A_C[:3, 3] = t

T_A_6 = T_C_6 @ T_A_C


# Load GT model
gt_poses_path = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/gt_cabinets/cabinet_gt_poses.npy'
gt_poses = np.load(gt_poses_path)
cabinet_idx = 4
T_A_W_gt = gt_poses[cabinet_idx]
width = 0.395
height = 0.495
# b_gt = 
