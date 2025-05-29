import rospy
import os
import numpy as np
import json
import csv
import subprocess
from rospkg import RosPack
from tf2_ros import Buffer, TransformListener
from gazebo_msgs.msg import ContactsState
import sys
from core.util import read_config
from core.ur5_commander import UR5Commander
from core.real_ur5_controller import UR5Controller
from core.transforms import rot_z, rot_y, rot_x, rodrigues_rotate_vector
from gazebo_push_open.cabinet_model import Cabinet
from core.gazebo import get_link_pose
from force_utils import *
# from push_force_trajectories import generate_tool_line_poses, generate_trajectories, generate_trajectories_and_approach
from push_force_trajectories import *
sys.path.append(os.path.join(os.path.dirname(__file__), '../../path_planning/src'))
from utils import contact_callback, kill_processes, reset_tf_buffer
import RVLPYDDManipulator as rvlpy
import threading
import open3d as o3d
sys.path.append(os.path.join(os.path.dirname(__file__), '../../door_detection/src'))
from door_state_detector import DoorStateDetector
from core.image_process import OneShotImageCapture


def create_gt_cabinet_model(load_gt: bool, 
                            gt_door_model_path: str, 
                            gt_door_width_path: str, 
                            cabinet_urdf_save_path: str,
                            robot: Union[UR5Commander, UR5Controller]):
    global door_thickness, static_depth, push_latch_mechanism_length
    T_A_W_gt = np.eye(4)
    width = 0.396
    height = 0.496
    door_thickness = 0.018
    static_depth = 0.4
    axis_pos = -1

    if load_gt:
        # Load ground truth door model from file
        T_A_W_gt = np.load(gt_door_model_path)
        width = np.load(gt_door_width_path)
    else:
        # Calculate the ground truth door model from the points on the front panel
        T_pen_6 = np.eye(4)
        T_pen_6[2, 3] = 0.315 # length of the pen

        T_6_0_pts = np.zeros((2, 4, 4))
        T_6_0_pts[:, 3, 3] = 1.

        for i in range(2):
            print("Place the robot in point %d." % i)
            input("Press Enter to continue...")

            T_6_0 = robot.get_current_tool_pose()
            T_6_0_pts[i, :, :] = T_6_0.copy()

        T_pts_0 = T_6_0_pts @ T_pen_6[np.newaxis, ...]

        # S_0 = S_W
        # Rotation of T_A_W
        z_ = np.array([0, 0, 1])
        y_ = T_pts_0[0, :3, 3] - T_pts_0[1, :3, 3]
        width = np.linalg.norm(y_)
        y_ = np.array(y_ / np.linalg.norm(y_))
        x_ = np.cross(y_, z_)
        T_A_W_gt[:3, 1] = y_.copy()
        T_A_W_gt[:3, 2] = z_.copy()
        T_A_W_gt[:3, 0] = x_.copy()

        # translation of T_A_W
        T_A_W_gt[:3, 3] = T_pts_0[0, :3, 3].copy()
        T_A_W_gt[:3, 3] = T_A_W_gt[:3, 0]*door_thickness*0.5 + T_A_W_gt[:3, 3]
        T_A_W_gt[:3, 3] = -T_A_W_gt[:3, 2]*height*0.5 + T_A_W_gt[:3, 3]

        np.save(gt_door_model_path, T_A_W_gt)
        np.save(gt_door_width_path, width)

        exit(0)

if __name__ == "__main__":
    rospy.init_node('get_ground_truth_pose', anonymous=True)
    rospy.loginfo("Starting get_ground_truth_pose node...")

    base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'

    cabinet_static_mesh_filename = os.path.join(base_dir, 'cabinet_model/cabinet_static_mesh.ply')
    cabinet_panel_mesh_filename = os.path.join(base_dir, 'cabinet_model/cabinet_panel_mesh.ply')
    cabinet_full_mesh_filename = os.path.join(base_dir, 'cabinet_model/cabinet_full_mesh.ply')
    cabinet_urdf_save_path = os.path.join(base_dir, 'cabinet_model/cabinet_model.urdf')
    trajs_path = os.path.join(base_dir, 'trajectories.npy')
    
    cabinet_models_path = os.path.join(base_dir, 'cabinet_model')

    gt_door_model_path = os.path.join(base_dir, 'models/T_A_W_gt.npy')
    gt_door_width_path = os.path.join(base_dir, 'models/width.npy')
    door_model_path = os.path.join(base_dir, 'models/doorModel.json')
    
    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')
    T_0_W = np.eye(4)

    rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_cfg)
    rvl_manipulator.set_robot_pose(T_0_W)
    T_G_6 = rvl_manipulator.get_T_G_6()

    robot = UR5Controller()
    robot.T_C_6 = T_C_6
    robot.T_G_6 = T_G_6
    _, _ = create_gt_cabinet_model(False, gt_door_model_path, gt_door_width_path, cabinet_urdf_save_path=cabinet_urdf_save_path, robot=robot)

    rospy.loginfo("Finished get_ground_truth_pose node.")