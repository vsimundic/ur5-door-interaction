#!/usr/bin/python

import rospy
import os, sys
import numpy as np
import json
import threading
from tf2_ros import Buffer, TransformListener
from tf.transformations import quaternion_matrix
from core.real_ur5_controller import UR5Controller
from core.image_process import OneShotImageCapture
from core.util import read_csv_DataFrame
sys.path.append(os.path.join(os.path.dirname(__file__), '../../door_detection/src'))
from door_state_detector import DoorStateDetector
from door_detection.srv import DetectDoor, DetectDoorState
from push_force_trajectories import *
from force_utils import *
from gazebo_push_open.cabinet_model2 import Cabinet2
from cabinet_adapter import CabinetAdapter
import RVLPYDDManipulator as rvlpy
import pickle
import open3d as o3d
from enum import Enum
from typing import List, Tuple
from collections import defaultdict
import matplotlib.pyplot as plt
import csv
import yaml

np.random.seed(12345)


class FSMState(Enum):
    INITIALIZE             = "INITIALIZE"
    PLAN_TRAJECTORY        = "PLAN_TRAJECTORY"
    APPROACH_PATH          = "APPROACH_PATH"
    INSERTION_PATH         = "INSERTION_PATH"
    OPENING_PATH           = "OPENING_PATH"
    EXIT                   = "EXIT"


class DoorReplanningFSM:
    def __init__(self, config: dict):
        rospy.init_node('door_replanning_fsm_node')

        self.timestamp = rospy.get_time()
        # Initialize ROS utilities
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # self.IS_OFFLINE = config.get("is_offline", False)
        self.CORRECT_ON_TOUCH = config.get("correct_on_touch", False)
        self.LOAD_SESSION = config.get("load_session", False)
        self.grasp_one_finger = config.get("grasp_one_finger", True)

        # Paths and configurations
        self.base_dir = config.get("base_dir", None)
        assert self.base_dir is not None, "Base directory must be specified in the configuration."
        # Restrict the scenes to the cabinets that fully succeeded in simulation
        # and keep their configurations (pose + dimensions) to build the GT models.
        self.multi_contact_simulation_results_path = config.get("multi_contact_simulation_results_path", None)
        assert self.multi_contact_simulation_results_path is not None, "Multi-contact simulation results path must be specified in the configuration."
        self.simulation_results = self.load_successful_simulation_results()
        self.scene_indices = self.simulation_results.index.astype(int).tolist()

        self.gt_idx = 0
        self.offline_gt_idx = 0
        self.scene_idx = 0
        self.session_idx = 0
        self.idx_to_save = 0
        self.touch_session_set = False

        self.door_detector_config_path = config.get("door_detector_config_path", None)
        self.rvl_ddmanipulator_cfg = config.get("rvl_ddmanipulator_cfg", None)
        self.rvl_touch_cfg = config.get("rvl_touch_cfg", None)
        assert self.door_detector_config_path is not None, "Door detector config path must be specified in the configuration."
        assert self.rvl_ddmanipulator_cfg is not None, "RVL DDManipulator config path must be specified in the configuration."
        assert self.rvl_touch_cfg is not None, "RVL Touch config path must be specified in the configuration."

        # Reuse a previously saved detection instead of re-detecting when available.
        self.load_existing_models = config.get("load_existing_models", True)

        self.rvl_manipulator = rvlpy.PYDDManipulator()
        self.rvl_manipulator.create(self.rvl_ddmanipulator_cfg)
        # self.rvl_manipulator.set_robot_pose(self.robot.T_0_W)
        # Touch initialization
        self.py_touch = self.rvl_manipulator.py_touch
        self.py_touch.create(self.rvl_touch_cfg)
        self.py_touch.set_results_folder(config.get('rvl_log_dir', './results'))

        # Separate instance for multi-contact path planning (path2) only.
        self.rvl_manipulator_mc = rvlpy.PYDDManipulator()
        self.rvl_manipulator_mc.create(self.rvl_ddmanipulator_cfg)

        # Default cabinet parameters
        self.door_thickness = config.get("door_thickness", 0.018)
        self.static_depth = config.get("static_depth", 0.4)
        self.max_width = config.get("max_width", 0.6)
        self.max_height = config.get("max_height", 0.8)
        self.gt_height = config.get("gt_height", 0.495)
        self.gt_width = config.get("gt_width", 0.395)

        # Cabinet model
        self.cabinet_model = None
        self.state_angle = None
        self.opening_angle = config.get("opening_angle", -90.0)
        self.push_latch_mechanism_length = config.get("push_latch_mechanism_length", 0.05)
        self.latch_offset = config.get("latch_offset", 0.023)
        self.axis_offset = config.get("axis_offset", 0.01)
        self.moving_to_static_part_distance = config.get("moving_to_static_part_distance", 0.005)
        self.side_width = config.get("side_width", 0.018)
        self.T_6_0_capture = np.eye(4)


        # === Camera ===
        camera_cfg = config.get("camera", None)

        # Image capture parameters
        self.rgb_topic = camera_cfg.get("rgb_topic", '/camera/color/image_raw')
        self.depth_topic = camera_cfg.get("depth_topic", '/camera/aligned_depth_to_color/image_raw')
        self.camera_info_topic = camera_cfg.get("camera_info_topic", '/camera/color/camera_info')
        self.depth_encoding = camera_cfg.get("depth_encoding", '16UC1')

        camera_fu = camera_cfg.get("fu", 597.9033203125)
        camera_fv = camera_cfg.get("fv", 598.47998046875)
        camera_uc = camera_cfg.get("uc", 323.8436584472656)
        camera_vc = camera_cfg.get("vc", 236.32774353027344)
        camera_w = camera_cfg.get("width", 640)
        camera_h = camera_cfg.get("height", 480)
        self.py_touch.set_camera_params(camera_fu, camera_fv, camera_uc, camera_vc, camera_w, camera_h)

        # Camera calibration
        T_C_6_path = config.get("T_C_6_path", None)
        self.T_C_6 = np.load(T_C_6_path) if T_C_6_path else None
        assert self.T_C_6 is not None, "T_C_6 path must be specified in the configuration."
        # Unscale the T_C_6 matrix
        R_C_6 = self.T_C_6[:3, :3]
        I_ = R_C_6 @ R_C_6.T
        mean_trace = np.trace(I_) / 3.0
        self.scale_factor = np.sqrt(mean_trace)
        self.T_C_6[:3, :3] /= self.scale_factor
        self.T_C_6_init = self.T_C_6.copy()

        # === Robot and manipulator ===
        self.robot = UR5Controller()
        self.robot.T_C_6 = self.T_C_6
        self.robot.T_G_6 = np.load(config.get("T_G_6_path", None))
        
        # Tz_180 = np.eye(4)
        # Tz_180[:3, :3] = rot_z(np.pi)
        
        # self.robot.T_G_6 = self.robot.T_G_6 @ Tz_180
        # self.robot.T_G_6[2, 3] = 0.0694
        # np.save(config.get("T_G_6_path", None), self.robot.T_G_6)
        # # Get transform between robotiq_3f_frame and tool0
        # try:
        #     ts = self.tf_buffer.lookup_transform(
        #         'tool0', 'robotiq_3f_frame', rospy.Time(0), rospy.Duration(5.0)
        #     )
        #     q = ts.transform.rotation
        #     t = ts.transform.translation
        #     self.T_tool0_robotiq3f = quaternion_matrix([q.x, q.y, q.z, q.w])
        #     self.T_tool0_robotiq3f[:3, 3] = [t.x, t.y, t.z]
        # except Exception as e:
        #     rospy.logerr("Failed to get transform tool0 -> robotiq_3f_frame: {}".format(e))
        #     self.T_tool0_robotiq3f = None

        self.robot.T_0_W = np.eye(4)
        self.home_q = None

        # === Tactile sensor parameters ===
        self.tactile_sensor_depth = config.get("tactile_sensor_depth", 0.006)
        self.tactile_sensor_width = config.get("tactile_sensor_width", 0.023)

        # === Tool fingertip parameters ===
        tool_cfg = config.get("tool", None) #type: dict
        assert tool_cfg is not None, "Tool configuration must be specified in the configuration."
        tx_E = tool_cfg.get("tx_E", 0.155/2 - self.tactile_sensor_depth)
        tz_E = tool_cfg.get("tz_E", 0.278)

        a_tool = tool_cfg.get("a", 0.0205)
        b_tool = tool_cfg.get("b", 0.032)
        c_tool = tool_cfg.get("c", 0.011)
        d_tool = tool_cfg.get("d", 0.026)
        h_tool = tool_cfg.get("h", 0.023)
        
        rotz_angle = tool_cfg.get("rotz_angle", -np.pi * 0.25) # to get the tcp on the one finger instead of two
        
        rvl_tool = RVLTool(a_tool, b_tool, c_tool, d_tool, h_tool, -tx_E, tz_E, rot_z_angle=rotz_angle, rot_y_angle=0.0)
        self.T_TCP_6 = rvl_tool.T_TCP_6 #type: np.ndarray
        self.py_touch.create_simple_tool(a_tool, b_tool, c_tool, d_tool, h_tool, rvl_tool.T_tool_6)
        # np.save(tool_cfg.get("T_tool_6_path", None), self.T_tool_6)

        # === Touch cfg ===
        touch_cfg = config.get("touch", None)

        self.touch_a = touch_cfg.get("a", self.static_depth)
        self.touch_b = touch_cfg.get("b", 0.0)
        self.touch_c = touch_cfg.get("c", 0.005)

        self.cabinets_estimation_filename = config.get("cabinets_estimation_filename", None)
        self.cabinets_estimation_filename_gt = config.get("cabinets_estimation_filename_gt", None)
        self.cabinets_touches_filename = config.get("cabinets_touches_filename", None)
        assert self.cabinets_estimation_filename is not None, "Cabinet estimation filename must be specified in the configuration."
        assert self.cabinets_estimation_filename_gt is not None, "Cabinet ground truth filename must be specified in the configuration."
        assert self.cabinets_touches_filename is not None, "Touches filename must be specified in the configuration."

        # FSM states
        self.state = FSMState.INITIALIZE

    def initialize_for_scene(self, scene_idx):
        self.scene_idx = scene_idx
        rospy.loginfo(f"[FSM] Initializing for pose {self.scene_idx}...")

        # Path setup
        self.base_detection_dir = os.path.join(self.base_dir, 'door_detection', 'cabinet_{}'.format(self.scene_idx))
        self.rvl_data_path = os.path.join(self.base_detection_dir, 'RVL_data')
        if not os.path.exists(self.rvl_data_path):
            os.makedirs(self.rvl_data_path)
            
        self.door_model_path = os.path.join(self.base_detection_dir, 'models/doorModel.json')
        cabinet_mesh_dir = os.path.join(self.base_detection_dir, 'cabinet_mesh_model')
        if not os.path.exists(cabinet_mesh_dir):
            os.makedirs(cabinet_mesh_dir)
        self.cabinet_static_mesh_path = os.path.join(cabinet_mesh_dir, 'cabinet_static.ply')
        self.cabinet_panel_mesh_path = os.path.join(cabinet_mesh_dir, 'cabinet_panel.ply')
        self.cabinet_mesh_path = os.path.join(cabinet_mesh_dir, 'cabinet_mesh.ply')
        
        self.best_hyp_path = os.path.join(self.base_detection_dir, 'DDT.txt')
        
        # Door-state detection directory (images + detected_state.json).
        # state_detection_dir is the relative name handed to the detect_door_state
        # service (joined with base_detection_dir).
        self.state_detection_dir = 'state_detection'
        self.detect_state_save_dir = os.path.join(self.base_detection_dir, self.state_detection_dir)
        if not os.path.exists(self.detect_state_save_dir):
            os.makedirs(self.detect_state_save_dir)
        self.detected_state_img_path = os.path.join(self.detect_state_save_dir, 'detected_rgb.png')
        self.detected_state_path = os.path.join(self.detect_state_save_dir, 'detected_state.json')


    def initialize(self, is_new_scene=False):
        rospy.loginfo("[FSM] Initializing...")

        if is_new_scene:
            # TODO: if load_session is called, then we do not detect doors. 
            
            # Place cabinet model in the scene
            self.create_gt_cabinet_model()
            self.place_cabinet_model()
            self.detect_door()
            self.state_angle = self.detect_door_state()
            self.create_detected_cabinet_model(self.door_model_path)
        
        # if not self.touch_session_set:
        #     # Set DDManipulator parameters
        #     self.set_rvl_manipulator()


    def set_rvl_manipulator(self, load_fcl_meshes=True):
        self.rvl_manipulator.set_robot_pose(self.robot.T_0_W)
        self.rvl_manipulator.set_door_model_params(
            self.cabinet_model.sx,
            self.cabinet_model.sy,
            self.cabinet_model.sz,
            self.cabinet_model.rx,
            self.cabinet_model.ry,
            self.cabinet_model.axis_pos,
            self.cabinet_model.side,
            self.cabinet_model.moving_to_static_part_distance)
        self.rvl_manipulator.set_door_pose(self.cabinet_model.T_A_W)
        self.rvl_manipulator.set_environment_state(self.state_angle)
        if load_fcl_meshes:
            self.rvl_manipulator.load_cabinet_static_mesh_fcl(self.cabinet_static_mesh_path)
            self.rvl_manipulator.load_cabinet_panel_mesh_fcl(self.cabinet_panel_mesh_path)


    def set_rvl_manipulator_mc(self):
        # Push the current cabinet model / robot pose into the multi-contact
        # planning instance.
        self.rvl_manipulator_mc.set_robot_pose(self.robot.T_0_W)
        self.rvl_manipulator_mc.set_door_model_params(
            self.cabinet_model.sx,
            self.cabinet_model.sy,
            self.cabinet_model.sz,
            self.cabinet_model.rx,
            self.cabinet_model.ry,
            self.cabinet_model.axis_pos,
            self.cabinet_model.side,
            self.cabinet_model.moving_to_static_part_distance)
        self.rvl_manipulator_mc.set_door_pose(self.cabinet_model.T_A_W)
        self.rvl_manipulator_mc.set_environment_state(self.state_angle)


    def plan_trajectory_multi_contact(self, num_traj_pts):
        """Plan a door-opening trajectory with the multi-contact planner (path2).

        Returns (trajectory, contact_poses), or (None, None) if no complete path
        is found:
          trajectory    (num_traj_pts+2, 6) joint waypoints; index 0,1 = approach
                        via-points, 2 = contact, 2: = opening sweep.
          contact_poses (num_traj_pts, 4, 4) T_6_0 poses along the opening sweep.
        """
        self.set_rvl_manipulator_mc()

        q_init = np.array(self.robot.get_current_joint_values(), dtype=np.float64)
        q_init[0] += np.pi
        q_init[5] += np.pi

        try:
            T_G_0_array, q = self.rvl_manipulator_mc.path2(
                q_init, float(self.opening_angle), int(num_traj_pts), False)
        except (MemoryError, ValueError, RuntimeError) as e:
            rospy.logerr(f"[FSM] Multi-contact path planning (path2) failed: {e}")
            return None, None

        # path2 returns [q_init, via0, via1, contact, state_1, ...]; the FSM
        # expects [via0, via1, contact, ...] (index 0,1 approach, 2 contact),
        # so require the full 2-via-point path and drop the leading q_init.
        if T_G_0_array.shape[0] != num_traj_pts + 3:
            rospy.logwarn(
                "[FSM] No complete multi-contact path found "
                f"({T_G_0_array.shape[0]} pts, expected {num_traj_pts + 3}).")
            return None, None

        T_G_0_array = T_G_0_array[1:].astype(np.float64)
        q = q[1:].astype(np.float64)
        q[:, 0] -= np.pi
        q[:, 5] -= np.pi

        T_6_G = np.linalg.inv(self.robot.T_G_6)
        T_6_0_array = T_G_0_array @ T_6_G[np.newaxis, ...]

        rospy.loginfo(f"[FSM] Multi-contact path found: {q.shape[0]} waypoints.")
        return q.astype(np.float32), T_6_0_array[2:, ...]


    def zero_sensor(self):
        rospy.loginfo("[FSM] Zeroing force sensor...")
        self.robot.zero_ft_sensor()
        rospy.loginfo("[FSM] Force sensor zeroed.")


    def add_cabinet_model_to_scene(self, state_angle, is_doorless=False, scale_factor=1.0):
        rospy.loginfo("[FSM] Adding cabinet model to scene...")
        # self.cabinet_model.save_door_panel_mesh(self.cabinet_panel_mesh_path)
        self.robot.remove_mesh_from_scene("cabinet_mesh_model")

        TArot_A = np.eye(4)
        TArot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
        T_Arot_W = self.cabinet_model.T_A_W @ TArot_A

        mesh = copy.deepcopy(self.cabinet_model.static_mesh)
        center = np.array([0.0, 0.0, 0.0])

        # mesh.transform(np.linalg.inv(self.cabinet_model.T_A_O))
        self.cabinet_model.change_door_angle(state_angle)
        self.cabinet_model.create_mesh()
        T_O_D = np.linalg.inv(self.cabinet_model.T_D_A) @ np.linalg.inv(self.cabinet_model.T_A_O)

        if is_doorless:
            # Add doorless cabinet model to the scene
            # static_mesh.transform(np.linalg.inv(self.cabinet_model.T_A_O_init))
            # center = np.array([0.0, 0.0, 0.0])
            mesh.scale(scale_factor, center)
            mesh_path = self.cabinet_static_mesh_path
            self.cabinet_model.save_mesh_without_doors(mesh_path, mesh=mesh, pose=T_O_D)
        else:
            mesh = copy.deepcopy(self.cabinet_model.mesh)
            
            mesh.scale(scale_factor, center)
            mesh_path = self.cabinet_mesh_path
            self.cabinet_model.save_mesh(mesh_path, mesh=mesh, pose=T_O_D)

        self.robot.add_mesh_to_scene(mesh_path, "cabinet_mesh_model", self.T_D_S)

        # # Add drawer part box above the cabinet
        # drawer_z = 0.35 * scale_factor
        # drawer_x = self.cabinet_model.static_d * scale_factor
        # drawer_y = (self.cabinet_model.sy + 2 * self.cabinet_model.side + 3 * self.cabinet_model.moving_to_static_part_distance) * scale_factor
        # self.robot.remove_box_from_scene("drawer_part_box")
        # T_O_S = T_Arot_W @ np.linalg.inv(self.cabinet_model.T_A_O)
        # T_center_S = T_O_S.copy()
        # T_center_S[:3, 3] = T_O_S[:3, :3] @ center + T_O_S[:3, 3]
        # T_center_S[:3, 3] += T_center_S[:3, 2] * ((self.cabinet_model.sz * 0.5 + self.cabinet_model.moving_to_static_part_distance + self.cabinet_model.side) * scale_factor + drawer_z * 0.5)
        # self.robot.add_box_to_scene(
        #     "drawer_part_box",
        #     T_center_S,
        #     np.array([drawer_x, drawer_y, drawer_z]))

        rospy.loginfo("[FSM] Cabinet model added to scene.")

    def remove_cabinet_model_from_scene(self):
        rospy.loginfo("[FSM] Removing cabinet model from scene...")
        self.robot.remove_mesh_from_scene("cabinet_mesh_model")
        self.robot.remove_box_from_scene("drawer_part_box")
        rospy.loginfo("[FSM] Cabinet model removed from scene.")

    def INSERTION_PATH(self, insertion_trajectory: np.ndarray, force_threshold=10.0, state_angle=0.0):
        # Zero the sensor
        rospy.loginfo("[FSM] Zeroing the sensor...")
        self.zero_sensor()

        rospy.loginfo("[FSM] Executing insertion path...")

        success = self.execute_with_monitoring(insertion_trajectory, force_threshold=force_threshold, max_velocity=0.05, max_acceleration=0.05)

        skipped_touch = False

        if not success:
            user_input = input('[FSM] Insertion path execution failed. Press 1 to record the touch or 0 to skip: ')
            if user_input == '0':
                # Skip recording
                skipped_touch = True

            rospy.logwarn("[FSM] Insertion path execution failed. Backing up.")
            # backup trajectory
            current_joints = self.robot.get_current_joint_values()
            backup_trajectory = np.array([current_joints, insertion_trajectory[0]])
            _, T_6_0_ft_loss = self.execute_and_remember_joints_on_force_loss(backup_trajectory, drop_delta_treshold=force_threshold-1.0, low_force_threshold=1.0)
            if T_6_0_ft_loss is not None:
                T_6before_0 = self.robot.get_fwd_kinematics_moveit(insertion_trajectory[-2])
                T_6insert_0 = self.robot.get_fwd_kinematics_moveit(insertion_trajectory[-1])
                self.set_touch(T_6_0_ft_loss, 
                            np.array((T_6before_0, T_6insert_0)), 
                            touch_type=TouchType.UNWANTED_TOUCH,
                            state_angle=state_angle)
                if self.CORRECT_ON_TOUCH:
                    self.correct_touch_model()

        return success, skipped_touch


    def correct_touch_model(self):
        rospy.loginfo("[FSM] Correcting touch model...")

        # Correct the model
        self.py_touch.correct()

        self.update_model_from_touch()

    def update_model_from_touch(self):
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(self.state_angle))

        # Update model x to py_touch
        self.rvl_manipulator.update_model_x()

        T_Arot_6_corrected = self.rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
        T_C_6_corrected = self.rvl_manipulator.get_corrected_camera_pose().astype(np.float64)

        if np.any(np.isnan(T_Arot_6_corrected)) or np.any(np.isnan(T_C_6_corrected)):
            rospy.logerr("[FSM] Corrected poses contain NaN values. Sticking to the previous model.")
            return
        
        # Update the poses
        self.T_C_6 = T_C_6_corrected.copy()
        self.T_A_C = np.linalg.inv(self.T_C_6) @ T_Arot_6_corrected @ np.linalg.inv(T_Arot_A)

        T_D_Arot = self.rvl_manipulator.get_corrected_pose_D_Arot().astype(np.float64)
        # self.cabinet_model.T_D_Arot[:3, 3] = T_D_Arot[:3, 3].copy()
        self.cabinet_model.T_D_Arot = T_D_Arot.copy()

        # Update the cabinet model
        self.cabinet_model.T_A_W = self.robot.T_0_W @ self.T_6_0_capture @ self.T_C_6 @ self.T_A_C
        print(f"[FSM] Corrected cabinet model pose: {self.cabinet_model.T_A_W}")

        # self.T_D_S = self.cabinet_model.T_A_W @ T_Arot_A @ self.cabinet_model.T_D_Arot
        T_D_0 = self.rvl_manipulator.get_corrected_pose_D_0().astype(np.float64)

        self.T_D_S = self.robot.T_0_W @ T_D_0
        # self.rvl_manipulator.set_pose_DD_S(self.T_D_S)
        # self.rvl_manipulator.set_environment_from_touch()

    def set_touch(self, T_6contact_0, T_6via_0_pts, touch_type: TouchType, state_angle: float, only_save=False):
        """Set touch in RVL and save the touch information for RVL.
        Args:
            T_6contact_0 (np.ndarray): The contact pose in the robot frame.
            T_6via_0 (np.ndarray): The via pose in the robot frame.
            V (np.ndarray): The unit vector of the approach path.
            touch_type (TouchType): The type of touch.
        """
        T_6contact_6 = np.linalg.inv(self.T_6_0_capture) @ T_6contact_0
        T_6via_6 = np.linalg.inv(self.T_6_0_capture)[np.newaxis, ...] @ T_6via_0_pts
        V = T_6via_6[1, :3, 3] - T_6via_6[0, :3, 3]
        t = np.linalg.norm(V)
        V /= t  # Normalize the vector
        self.save_touches_info_for_rvl(T_6contact_6, V, t, touch_type, state_angle)
        b_miss = True if touch_type == TouchType.MISS else False
        if not only_save:
            self.py_touch.set_touch(T_6contact_6, V, t, b_miss)


    def recapture(self, trajectory):
        rospy.loginfo("[FSM] Replanning...")

        # Get estimated door state
        # Find the closest point in the trajectory

        estimated_angle = self.state_angle
        print(f"[replan] Estimated door angle at contact loss: {estimated_angle:.2f} degrees")

        T_C_W_new = self.robot.T_0_W @ self.T_6_0_capture @ self.robot.T_C_6
        joints_camera_capture = self.robot.get_closest_ik_solution(self.T_6_0_capture, None)
        if joints_camera_capture is None:
            joints_camera_capture = self.joint_values_capture


        # Backup from the current state
        T_6_0_current = self.robot.get_current_tool_pose()
        T_6_0_backup = T_6_0_current.copy()
        T_6_0_backup[:3, 3] = T_6_0_backup[:3, 3] - T_6_0_backup[:3, 2] * 0.05 # move 5 cm in z-direction
        joints_backup = self.robot.get_closest_ik_solution(T_6_0_backup, None) # get ik solution for backup and take current joints
        if joints_backup is None:
            rospy.logerr("[FSM] No valid IK solution for backup. Exiting.")
            input("Move the robot by hand to a valid position and press Enter to continue...")
        else:
            success = False
            while not success:
                backup_trajectory = np.array([self.robot.get_current_joint_values(), joints_backup])
                success = self.execute_with_monitoring(backup_trajectory, force_threshold=30.0, max_velocity=0.1, max_acceleration=0.1)

        # # Add cabinet model to the robot environment in Moveit
        self.cabinet_model.change_door_angle(estimated_angle)
        # self.cabinet_model.save_mesh(self.cabinet_mesh_path)
        # self.robot.remove_mesh_from_scene("cabinet_mesh_model")
        # self.robot.add_mesh_to_scene(self.cabinet_mesh_path, "cabinet_mesh_model", self.cabinet_model.T_A_W)
        self.add_cabinet_model_to_scene(estimated_angle, scale_factor=1.05)

        # Move to the new camera capture pose
        success = False
        while not success:
            trajectory_capture = np.array([self.robot.get_current_joint_values(), joints_camera_capture])
            planned_joints, success = self.robot.plan_to_joint_goals2(trajectory_capture)

        success = self.execute_with_monitoring(planned_joints, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.3)

        # Remove cabinet model from the robot environment
        self.robot.remove_mesh_from_scene("cabinet_mesh_model")

        # Capture door state
        image_capture = OneShotImageCapture(self.detect_state_save_dir, self.rgb_topic, self.depth_topic, self.camera_info_topic, self.depth_encoding)
        rgb_path, _, ply_path = image_capture.capture_single_image_and_save()
        T_C_W_init_capture = self.robot.T_0_W @ self.T_6_0_capture @ self.T_C_6_init
        # T_Cdet_Cdiff = np.linalg.inv(T_C_W_init_capture) @ T_C_W_new
        T_Cdiff_Cdet = np.linalg.inv(T_C_W_new) @ T_C_W_init_capture
        door_state_detector = DoorStateDetector(detector_config_path=self.door_detector_config_path, best_hyp_path=self.best_hyp_path, save_detected_img_path=self.detected_state_img_path)
        self.state_angle = door_state_detector.detect_state(rgb_path, ply_path, T_Cdiff_Cdet)
        self.state_angle = self.state_angle[0] # take the first element
        print(f"[replan] Detected door state: {self.state_angle}")

        # Set new touch scene
        self.set_new_touch_scene(self.state_angle, self.state_angle)
        self.update_model_from_touch()

        # Plan new trajectory
        self.state = FSMState.PLAN_TRAJECTORY


    def save_models_init_info_for_rvl(self):
        if not os.path.exists(self.cabinets_estimation_filename):
            with open(self.cabinets_estimation_filename, 'w') as f:
                header = 'session_idx,scene_idx,load_idx,sx,sy,sz,rx,ry,state_angle,R_A_C,t_A_C,R_C_E,t_C_E,R_E_0,t_E_0'
                f.write(header + '\n')
        else:
            # Get last line and check the session_idx
            with open(self.cabinets_estimation_filename, 'r') as f:
                lines = f.readlines()
                if len(lines) > 1:
                    last_line = lines[-1].strip()
                    self.session_idx = int(last_line.split(',')[0]) + 1
                else:
                    self.session_idx = 0
        if not os.path.exists(self.cabinets_estimation_filename_gt):
            with open(self.cabinets_estimation_filename_gt, 'w') as f:
                header = 'session_idx,scene_idx,sx,sy,sz,rx,ry,state_angle,R_A_S,t_A_S'
                f.write(header + '\n')
        if not os.path.exists(self.cabinets_touches_filename):
            with open(self.cabinets_touches_filename, 'w') as f:
                header = 'session_idx,scene_idx,type,R_Ek_E,t_Ek_E,V,t,state_angle'
                f.write(header + '\n')


    def save_models_info_for_rvl(self):
        # For detected cabinet model, index, dimensions, self.T_A_C, T_C_E and T_E_0_capture are saved
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(self.state_angle))

        with open(self.cabinets_estimation_filename, 'a') as f:
            txt = f'{self.session_idx},{self.idx_to_save},{self.scene_idx},{self.cabinet_model.sx},{self.cabinet_model.sy},{self.cabinet_model.sz},{self.cabinet_model.rx},{self.cabinet_model.ry},{self.state_angle}'
            R_A_C = self.T_A_C_init[:3,:3].reshape(-1)
            t_A_C = self.T_A_C_init[:3, 3].reshape(-1)
            for i in range(9):
                txt += f',{R_A_C[i]}'
            for i in range(3):
                txt += f',{t_A_C[i]}'
            R_C_E = self.T_C_6_init[:3,:3].reshape(-1)
            t_C_E = self.T_C_6_init[:3, 3].reshape(-1)
            for i in range(9):
                txt += f',{R_C_E[i]}'
            for i in range(3):
                txt += f',{t_C_E[i]}'
            R_E_0_capture = self.T_6_0_capture[:3,:3].reshape(-1)
            t_E_0_capture = self.T_6_0_capture[:3, 3].reshape(-1)
            for i in range(9):
                txt += f',{R_E_0_capture[i]}'
            for i in range(3):
                txt += f',{t_E_0_capture[i]}'
            f.write(txt + '\n')
        
        T_Arotgt_A = np.eye(4)
        T_Arotgt_A[:3, :3] = rot_z(np.deg2rad(self.state_angle_gt))
        T_Arot_W = self.cabinet_model_gt.T_A_W @ T_Arot_A

        with open(self.cabinets_estimation_filename_gt, 'a') as f:
            R_A_S_gt = T_Arot_W[:3,:3].reshape(-1)
            t_A_S_gt = T_Arot_W[:3, 3].reshape(-1)
            txt = f'{self.session_idx},{self.idx_to_save},{self.cabinet_model_gt.sx},{self.cabinet_model_gt.sy},{self.cabinet_model_gt.sz},{self.cabinet_model_gt.rx},{self.cabinet_model_gt.ry},{self.state_angle_gt}'
            for i in range(9):
                txt += f',{R_A_S_gt[i]}'
            for i in range(3):
                txt += f',{t_A_S_gt[i]}'
            f.write(txt + '\n')


    def save_touches_info_for_rvl(self, T_Ek_E, V, t, touch_type: TouchType, state_angle):
        """Save touch information for RVL.
        Args:
            T_Ek_E (np.ndarray): The contact pose in the robot frame.
            V (np.ndarray): The unit vector of the approach path.
            touch_type (TouchType): The type of touch.
            state_angle (float): The state angle of the door.
        """
        R_Ek_E = T_Ek_E[:3, :3].reshape(-1)
        t_Ek_E = T_Ek_E[:3, 3].reshape(-1)
        V_flat = V.reshape(-1)
        with open(self.cabinets_touches_filename, 'a') as f:
            txt = f'{self.session_idx},{self.idx_to_save},{touch_type.value}'
            for i in range(9):
                txt += f',{R_Ek_E[i]}'
            for i in range(3):
                txt += f',{t_Ek_E[i]}'
            for i in range(3):
                txt += f',{V_flat[i]}'
            txt += f',{t}'
            txt += f',{state_angle}'
            f.write(txt + '\n')


    def execute_with_monitoring(self, trajectory, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.5):
        """Execute the trajectory with force monitoring."""
        monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(self.robot, force_threshold))
        monitor_thread.start()
        success = self.robot.send_joint_trajectory_action2(trajectory, max_velocity=max_velocity, max_acceleration=max_acceleration)
        monitor_thread.join()
        return success


    def execute_with_monitoring_remember_joints(self, trajectory, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.5):
        collision_joints = []
        monitor_thread = threading.Thread(target=monitor_force_and_cancel_remember_joints, args=(
            self.robot, collision_joints, force_threshold, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        success = self.robot.send_joint_trajectory_action2(trajectory, max_velocity=max_velocity, max_acceleration=max_acceleration)
        monitor_thread.join()
        return success, collision_joints


    def execute_without_monitoring(self, trajectory):
        success = self.robot.send_joint_trajectory_action2(trajectory, max_velocity=0.5, max_acceleration=0.5)
        return success


    def execute_and_remember_joints_on_force_loss(self, trajectory, drop_delta_treshold=2.0, low_force_threshold=1.5):
        ft_loss_joints = [] # reset the joints
        T_6_0_ft_loss = None
        monitor_thread = threading.Thread(target=monitor_force_drop_and_remember_joints, args=(self.robot, ft_loss_joints, drop_delta_treshold, low_force_threshold, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        self.robot.send_joint_trajectory_action2(trajectory, max_velocity=0.05, max_acceleration=0.05)
        monitor_thread.join()
        ft_loss_joints = np.array(ft_loss_joints)
        if ft_loss_joints.shape[0] == 1:
            rospy.logwarn("[FSM] Force loss detected during backup. Remembered joints: %s", ft_loss_joints[0])
            T_6_0_ft_loss = self.robot.get_fwd_kinematics_moveit(ft_loss_joints[0].tolist())
        return ft_loss_joints, T_6_0_ft_loss


    def cabinet_gt_front_corners(self):
        """
        Get the 3D coordinates of the cabinet ground truth model front corners in the robot frame.
        """
        sx, sy, sz = (self.cabinet_model_gt.sx, self.cabinet_model_gt.sy, self.cabinet_model_gt.sz) if type(self.cabinet_model_gt) == Cabinet2 else (self.cabinet_model_gt.d_door, self.cabinet_model_gt.w_door, self.cabinet_model_gt.h_door)
        rx, ry = self.cabinet_model_gt.rx, self.cabinet_model_gt.ry
        axis_dist = self.cabinet_model_gt.axis_distance
        mpt_dist = self.cabinet_model_gt.moving_to_static_part_distance
        side = self.cabinet_model_gt.side

        # relative to S_A frame (hinge) - homogeneous coordinates
        top_front_corner_furniture = np.array([
            [-sx * 0.5, axis_dist + mpt_dist + side, sz * 0.5 + mpt_dist + side, 1],
            [-sx * 0.5, -(sy - axis_dist + mpt_dist + side), sz * 0.5 + mpt_dist + side, 1]
        ])
        
        # relative to robot frame
        T_A_W_gt = self.cabinet_model_gt.T_A_W

        top_front_corner_furniture_robot = (T_A_W_gt @ top_front_corner_furniture.T).T
        return top_front_corner_furniture_robot

    def place_cabinet_model(self):
        """Place the cabinet model in the scene."""
        rospy.loginfo("[FSM] Place the cabinet near the robot...")
        
        # Calculate the cabinet top front corners position
        t_cabinet_W_pts = self.cabinet_gt_front_corners()
        # Rotate around the z-axis by 180 degrees for placing in polyscope
        Tz_180 = np.eye(4)
        Tz_180[:3, :3] = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                            [np.sin(np.pi), np.cos(np.pi), 0],
                            [0, 0, 1]])
        t_cabinet_W_pts_rotated = (Tz_180 @ t_cabinet_W_pts.T).T

        pt1 = t_cabinet_W_pts_rotated[0, :3] * 1000  # Convert to mm
        pt2 = t_cabinet_W_pts_rotated[1, :3] * 1000  # Convert to mm
        
        rospy.loginfo(f"[FSM] Cabinet top front corners: {pt1}, {pt2}")
        rospy.loginfo("[FSM] Navigate the tool tip to the above positions in Polyscope and place the cabinet accordingly. Press Enter when done.")
        input()

    def detect_door(self):
        """Detect the door via the 'detect_door' service and write the result to
        self.door_model_path. Mirrors the INITIALIZE state of
        multi_contact_force_node.py. If load_existing_models is set and a model is
        already saved, the saved detection is kept."""
        os.makedirs(os.path.dirname(self.door_model_path), exist_ok=True)
        if self.load_existing_models and os.path.exists(self.door_model_path):
            rospy.loginfo(f"[FSM] Using existing door model: {self.door_model_path}")
            return

        while not rospy.is_shutdown():
            key = input("[FSM] Press 'c' to detect door via camera: ").strip().lower()
            if key != 'c':
                rospy.logwarn("[FSM] Invalid key. Press 'c'.")
                continue
            try:
                rospy.wait_for_service('detect_door', timeout=5.0)
                detect_door_srv = rospy.ServiceProxy('detect_door', DetectDoor)
                curr_joints = self.robot.get_current_joint_values()
                T_6_0 = self.robot.get_current_tool_pose()
                resp = detect_door_srv(
                    trigger=True,
                    base_dir=self.base_detection_dir,
                    joint_values=curr_joints,
                    T_6_0=T_6_0.flatten().tolist())
                if resp.success:
                    rospy.loginfo("[FSM] Door detected successfully!")
                    with open(self.door_model_path, 'w') as f:
                        f.write(resp.result_json)
                    return
                rospy.logwarn("[FSM] Detection failed. Try again.")
            except rospy.ROSException:
                rospy.logerr("[FSM] 'detect_door' service not available!")
            except rospy.ServiceException as e:
                rospy.logerr(f"[FSM] Detection service call failed: {e}")

    def detect_door_state(self):
        """Detect the door opening angle via the 'detect_door_state' service.
        Mirrors the DETECT_STATE state of multi_contact_force_node.py. Returns the
        angle in degrees. Called right after detect_door(); the detection pose is
        read from the saved door model so the robot can be moved back to it."""
        if self.load_existing_models and os.path.exists(self.detected_state_path):
            try:
                with open(self.detected_state_path, 'r') as f:
                    data = json.load(f)
                state_angle = data.get('state_angle_deg', 0.0)
                rospy.loginfo(f"[FSM] Loaded state angle: {state_angle:.2f} deg")
                return state_angle
            except Exception as e:
                rospy.logwarn(f"[FSM] Failed to load state detection: {e}. Falling back to prompt.")

        while not rospy.is_shutdown():
            key = input(
                "[FSM] Door state: 'd' auto-detect | 'm' manual angle | 's' skip (use 0.0): "
            ).strip().lower()

            if key == 'd':
                try:
                    rospy.wait_for_service('detect_door_state', timeout=5.0)
                    detect_door_state_srv = rospy.ServiceProxy('detect_door_state', DetectDoorState)

                    # Move the robot back to the detection pose so the camera sees the door.
                    with open(self.door_model_path, 'r') as f:
                        joint_values_detection = np.array(json.load(f)["joint_values"])
                    current_joint_values = self.robot.get_current_joint_values()
                    traj_ = np.vstack((np.array(current_joint_values), joint_values_detection))
                    self.execute_with_monitoring(traj_, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.5)

                    resp = detect_door_state_srv(
                        base_dir=self.base_detection_dir,
                        state_detection_dir=self.state_detection_dir,
                        T_Cdetected_Cstate=np.eye(4).reshape(-1).tolist())
                    if resp.success:
                        rospy.loginfo(f"[FSM] Door state detected: {resp.state_angle_deg:.2f} deg")
                        return resp.state_angle_deg
                    rospy.logwarn(f"[FSM] State detection failed: {resp.message}. Try again.")
                except rospy.ROSException:
                    rospy.logerr("[FSM] 'detect_door_state' service not available!")
                except rospy.ServiceException as e:
                    rospy.logerr(f"[FSM] State detection service call failed: {e}")

            elif key == 'm':
                try:
                    state_angle = float(input("[FSM] Enter door angle: ").strip())
                    rospy.loginfo(f"[FSM] Using manual angle: {state_angle:.2f} deg")
                    return state_angle
                except ValueError:
                    rospy.logwarn("[FSM] Invalid number. Try again.")

            elif key == 's':
                rospy.loginfo("[FSM] Skipping state detection. Using 0.0 deg.")
                return 0.0

            else:
                rospy.logwarn("[FSM] Invalid key. Use 'd', 'm', or 's'.")

    def create_detected_cabinet_model(self, data_model_path):
        with open(data_model_path, 'r') as f:
            data = json.load(f)

        R = np.array(data["R"])
        t = np.array(data["t"])
        self.s = np.array(data["s"])
        self.r = np.array(data["r"])
        # self.r = np.array([-self.door_thickness * 0.5, -self.s[0] * 0.5])  # static side position
        axis_pos = data["openingDirection"]
        self.T_A_C = np.eye(4)
        self.T_A_C[:3, :3] = R
        self.T_A_C[:3, 3] = t

        self.T_A_C_init = self.T_A_C.copy()

        # State angle was detected by detect_door_state(), called right before this.
        state_angle = self.state_angle
        joint_values = np.array(data["joint_values"])

        T_6_0 = self.robot.get_fwd_kinematics_moveit(joint_values)
        np.save(os.path.join(self.rvl_data_path, 'T_6_0_capture.npy'), T_6_0)
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
        T_A_6 = self.T_C_6 @ self.T_A_C @ T_Arot_A
        np.save(os.path.join(self.rvl_data_path, 'T_A_6.npy'), T_A_6)
        T_A_W = self.robot.T_0_W @ T_6_0 @ self.T_C_6 @ self.T_A_C
        
        self.cabinet_model = self.create_cabinet_model(
            s=np.array([self.door_thickness, self.s[0], self.s[1], self.static_depth]),
            r=self.r,
            axis_pos=axis_pos,
            T_A_W=T_A_W,
            axis_distance=self.axis_offset,
            moving_to_static_part_distance=self.moving_to_static_part_distance,
            save_path=None,
            has_handle=False
        )

        self.T_6_0_capture = T_6_0
        self.joint_values_capture = joint_values
        self.state_angle = state_angle


    def load_successful_simulation_results(self):
        """Return the simulation result rows for cabinets that fully succeeded.

        A cabinet is considered successful when the simulation results report
        path_found, traj_success, contact_free and door_opened all True. The
        returned DataFrame is indexed by the cabinet 'idx' so each scene's
        configuration (pose + dimensions) can be looked up directly.
        """
        data = read_csv_DataFrame(self.multi_contact_simulation_results_path)
        success_data = data.loc[((data['path_found'] == True) &
                                 (data['traj_success'] == True) &
                                 (data['contact_free'] == True) &
                                 (data['door_opened'] == True))]
        return success_data.set_index('idx', drop=False)

    def create_gt_cabinet_model(self):
        # Read the cabinet pose and dimensions from the simulation results.
        row = self.simulation_results.loc[self.scene_idx]
        width = float(row['door_width'])
        height = float(row['door_height'])
        axis_pos = float(row['axis_pos'])

        # Cabinet pose from the simulation configuration (position + rotation about z).
        T_A_W_gt = np.eye(4)
        T_A_W_gt[:3, 3] = np.array([row['x'], row['y'], row['z']], dtype=float)
        T_A_W_gt[:3, :3] = rot_z(np.radians(float(row['rot_z'])))
        
        # Lower the cabinet by 5mm to ensure it's on the ground (simulation might have it floating slightly)
        T_A_W_gt[:3, 3] -= np.array([0, 0, 0.005])

        self.state_angle_gt = float(row['state_angle'])

        self.cabinet_model_gt = self.create_cabinet_model(
            s=np.array([self.door_thickness, width, height, self.static_depth]),
            r=np.array([0, -width*0.5 + self.axis_offset]),
            axis_pos=axis_pos,
            T_A_W=T_A_W_gt,
            axis_distance=self.axis_offset,
            moving_to_static_part_distance=self.moving_to_static_part_distance,
            save_path=None,
            has_handle=False
        )

    def create_cabinet_model(self, s, r, axis_pos, T_A_W, axis_distance, moving_to_static_part_distance, save_path=None, has_handle=False):
        """Create the cabinet model from data."""
        # CabinetAdapter wraps the TSMC24 Cabinet but exposes the Cabinet2 API.
        # Cabinet uses axis_distance / static_side_width / moving_to_static_part_distance
        # at construction time, so they are passed in (not set post-hoc).
        cabinet_model = CabinetAdapter(s=s, r=r, axis_pos=axis_pos, T_A_W=T_A_W,
                                       save_path=save_path, has_handle=has_handle,
                                       axis_distance=axis_distance,
                                       static_side_width=self.side_width,
                                       moving_to_static_part_distance=moving_to_static_part_distance)
        return cabinet_model

    def go_to_home_position(self):
        """Go to the home position of the robot."""
        if self.home_q is None:
            rospy.logerr("[FSM] Home position is not set. Setting default.")
            self.home_q = np.array([-np.pi/2, -np.pi/2, 0.0, -np.pi/2, 0.0, -3/4 * np.pi])

        # Check if the robot is in the home position
        current_joints = self.robot.get_current_joint_values()
        if np.allclose(current_joints, self.home_q, atol=0.01):
            rospy.loginfo("[FSM] Robot is already in the home position.")
            return
        
        self.add_cabinet_model_to_scene(self.state_angle, scale_factor=1.05)
        home_trajectory = np.vstack((current_joints, self.home_q))
        traj_moveit, success = self.robot.plan_to_joint_goals2(home_trajectory)
        traj = traj_moveit if success else home_trajectory

        for _ in range(2):
            success = self.execute_with_monitoring(traj, force_threshold=30.0, max_velocity=0.5, max_acceleration=0.5)
            if not success:
                input("Failed to go to home position, move the robot to a position and try again. Press Enter to retry...")
                traj = np.vstack((self.robot.get_current_joint_values(), self.home_q))
            else:
                break


    def get_closest_contact_pose_idx(self, contact_poses, T_6_0):
        # Calculate the angle based on the distance from the contact points
        contact_positions = contact_poses[:, :3, 3]
        distances = np.linalg.norm(contact_positions - T_6_0[:3, 3], axis=1)
        closest_idx = np.argmin(distances)
        return closest_idx


    def set_new_touch_scene(self, state_angle, state_angle_gt):
        self.idx_to_save += 1
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))

        T_Arot_A_gt = np.eye(4)
        T_Arot_A_gt[:3, :3] = rot_z(np.deg2rad(state_angle_gt))
        T_Arot_0_gt = self.cabinet_model_gt.T_A_W @ T_Arot_A_gt

        self.py_touch.set_new_scene(self.door_thickness, self.s[0], self.s[1], self.r[0], self.r[1], self.touch_a, self.touch_b, self.touch_c, state_angle, self.T_C_6_init, self.T_A_C_init, self.T_6_0_capture,
                                    self.cabinet_model_gt.sx, self.cabinet_model_gt.sy, self.cabinet_model_gt.sz, self.cabinet_model_gt.rx, self.cabinet_model_gt.ry, state_angle_gt, T_Arot_0_gt)
        self.save_models_info_for_rvl()


    def load_session(self):
        def get_matrix(base, size, row_, col_idx_):
            start = col_idx_[base]
            return np.array(row_[start:start + size], dtype=np.float64)

        def compute_column_indices(header):
            col_idx = {}
            idx = 0
            # while idx < len(header):
            for name in header:
                # name = header[idx]
                if name in {"R_A_C", "R_C_E", "R_E_0", 'R_A_S', 'R_Ek_E'}:
                    col_idx[name] = idx
                    idx += 9
                elif name in {"t_A_C", "t_C_E", "t_E_0", 't_A_S', 't_Ek_E', 'V'}:
                    col_idx[name] = idx
                    idx += 3
                else:
                    col_idx[name] = idx
                    idx += 1
            return col_idx

        idx_to_save = -1
        type_touch = -1
        succ_trajs_all = 0
        is_really_offline = False
        succ_trajs = 0

        with open(self.cabinets_estimation_filename, 'r') as f:
            reader = csv.reader(f)

            header = next(reader)

            # col_idx = {col: idx for idx, col in enumerate(header)}
            col_idx = compute_column_indices(header)

            rows = list(reader)
            if not rows:
                self.session_idx = 0
                raise RuntimeError("No session found, please run the script to create a session.")
        
            last_row = rows[-1]
            self.session_idx = int(last_row[col_idx['session_idx']])  # Load last session
            prev_load_idx = -1
            last_scene_idx = int(last_row[col_idx['scene_idx']])

            for row in rows:
                sess_idx = int(row[col_idx['session_idx']])
                if sess_idx != self.session_idx: # load only last session
                    continue

                scene_idx = int(row[col_idx['scene_idx']])
                load_idx = int(row[col_idx['load_idx']])
                self.initialize_for_scene(load_idx)
                self.initialize(is_new_scene=True)

                if prev_load_idx != load_idx: 
                    succ_trajs = 0

                # is_really_offline = True if load_idx < 3 and self.IS_OFFLINE else False
                # is_really_offline = True if succ_trajs_all < 9 and self.IS_OFFLINE else False

                sx = float(row[col_idx['sx']])
                sy = float(row[col_idx['sy']])
                sz = float(row[col_idx['sz']])

                self.s = np.array([sy, sz])

                rx = float(row[col_idx['rx']])
                ry = float(row[col_idx['ry']])

                self.r = np.array([rx, ry])

                state_angle = float(row[col_idx['state_angle']])
                self.state_angle = state_angle

                R_A_C = get_matrix('R_A_C', 9, row, col_idx).reshape(3, 3)
                t_A_C = get_matrix('t_A_C', 3, row, col_idx)
                R_C_E = get_matrix('R_C_E', 9, row, col_idx).reshape(3, 3)
                t_C_E = get_matrix('t_C_E', 3, row, col_idx)
                R_E_0_capture = get_matrix('R_E_0', 9, row, col_idx).reshape(3, 3)
                t_E_0_capture = get_matrix('t_E_0', 3, row, col_idx)

                self.T_A_C = np.eye(4)
                self.T_A_C[:3, :3] = R_A_C
                self.T_A_C[:3, 3] = t_A_C

                self.T_A_C_init = self.T_A_C.copy()

                T_C_E = np.eye(4)
                T_C_E[:3, :3] = R_C_E
                T_C_E[:3, 3] = t_C_E
                # self.T_C_6 = T_C_E.copy()

                T_E_0_capture = np.eye(4)
                T_E_0_capture[:3, :3] = R_E_0_capture
                T_E_0_capture[:3, 3] = t_E_0_capture

                self.T_6_0_capture = T_E_0_capture.copy()

                T_A_W = self.robot.T_0_W @ T_E_0_capture @ T_C_E @ self.T_A_C
                
                self.cabinet_model = self.create_cabinet_model(
                    s=np.array([self.door_thickness, sy, sz, self.static_depth]),
                    r=np.array([rx, ry]),
                    axis_pos=-1,
                    T_A_W=T_A_W,
                    axis_distance=self.axis_offset,
                    moving_to_static_part_distance=self.moving_to_static_part_distance,
                    save_path=None,
                    has_handle=False
                )
                
                # Load GT cabinet model
                with open(self.cabinets_estimation_filename_gt, 'r') as f_gt:

                    reader_gt = csv.reader(f_gt)

                    header_gt = next(reader_gt)
                    col_idx_gt = compute_column_indices(header_gt)

                    rows_gt = list(reader_gt)

                    for row_gt in rows_gt:

                        gt_sess_idx = int(row_gt[col_idx_gt['session_idx']])
                        if gt_sess_idx != sess_idx:
                            continue
                        gt_scene_idx = int(row_gt[col_idx_gt['scene_idx']])
                        if gt_scene_idx != scene_idx:
                            continue

                        sx_gt = float(row_gt[col_idx_gt['sx']])
                        sy_gt = float(row_gt[col_idx_gt['sy']])
                        sz_gt = float(row_gt[col_idx_gt['sz']])
                        rx_gt = float(row_gt[col_idx_gt['rx']])
                        ry_gt = float(row_gt[col_idx_gt['ry']])
                        self.state_angle_gt = float(row_gt[col_idx_gt['state_angle']])

                        R_Arot_W_gt = get_matrix('R_A_S', 9, row_gt, col_idx_gt).reshape(3, 3)
                        t_Arot_W_gt = get_matrix('t_A_S', 3, row_gt, col_idx_gt)

                        T_Arot_W_gt = np.eye(4)
                        T_Arot_W_gt[:3, :3] = R_Arot_W_gt
                        T_Arot_W_gt[:3, 3] = t_Arot_W_gt

                        T_Arot_A_gt = np.eye(4)
                        T_Arot_A_gt[:3, :3] = rot_z(np.deg2rad(self.state_angle_gt))
                        T_A_W_gt = T_Arot_W_gt @ np.linalg.inv(T_Arot_A_gt)
                        
                        self.cabinet_model_gt = self.create_cabinet_model(
                            s=np.array([self.door_thickness, sy_gt, sz_gt, self.static_depth]),
                            r=np.array([rx_gt, ry_gt]),
                            axis_pos=-1,
                            T_A_W=T_A_W_gt,
                            axis_distance=self.axis_offset,
                            moving_to_static_part_distance=self.moving_to_static_part_distance,
                            save_path=None,
                            has_handle=False
                        )

                # if not self.touch_session_set:
                #     self.set_rvl_manipulator(load_fcl_meshes=False)


                # if scene_idx == last_scene_idx:
                #     self.py_touch.set_visualization(True)
                #     debug_ = 0
                #     print('debug')

                self.py_touch.set_new_scene(sx, sy, sz, rx, ry, self.touch_a, self.touch_b, self.touch_c, self.state_angle, T_C_E, self.T_A_C_init, T_E_0_capture,
                                            sx_gt, sy_gt, sz_gt, rx_gt, ry_gt, self.state_angle_gt, T_Arot_W_gt)
                self.touch_session_set = True
                self.update_model_from_touch()
                # self.correct_touch_model()
                # plan trajectory from estimated cabinet


                # self.rvl_manipulator.set_environment_from_touch()
                # self.cabinet_model.create_mesh()
                # self.cabinet_model.save_mesh('/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/offline_detection/cabinet_1/cabinet_mesh_model/cabinet_mesh.ply', 
                #                              mesh=self.cabinet_model.mesh, pose=np.linalg.inv(self.cabinet_model.T_A_O))
                # self.rvl_manipulator.visualize_vn_model()


                # if scene_idx == last_scene_idx:
                    # num_traj_pts = 3 if is_really_offline else 10
                    # self.opening_angle = self.state_angle - 5.0 if is_really_offline else -45.0
                    # self.scale_T_G_DD_all()
                    # waypoint_trajectories, trajectories = self.plan_trajectory(num_traj_pts)

                with open(self.cabinets_touches_filename, 'r') as f_touches:
                    reader_touches = csv.reader(f_touches)
                    header_touches = next(reader_touches)
                    col_idx_touches = compute_column_indices(header_touches)

                    rows_touches = list(reader_touches)
                    if not rows_touches:
                        continue

                    for row_touch in rows_touches:
                        touch_sess_idx = int(row_touch[col_idx_touches['session_idx']])
                        if touch_sess_idx != sess_idx:
                            continue
                        touch_scene_idx = int(row_touch[col_idx_touches['scene_idx']])
                        if touch_scene_idx != scene_idx:
                            continue
                        
                        type_touch = TouchType(int(row_touch[col_idx_touches['type']]))
                        if type_touch == TouchType.WANTED_TOUCH:
                            succ_trajs += 1
                            succ_trajs_all += 1
                            continue  # Skip wanted touches for now

                        R_Ek_E = get_matrix('R_Ek_E', 9, row_touch, col_idx_touches).reshape(3, 3)
                        t_Ek_E = get_matrix('t_Ek_E', 3, row_touch, col_idx_touches)
                        V = get_matrix('V', 3, row_touch, col_idx_touches)
                        t = float(row_touch[col_idx_touches['t']])
                        state_angle_touch = float(row_touch[col_idx_touches['state_angle']])
                        b_miss = True if type_touch == TouchType.MISS else False

                        T_Ek_E = np.eye(4)
                        T_Ek_E[:3, :3] = R_Ek_E
                        T_Ek_E[:3, 3] = t_Ek_E

                        self.py_touch.set_touch(T_Ek_E, V, t, b_miss)

                        if self.CORRECT_ON_TOUCH:
                            # if scene_idx == 2:
                            #     print('debug')
                            self.correct_touch_model()
                            # if scene_idx == last_scene_idx and load_idx == prev_load_idx:
                                # self.rvl_manipulator.visualize_vn_current_state(np.zeros((6,)), np.eye(4))
                
                prev_load_idx = load_idx


        # self.py_touch.set_visualization(True)
        # if self.CORRECT_ON_TOUCH:
        #     self.correct_touch_model()

        # self.initialize_for_scene(load_idx, is_really_offline)
        # if load_idx == prev_load_idx:
        #     num_traj_pts = 3 if is_really_offline else 10
        #     self.opening_angle = self.state_angle - 5.0 if is_really_offline else -45.0
        #     self.scale_T_G_DD_all()
        #     waypoint_trajectories, trajectories = self.plan_trajectory(num_traj_pts)


        trajs_done = 0
        # IS_OFFLINE_ = self.IS_OFFLINE
        # IS_OFFLINE_ = is_really_offline
        is_new_scene = False
        # if IS_OFFLINE_:
        #     if load_idx <= 2:
        #         if succ_trajs > 2:
        #             load_idx += 1
        #             is_new_scene = True
        #             succ_trajs = 0

        #     if load_idx > 2:
        #         IS_OFFLINE_ = False
        #         load_idx = succ_trajs_all - 9
        #     trajs_done = succ_trajs

        # if not IS_OFFLINE_:
        if succ_trajs > 0:
            load_idx += 1
            trajs_done = 0
            is_new_scene = True

        return load_idx, trajs_done, scene_idx, is_new_scene


    # ------------------------------------------------------------------
    # FSM state handlers
    #
    # Each handler runs one state and returns the next FSMState, or None to
    # abandon the current trajectory (the old `break`). The per-trajectory
    # working set lives on self (trajectory, contact_poses, states).
    # ------------------------------------------------------------------
    def _handle_plan_trajectory(self):
        self.trajectory, self.contact_poses = self.plan_trajectory_multi_contact(self._num_traj_pts)
        if self.trajectory is None:
            rospy.logerr("[FSM] No trajectory generated.")
            return None  # give up on this trajectory (was: break)

        # Display the trajectory in RViz
        self.robot.visualize_trajectory(self.trajectory, start_joints=self.home_q)

        return FSMState.APPROACH_PATH

    def _handle_execute_approach_path(self):
        forces = [30.0, 10.0]
        velocities = [1.0, 0.1]
        accelerations = [0.7, 0.05]

        self._state_angle = self.state_angle
        self.add_cabinet_model_to_scene(self._state_angle, scale_factor=1.05)

        T_6_0_collision = None
        approach_success = True

        for i_approach in range(2):
            current_joints = self.robot.get_current_joint_values()
            approach_trajectory = np.vstack((current_joints, self.trajectory[i_approach]))

            plan_success = False
            if i_approach < 1:
                planned_trajectory, plan_success = self.robot.plan_to_joint_goals2(approach_trajectory)
            if not plan_success:
                planned_trajectory = approach_trajectory
            self.zero_sensor()  # Zero sensor before executing the trajectory
            success, collision_joints = self.execute_with_monitoring_remember_joints(
                planned_trajectory, force_threshold=forces[i_approach],
                max_velocity=velocities[i_approach], max_acceleration=accelerations[i_approach])
            if not success:
                if len(collision_joints) > 0:
                    collision_joints = np.array(collision_joints)
                    T_6_0_before = self.robot.get_fwd_kinematics_moveit(collision_joints[0])
                    T_6_0_collision = self.robot.get_fwd_kinematics_moveit(collision_joints[1])

                    key = input("[FSM] Approach path execution failed. Press 1 if it is a touch and 0 if it should go again: ")
                    if key != '1':
                        rospy.logwarn("[FSM] Contact point is too far from the collision point. Skipping touch recording.")
                        T_6_0_collision = None
                        approach_success = False
                        break
                    else:
                        self.set_touch(T_6_0_collision,
                                np.array((T_6_0_before, T_6_0_collision)),
                                touch_type=TouchType.UNWANTED_TOUCH,
                                state_angle=self._state_angle)
                rospy.logerr("[FSM] Approach path execution failed.")
                approach_success = False
                break

        if not approach_success:
            if T_6_0_collision is not None:
                if self.CORRECT_ON_TOUCH:
                    self.correct_touch_model()
                return FSMState.PLAN_TRAJECTORY
            return FSMState.APPROACH_PATH  # retry approach

        return FSMState.INSERTION_PATH

    def _handle_execute_insertion_path(self):
        rospy.loginfo("[FSM] Executing insertion path...")
        current_joints = self.robot.get_current_joint_values()
        insertion_trajectory = np.vstack((current_joints.copy(), self.trajectory[2]))

        success, skipped_touch = self.INSERTION_PATH(
            insertion_trajectory, force_threshold=7.0, state_angle=self._state_angle)

        if skipped_touch:
            rospy.logwarn("[FSM] Skipped touch during insertion path execution. Retrying approach path.")
            return FSMState.APPROACH_PATH

        if not success:
            return FSMState.PLAN_TRAJECTORY

        rospy.loginfo("[FSM] Insertion path executed successfully.")
        return FSMState.OPENING_PATH

    def _handle_execute_opening_path(self):
        open_trajectory = self.trajectory[2:]

        # Set up monitoring for tactile loss and contact establishment
        tactile_loss_joints = []
        self.robot.tactile_contact_established = False
        monitor_thread = threading.Thread(
            target=monitor_tactile_loss_and_remember_joints,
            args=(self.robot, tactile_loss_joints, 0.3, 4.0, 50.0)  # robot, joints, force_threshold, timeout, refresh_rate
        )
        tactile_establish_joints = []
        monitor_thread2 = threading.Thread(
            target=monitor_tactile_contact_establish,
            args=(self.robot, tactile_establish_joints, 0.5, 5.0, 50.0)  # robot, joints, threshold, timeout, refresh_rate
        )
        monitor_thread2.start()
        monitor_thread.start()
        success = self.robot.send_joint_trajectory_action2(open_trajectory, max_velocity=0.2, max_acceleration=0.2)
        monitor_thread2.join()
        monitor_thread.join()

        if success:
            if len(tactile_establish_joints) > 0:
                tactile_establish_joints = np.array(tactile_establish_joints)
                if tactile_establish_joints.shape[0] == 2:
                    T_6before_0 = self.robot.get_fwd_kinematics_moveit(tactile_establish_joints[-2])
                elif tactile_establish_joints.shape[0] == 1:
                    T_6before_0 = self.robot.get_fwd_kinematics_moveit(open_trajectory[0])
                else:
                    rospy.logerr("[FSM] Unexpected number of tactile establish joints. Exiting.")
                    self._abort_run = True
                    return FSMState.EXIT

                T_6contact_0 = self.robot.get_fwd_kinematics_moveit(tactile_establish_joints[-1])

                # Calculate the angle based on the distance from the contact points
                closest_idx = self.get_closest_contact_pose_idx(self.contact_poses, T_6contact_0)
                state_angle = self.states[closest_idx]

                self.set_touch(T_6contact_0,
                            np.array((T_6before_0, T_6contact_0)),
                            touch_type=TouchType.WANTED_TOUCH,
                            state_angle=state_angle, only_save=True)

                T_6_0_current = self.robot.get_current_tool_pose()
                T_6_0_backup = T_6_0_current.copy()
                T_6_0_backup[:3, 3] += -0.03 * T_6_0_backup[:3, 2]
                backup_q = self.robot.get_closest_ik_solution(T_6_0_backup, self.robot.get_current_joint_values())
                if backup_q is not None:
                    backup_traj = np.vstack((self.robot.get_current_joint_values(), backup_q))
                    rospy.loginfo("[FSM] Executing backup trajectory...")
                    self.execute_with_monitoring(backup_traj, force_threshold=30.0, max_velocity=0.1, max_acceleration=0.1)

                rospy.loginfo("[FSM] Opening path executed successfully.")
                return FSMState.EXIT

            rospy.logwarn("[FSM] No tactile contact established during opening path execution.")
            self._record_miss_on_door_plane(open_trajectory)
            if self.CORRECT_ON_TOUCH:
                self.correct_touch_model()
            return FSMState.PLAN_TRAJECTORY

        if len(tactile_loss_joints) > 0:
            tactile_loss_joints = np.array(tactile_loss_joints)
            if tactile_loss_joints.shape[0] == 2:
                T_6before_0 = self.robot.get_fwd_kinematics_moveit(tactile_loss_joints[-2])
            else:
                T_6before_0 = self.robot.get_fwd_kinematics_moveit(open_trajectory[0])
            T_6miss_0 = self.robot.get_fwd_kinematics_moveit(tactile_loss_joints[-1])

            T_TCPmiss_0 = T_6miss_0 @ self.T_TCP_6

            states_ = np.linspace(self.state_angle, self.opening_angle, 200)
            states_ = np.deg2rad(states_)
            T_Arot_A_arr = np.zeros((len(states_), 4, 4))
            T_Arot_A_arr[:, :3, :3] = rotz_multiple(states_)
            T_Arot_A_arr[:, 3, 3] = 1.0
            T_D_S_arr = self.cabinet_model.T_A_W[np.newaxis, ...] @ T_Arot_A_arr @ self.cabinet_model.T_D_Arot[np.newaxis, ...]

            T_TCPmiss_D_arr = np.linalg.inv(T_D_S_arr) @ T_TCPmiss_0
            t_TCPmiss_D_array = T_TCPmiss_D_arr[:, :3, 3]
            mask_ = t_TCPmiss_D_array[:, 2] > 0.0
            t_TCPmiss_D_array = t_TCPmiss_D_array[mask_]
            states_ = states_[mask_]
            if t_TCPmiss_D_array.shape[0] < 1:
                rospy.logerr("[FSM] No valid TCP contact points found in the door frame. Skipping touch recording.")
                return FSMState.PLAN_TRAJECTORY

            idx_ = np.argmin(t_TCPmiss_D_array[:, 2])
            state_angle = np.rad2deg(states_[idx_])
            self.state_angle = state_angle

            states_gt_ = np.linspace(self.state_angle_gt, self.opening_angle, 200)
            states_gt_ = np.deg2rad(states_gt_)
            T_Arot_A_arr_gt = np.zeros((len(states_gt_), 4, 4))
            T_Arot_A_arr_gt[:, :3, :3] = rotz_multiple(states_gt_)
            T_Arot_A_arr_gt[:, 3, 3] = 1.0
            T_D_S_arr_gt = self.cabinet_model_gt.T_A_W[np.newaxis, ...] @ T_Arot_A_arr_gt @ self.cabinet_model_gt.T_D_Arot[np.newaxis, ...]

            T_TCPmiss_D_arr_gt = np.linalg.inv(T_D_S_arr_gt) @ T_TCPmiss_0
            t_TCPmiss_D_array_gt = T_TCPmiss_D_arr_gt[:, :3, 3]
            mask_ = t_TCPmiss_D_array_gt[:, 2] > 0.0
            t_TCPmiss_D_array_gt = t_TCPmiss_D_array_gt[mask_]
            if t_TCPmiss_D_array_gt.shape[0] < 1:
                state_angle_gt = state_angle
            else:
                idx_gt = np.argmin(t_TCPmiss_D_array_gt[:, 2])
                states_gt_ = states_gt_[mask_]
                state_angle_gt = np.rad2deg(states_gt_[idx_gt])

            self.state_angle_gt = state_angle_gt

            self.set_new_touch_scene(state_angle, state_angle_gt)
            self.set_touch(T_6miss_0,
                        np.array((T_6before_0, T_6miss_0)),
                        touch_type=TouchType.MISS,
                        state_angle=state_angle)

            if self.CORRECT_ON_TOUCH:
                self.correct_touch_model()

            self.recapture(self.trajectory)
            return FSMState.PLAN_TRAJECTORY

        rospy.logwarn("[FSM] No tactile contact established during opening path execution.")
        self._record_miss_on_door_plane(open_trajectory)
        if self.CORRECT_ON_TOUCH:
            self.correct_touch_model()
        return FSMState.PLAN_TRAJECTORY

    def _record_miss_on_door_plane(self, open_trajectory):
        """Project the first opening pose onto the door plane and record it as a
        TouchType.MISS. Shared by both 'no tactile contact established' branches."""
        T_6contact_0 = self.robot.get_fwd_kinematics_moveit(open_trajectory[0])
        T_TCPcontact_0 = T_6contact_0 @ self.T_TCP_6
        T_D_0 = np.linalg.inv(self.robot.T_0_W) @ self.T_D_S

        T_TCPcontact_D = np.linalg.inv(T_D_0) @ T_TCPcontact_0
        T_TCPcontact_D[2, 3] = 0.0
        T_TCP_realcontact_0 = T_D_0 @ T_TCPcontact_D
        T_6realcontact_0 = T_TCP_realcontact_0 @ np.linalg.inv(self.T_TCP_6)
        self.set_touch(T_6realcontact_0,
                    np.array((T_6contact_0, T_6realcontact_0)),
                    touch_type=TouchType.MISS,
                    state_angle=self.state_angle)

    def _run_one_trajectory(self):
        """Drive the per-state handlers until EXIT (success) or a handler
        returns None (abandon this trajectory)."""
        handlers = {
            FSMState.PLAN_TRAJECTORY:        self._handle_plan_trajectory,
            FSMState.APPROACH_PATH:  self._handle_execute_approach_path,
            FSMState.INSERTION_PATH: self._handle_execute_insertion_path,
            FSMState.OPENING_PATH:   self._handle_execute_opening_path,
        }
        while self.state != FSMState.EXIT:
            handler = handlers.get(self.state)
            if handler is None:
                break
            next_state = handler()
            if next_state is None:
                break
            self.state = next_state
            if self._abort_run:
                break

    # === Run the FSM ===
    def run(self):
        """Run the FSM."""
        rospy.loginfo("[FSM] Starting FSM...")
        self._abort_run = False

        is_new_scene = True
        if self.LOAD_SESSION:
            rospy.loginfo("[FSM] Loading session...")
            self.py_touch.set_visualization(True)
            scenes_done, trajs_done, self.idx_to_save, is_new_scene = self.load_session()
            self.py_touch.set_visualization(True)
        else:
            self.save_models_init_info_for_rvl()
            self.idx_to_save = -1
            self.touch_session_set = False
            trajs_done = 0
            scenes_done = 0

        self.home_q = np.array([-np.pi/2, -np.pi/2, 0.0, -np.pi/2, 0.0, -3/4 * np.pi])

        # === Parameters for running through multiple scenes and trajectories ===
        num_trajectories = 1
        num_traj_pts = 27

        self.state = FSMState.PLAN_TRAJECTORY

        for i_scene in self.scene_indices:
            if i_scene < scenes_done:
                continue  # already processed in a previous session
            self.initialize_for_scene(i_scene)
            self.initialize(is_new_scene)

            # Set touch scene
            if is_new_scene:
                self.set_new_touch_scene(self.state_angle, self.state_angle_gt)
                self.touch_session_set = True
                self.update_model_from_touch()
                is_new_scene = True

            self.state = FSMState.PLAN_TRAJECTORY
            self.trajectory = []
            self.contact_poses = []
            self._num_traj_pts = num_traj_pts

            for i_trajectory in range(trajs_done, num_trajectories):
                self.states = np.linspace(self.state_angle, self.opening_angle, num_traj_pts)

                self._run_one_trajectory()
                if self._abort_run:
                    rospy.loginfo("[FSM] Aborting FSM run.")
                    return

                if self.state == FSMState.EXIT:
                    # Go to next trajectory, but change the state
                    rospy.loginfo("[FSM] Exiting current trajectory. Moving to next contact touch...")
                    self.state = FSMState.PLAN_TRAJECTORY

                trajs_done = 0 # reset trajectory done counter for the next scene


            input("Press Enter to continue to the next scene...")
            is_new_scene = True
            self.state = FSMState.PLAN_TRAJECTORY
        
        scenes_done = 0


if __name__ == "__main__":
    with open(os.path.join(os.path.dirname(__file__), '../cfg/door_replanning_control_multi_contact.yaml'), 'r') as f:
        config = yaml.safe_load(f)

    fsm = DoorReplanningFSM(config)
    fsm.run()