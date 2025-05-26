#!/usr/bin/python

import rospy
import os, sys
import numpy as np
import json
import threading
from tf2_ros import Buffer, TransformListener
from core.ur5_commander import UR5Commander
from core.real_ur5_controller import UR5Controller
from core.image_process import OneShotImageCapture
sys.path.append(os.path.join(os.path.dirname(__file__), '../../door_detection/src'))
from door_state_detector import DoorStateDetector
from push_force_trajectories import generate_tool_line_poses, generate_trajectories_and_approach2
from force_utils import *
from gazebo_push_open.cabinet_model import Cabinet
import RVLPYDDManipulator as rvlpy
import pickle

class DoorReplanningFSM:
    def __init__(self):
        rospy.init_node('door_replanning_fsm_node')

        # Initialize ROS utilities
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Paths and configurations
        self.base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'
        self.door_model_path = os.path.join(self.base_dir, 'models/doorModel.json')
        self.cabinet_static_mesh_path = os.path.join(self.base_dir, 'cabinet_model/cabinet_static.ply')
        self.cabinet_panel_mesh_path = os.path.join(self.base_dir, 'cabinet_model/cabinet_panel.ply')


        self.gt_poses_path = os.path.join(self.base_dir, 'gt_cabinets/cabinet_gt_poses.npy')
        self.gt_trajectories_path = os.path.join(self.base_dir, 'gt_cabinets/cabinet_gt_trajectories.pkl')

        self.gt_poses = np.load(self.gt_poses_path)
        with open(self.gt_trajectories_path, 'rb') as f:
            self.trajectories_per_pose = pickle.load(f)
        self.gt_idx = 0

        self.gt_door_model_path = os.path.join(self.base_dir, 'models/T_A_W_gt.npy')
        self.gt_door_width_path = os.path.join(self.base_dir, 'models/width.npy')

        self.door_detector_config_path = '/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg'
        self.best_hyp_path = os.path.join(self.base_dir, 'DDT.txt')
        self.cabinet_mesh_path = os.path.join(self.base_dir, 'cabinet_model/cabinet_mesh.ply')

        self.T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')
        self.T_0_W = np.eye(4)
        self.T_TCP_G = np.eye(4)
        # self.T_TCP_G[:3, 3] = np.array([0.155 * 0.5 - 0.005, 0, 0.098])
        self.T_TCP_G[:3, 3] = np.array([0.155 * 0.5 - 0.007, 0, 0.100])
        self.R_TCP_D = np.array([[0, 0, -1],
                                [0, 1, 0],
                                [1, 0, 0]])

        # Image capture parameters
        self.rgb_topic = '/camera/color/image_raw'
        self.depth_topic = '/camera/aligned_depth_to_color/image_raw'
        self.camera_info_topic = '/camera/color/camera_info'
        self.depth_encoding = '16UC1'
        self.save_dir = os.path.join(self.base_dir, 'detect_state_images')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Initialize robot and manipulator
        self.robot = UR5Controller()
        self.robot.T_C_6 = self.T_C_6
        self.robot.T_G_6 = np.eye(4)

        self.tactile_loss_joints = []
        self.contact_established = False

        self.rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
        self.rvl_manipulator = rvlpy.PYDDManipulator()
        self.rvl_manipulator.create(self.rvl_cfg)
        # self.rvl_manipulator.set_robot_pose(self.robot.T_0_W)

        # Cabinet model
        self.cabinet_model = None
        self.state_angle = None
        self.opening_angle = -45.0
        self.push_latch_mechanism_length = 0.046 + 0.018 * 0.5
        self.T_6_0_capture = np.eye(4)

        # FSM states
        self.state = "INITIALIZE"

    def initialize(self):
        rospy.loginfo("[FSM] Initializing...")

        self.cabinet_model, self.T_6_0_capture, self.state_angle = self.create_detected_cabinet_model(self.door_model_path)

        # Save meshes
        self.state = "PLAN_TRAJECTORY"
        # self.state = "LOAD_TRAJECTORY"
        # self.correct_model()

    def set_rvl_manipulator(self):
        self.rvl_manipulator.set_robot_pose(self.robot.T_0_W)
        self.rvl_manipulator.set_door_model_params(
            self.cabinet_model.d_door,
            self.cabinet_model.w_door,
            self.cabinet_model.h_door,
            self.cabinet_model.rx,
            self.cabinet_model.ry,
            self.cabinet_model.axis_pos,
            self.cabinet_model.static_side_width,
            self.cabinet_model.moving_to_static_part_distance)
        self.rvl_manipulator.set_door_pose(self.cabinet_model.T_A_S)
        self.rvl_manipulator.load_cabinet_static_mesh_fcl(self.cabinet_static_mesh_path)
        self.rvl_manipulator.load_cabinet_panel_mesh_fcl(self.cabinet_panel_mesh_path)

    def plan_trajectory(self):
        self.add_cabinet_model_to_scene(self.state_angle)

        self.set_rvl_manipulator()
    
        rospy.loginfo("[FSM] Planning trajectory...")
        T_G_DD_all = generate_tool_line_poses(self.cabinet_model.h_door, self.T_TCP_G, self.R_TCP_D).reshape(-1, 4, 4)
        _, self.trajectories = generate_trajectories_and_approach2(
            T_G_DD_all, 
            10, 
            self.state_angle, 
            self.opening_angle, 
            self.cabinet_model, 
            self.rvl_manipulator, 
            self.T_0_W, 
            self.robot
        )
        np.save(os.path.join(self.base_dir, 'trajectories.npy'), self.trajectories)

        if len(self.trajectories) < 1:
            rospy.logerr("[FSM] No trajectories generated. Exiting.")
            self.state = "EXIT"
        else:           
            self.extract_closest_trajectory()
            self.state = "EXECUTE_APPROACH_PATH"

    def load_trajectory(self):
        rospy.loginfo("[FSM] Loading trajectory...")
        trajectory_path = os.path.join(self.base_dir, 'trajectories.npy')
        if os.path.exists(trajectory_path):
            self.trajectories = np.load(trajectory_path)

            if len(self.trajectories) < 1:
                rospy.logerr("[FSM] No trajectories loaded. Exiting.")
                self.state = "EXIT"
                return
            else:
                self.extract_closest_trajectory()

            rospy.loginfo("[FSM] Trajectory loaded successfully.")
            self.state = "EXECUTE_APPROACH_PATH"
        else:
            rospy.logerr("[FSM] Trajectory file not found. Exiting.")
            self.state = "EXIT"

    def extract_closest_trajectory(self):
        rospy.loginfo("[FSM] Extracting closest trajectory...")
        current_joints = self.robot.get_current_joint_values()
        first_joints = self.trajectories[:, 0, :]
        cheb_distances = np.max(np.abs(first_joints - current_joints), axis=1)
        closest_idx = np.argmin(cheb_distances)
        rospy.loginfo(f"[FSM] Closest trajectory index: {closest_idx}")
        self.trajectory = self.trajectories[closest_idx]

    def execute_approach_path(self):
        rospy.loginfo("[FSM] Executing approach path...")

        # Add cabinet model to the robot environment in Moveit
        self.add_cabinet_model_to_scene(self.state_angle)

        approach_path = np.array([self.robot.get_current_joint_values(), self.trajectory[0], self.trajectory[1]])
        for i_pt in range(1, approach_path.shape[0]):
            planned_joints, plan_success = self.robot.plan_to_joint_goal(approach_path[i_pt])
            if not plan_success:
                rospy.logwarn("[FSM] Approach path planning failed. Replanning...")
                self.state = "CORRECT_MODEL"
                break
            success = self.execute_with_monitoring(planned_joints, force_threshold=30.0)
            if success:
                rospy.loginfo("[FSM] Approach path executed successfully.")
                self.state = "EXECUTE_INSERTION_PATH"
            else:
                rospy.logwarn("[FSM] Approach path execution failed. Replanning...")
                self.state = "CORRECT_MODEL"
                break
        
        self.robot.remove_mesh_from_scene("cabinet_model")

    def zero_sensor(self):
        rospy.loginfo("[FSM] Zeroing force sensor...")
        self.robot.zero_ft_sensor()
        rospy.loginfo("[FSM] Force sensor zeroed.")

    def add_cabinet_model_to_scene(self, state_angle):
        rospy.loginfo("[FSM] Adding cabinet model to scene...")
        self.cabinet_model.change_door_angle(state_angle)
        self.cabinet_model.save_mesh(self.cabinet_mesh_path)
        self.cabinet_model.save_mesh_without_doors(self.cabinet_static_mesh_path)
        self.cabinet_model.save_door_panel_mesh(self.cabinet_panel_mesh_path)
        self.robot.remove_mesh_from_scene("cabinet_model")
        self.robot.add_mesh_to_scene(self.cabinet_mesh_path, "cabinet_model", self.cabinet_model.T_A_S)
        rospy.loginfo("[FSM] Cabinet model added to scene.")

    def execute_insertion_path(self):
        self.zero_sensor()
        rospy.loginfo("[FSM] Executing insertion path...")
        insertion_path = np.array([self.robot.get_current_joint_values(), self.trajectory[2]])
        success = self.execute_with_monitoring(insertion_path, force_threshold=15.0)
        if success:
            rospy.loginfo("[FSM] Insertion path executed successfully.")
            self.state = "EXECUTE_OPENING_PATH"
        else:
            rospy.logwarn("[FSM] Insertion path execution failed. Replanning...")
            # TODO: back up first then correct model
            self.backup_current_state()
            self.state = "CORRECT_MODEL"

    def execute_opening_path(self):
        rospy.loginfo("[FSM] Executing opening path...")
        trajectory = self.trajectory[2:]

        self.tactile_loss_joints = []
        self.robot.tactile_contact_established = False
        monitor_thread = threading.Thread(
            target=monitor_tactile_loss_and_remember_joints, 
            args=(self.robot, self.tactile_loss_joints, 0.3)
        )
        monitor_thread2 = threading.Thread(
            target=monitor_tactile_contact_establish,
            args=(self.robot, 0.5, 4.0)
        )
        monitor_thread2.start()
        monitor_thread.start()
        success = self.robot.send_joint_trajectory_action(trajectory, max_velocity=0.5, max_acceleration=0.5)
        monitor_thread2.join()
        monitor_thread.join()

        if success:
            rospy.loginfo("[FSM] Door opened successfully.")
            self.state = "EXIT"
        elif len(self.tactile_loss_joints) > 0:
            rospy.logerr("[FSM] Opening path execution failed. Contact loss detected. Replanning...")
            self.state = "RECAPTURE"
        else:
            rospy.logerr("[FSM] Opening path execution failed. No contact loss detected.")
            self.state = "CORRECT_MODEL"

    def backup_current_state(self):
        rospy.loginfo("[FSM] Backing up current state...")
        current_joints = self.robot.get_current_joint_values()
        T_6_0 = self.robot.get_fwd_kinematics_moveit(current_joints)
        backup_trajectory = np.array([current_joints, self.trajectory[1]])
        # self.execute_without_monitoring(backup_trajectory)
        self.execute_and_remember_joints_on_force_loss(backup_trajectory)
        rospy.loginfo("[FSM] Current state backed up.")

    def recapture(self):
        rospy.loginfo("[FSM] Replanning...")

        if len(self.tactile_loss_joints) < 1:
            rospy.logerr("[FSM] No tactile loss detected. Exiting.")
            self.state = "EXIT"
            return

        # Get estimated door state
        # Find the closest point in the trajectory
        cheb_distances = np.max(np.abs(self.trajectory[2:] - self.tactile_loss_joints), axis=1)
        closest_idx = np.argmin(cheb_distances)

        door_opening_angles = np.linspace(self.state_angle, self.opening_angle, self.trajectory[2:].shape[0])
        estimated_door_angle = door_opening_angles[closest_idx]
        print(f"[replan] Estimated door angle at contact loss: {estimated_door_angle:.2f} degrees")

        # Get new camera capture pose
        T_C_W_new, joints_camera_capture = get_camera_pose_on_sphere_distance(self.cabinet_model, 
                                                                              self.robot, 
                                                                              self.rvl_manipulator, 
                                                                              self.T_6_0_capture,
                                                                              estimated_door_angle)
        if joints_camera_capture is None:
            joints_camera_capture = self.robot.get_closest_ik_solution(self.T_6_0_capture, None)
            
            if joints_camera_capture is None:
                rospy.logerr("[FSM] No valid camera capture pose found. Exiting.")
                self.state = "EXIT"
                return
            
            T_C_W_new = self.robot.T_0_W @ self.T_6_0_capture @ self.robot.T_C_6

        # Backup from the current state
        T_6_0_current = self.robot.get_current_tool_pose()
        T_6_0_backup = T_6_0_current.copy()
        T_6_0_backup[:3, 3] = T_6_0_backup[:3, 3] - T_6_0_backup[:3, 2] * 0.05 # move 5 cm in z-direction
        joints_backup = self.robot.get_closest_ik_solution(T_6_0_backup, None) # get ik solution for backup and take current joints
        if joints_backup is None:
            rospy.logerr("[FSM] No valid IK solution for backup. Exiting.")
            self.state = "EXIT"
            return
        backup_trajectory = np.array([self.robot.get_current_joint_values(), joints_backup])
        success = self.execute_with_monitoring(backup_trajectory, force_threshold=30.0)
        if not success:
            rospy.logerr("[FSM] Backup trajectory execution failed. Exiting.")
            self.state = "EXIT"
            return

        # # Add cabinet model to the robot environment in Moveit
        # self.cabinet_model.change_door_angle(estimated_door_angle)
        # self.cabinet_model.save_mesh(self.cabinet_mesh_path)
        # self.robot.remove_mesh_from_scene("cabinet_model")
        # self.robot.add_mesh_to_scene(self.cabinet_mesh_path, "cabinet_model", self.cabinet_model.T_A_S)
        self.add_cabinet_model_to_scene(estimated_door_angle)

        # Move to the new camera capture pose
        planned_joints, _ = self.robot.plan_to_joint_goal(joints_camera_capture)

        success = self.execute_with_monitoring(planned_joints, force_threshold=30.0)
        if not success:
            rospy.logerr("[FSM] Capture trajectory execution failed. Exiting.")
            self.state = "EXIT"
            return

        # Remove cabinet model from the robot environment
        self.robot.remove_mesh_from_scene("cabinet_model")

        # Capture door state
        image_capture = OneShotImageCapture(self.save_dir, self.rgb_topic, self.depth_topic, self.camera_info_topic, self.depth_encoding)
        rgb_path, _, ply_path = image_capture.capture_single_image_and_save()
        T_C_W_init_capture = self.robot.T_0_W @ self.T_6_0_capture @ self.robot.T_C_6
        # T_Cdet_Cdiff = np.linalg.inv(T_C_W_init_capture) @ T_C_W_new
        T_Cdiff_Cdet = np.linalg.inv(T_C_W_new) @ T_C_W_init_capture
        door_state_detector = DoorStateDetector(detector_config_path=self.door_detector_config_path, best_hyp_path=self.best_hyp_path)
        self.state_angle = door_state_detector.detect_state(rgb_path, ply_path, T_Cdiff_Cdet)
        self.state_angle = self.state_angle[0] # take the first element
        print(f"[replan] Detected door state: {self.state_angle}")

        # Plan new trajectory
        self.state = "PLAN_TRAJECTORY"

    def correct_model(self):
        rospy.loginfo("[FSM] Correcting model...")
        # For now load the GT model
        self.cabinet_model, self.state_angle = self.create_gt_cabinet_model()
        self.trajectories = self.trajectories_per_pose[self.gt_idx]
        self.extract_closest_trajectory()
        self.state = "EXECUTE_APPROACH_PATH"
        # self.state = "PLAN_TRAJECTORY"

    def execute_with_monitoring(self, trajectory, force_threshold=30.0):
        monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(self.robot, force_threshold))
        monitor_thread.start()
        success = self.robot.send_joint_trajectory_action(trajectory, max_velocity=0.5, max_acceleration=0.5)
        monitor_thread.join()
        return success
    
    def execute_without_monitoring(self, trajectory):
        success = self.robot.send_joint_trajectory_action(trajectory, max_velocity=0.5, max_acceleration=0.5)
        return success

    def execute_and_remember_joints_on_force_loss(self, trajectory):
        self.loss_joints = []
        monitor_thread = threading.Thread(target=monitor_force_drop_and_remember_joints, args=(self.robot, self.loss_joints, 10.0, 1.0, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        self.robot.send_joint_trajectory_action(trajectory, max_velocity=0.5, max_acceleration=0.5)
        monitor_thread.join()

    def capture_door_state(self, T_C_W_new):
        rospy.loginfo("[FSM] Capturing door state...")
        image_capture = OneShotImageCapture(self.save_dir, self.rgb_topic, self.depth_topic, self.camera_info_topic, self.depth_encoding)
        rgb_path, _, ply_path = image_capture.capture_single_image_and_save()

        with open(self.door_model_path, 'r') as f:
            data = json.load(f)
        q_detected = np.array(data["joint_values"])
        T_6_0_detected = self.robot.get_fwd_kinematics_moveit(q_detected)
        T_C_W_detected = self.robot.T_0_W @ T_6_0_detected @ self.T_C_6
        T_Cdet_Cdiff = np.linalg.inv(T_C_W_detected) @ T_C_W_new

        door_state_detector = DoorStateDetector(detector_config_path=self.door_detector_config_path, best_hyp_path=self.best_hyp_path)
        self.state_angle = door_state_detector.detect_state(rgb_path, ply_path, T_Cdet_Cdiff)

    def create_detected_cabinet_model(self, data_model_path):
        with open(data_model_path, 'r') as f:
            data = json.load(f)

        R = np.array(data["R"])
        t = np.array(data["t"])
        s = np.array(data["s"])
        r = np.array(data["r"])
        axis_pos = data["openingDirection"]
        T_A_C = np.eye(4)
        T_A_C[:3, :3] = R
        T_A_C[:3, 3] = t
        state_angle = axis_pos * np.rad2deg(np.arcsin(self.push_latch_mechanism_length / s[0]))
        joint_values = np.array(data["joint_values"])

        T_6_0 = self.robot.get_fwd_kinematics_moveit(joint_values)
        T_A_W = self.T_0_W @ T_6_0 @ self.T_C_6 @ T_A_C
        cabinet_model = Cabinet(door_params=np.array([s[0], s[1], 0.018, 0.4]),
                                r=r,
                                axis_pos=axis_pos,
                                T_A_S=T_A_W,
                                save_path=os.path.join(self.base_dir, 'cabinet_model/cabinet_model.urdf'),
                                has_handle=False)
        return cabinet_model, T_6_0, state_angle

    def create_gt_cabinet_model(self):
        width = 0.396
        height = 0.496
        door_thickness = 0.018
        static_depth = 0.4
        axis_pos = -1
        # T_A_W_gt = np.load(self.gt_door_model_path)
        T_A_W_gt = self.gt_poses[self.gt_idx]
        # width = np.load(self.gt_door_width_path)

        state_angle_gt = axis_pos * np.rad2deg(np.arcsin(self.push_latch_mechanism_length / width))
        cabinet_gt = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]),
                                r=np.array([0., -width*0.5]),
                                axis_pos=axis_pos,
                                T_A_S=T_A_W_gt,
                                save_path=None,
                                has_handle=False)
        return cabinet_gt, state_angle_gt


    def run(self):
        while self.state != "EXIT":
            if self.state == "INITIALIZE":
                self.initialize()
            elif self.state == "PLAN_TRAJECTORY":
                self.plan_trajectory()
            elif self.state == "LOAD_TRAJECTORY":
                self.load_trajectory()
            elif self.state == "EXECUTE_APPROACH_PATH":
                self.execute_approach_path()
            elif self.state == "EXECUTE_INSERTION_PATH":
                self.execute_insertion_path()
            elif self.state == "EXECUTE_OPENING_PATH":
                self.execute_opening_path()
            elif self.state == "RECAPTURE":
                self.recapture()
            elif self.state == "CORRECT_MODEL":
                self.correct_model()
        rospy.loginfo("[FSM] Exiting...")

if __name__ == "__main__":
    fsm = DoorReplanningFSM()
    fsm.gt_idx = 4
    fsm.run()