#!/usr/bin/env python
"""
Fused multi-contact path planning with sensor-based door opening.

Flow:
1. Load cabinet configs + pre-computed trajectories (from path_planning)
2. For each cabinet, create a Cabinet model
3. Use DoorReplanningFSM-style force-monitored execution
4. On contact loss or collision -> replan using touch correction
"""

import rospy
import os
import sys
import numpy as np
import yaml
import threading
import json
from enum import Enum  # Add this import
from door_detection.srv import DetectDoor  # Add this import
from path_planning.srv import PlanMultiContactPath  # <-- ADD THIS IMPORT

from core.real_ur5_controller import UR5Controller
from core.transforms import rot_z
from core.util import read_csv_DataFrame
from gazebo_push_open.cabinet_model2 import Cabinet2

# Reuse utilities from force_manipulation
sys.path.append(os.path.join(os.path.dirname(__file__), 
    '../../force_manipulation/src'))
# from push_force_trajectories import *
from force_utils import TouchType, RVLTool, monitor_force_and_cancel, monitor_tactile_loss_and_remember_joints, monitor_tactile_contact_establish

import RVLPYDDManipulator as rvlpy


class FSMState(Enum):
    IDLE = "IDLE"
    INITIALIZE = "INITIALIZE"
    APPROACH = "APPROACH"
    OPENING = "OPENING"
    TOUCH_CORRECT = "TOUCH_CORRECT"
    REPLAN = "REPLAN"
    RECORD_RESULT = "RECORD_RESULT"
    DONE = "DONE"
    ERROR = "ERROR"

class MultiContactForceController:
    """
    Combines:
    - Pre-planned multi-contact trajectories (from path_planning)
    - Force/tactile-monitored execution (from force_manipulation)
    - Touch-based replanning on contact loss
    """

    def __init__(self, config: dict):
        rospy.init_node('multi_contact_force_node')

        # --- Robot ---
        self.robot = UR5Controller()
        self.robot.force_threshold = config.get('force_threshold_approach', 30.0)

        # --- Paths ---
        self.traj_dir = config['trajectory_dir']
        self.door_configs_path = config['door_configs_path']
        self.results_path = config['results_path']
        self.rvl_cfg = config['rvl_cfg_path']

        # --- Cabinet parameters ---
        self.door_thickness = config.get('door_thickness', 0.018)
        self.static_depth = config.get('static_depth', 0.4)

        # --- Load door configurations ---
        self.doors = np.load(self.door_configs_path)

        # --- Load previous results for resume ---
        self.results = config.get('results_csv_path', self.results_path)

        # --- RVL manipulator (for replanning) ---
        self.rvl_manipulator = rvlpy.PYDDManipulator()
        self.rvl_manipulator.create(self.rvl_cfg)
        
        self.rvl_touch_cfg = config.get('rvl_touch_cfg', None)
        self.py_touch = self.rvl_manipulator.py_touch
        if self.rvl_touch_cfg:
            self.py_touch.create(self.rvl_touch_cfg)

        # --- Force thresholds ---
        self.force_threshold_approach = config.get('force_threshold_approach', 30.0)
        self.force_threshold_insertion = config.get('force_threshold_insertion', 10.0)
        self.force_threshold_opening = config.get('force_threshold_opening', 40.0)

        # --- Touch/camera (from DoorReplanningFSM) ---
        T_C_6_path = config.get('T_C_6_path', None)
        if T_C_6_path:
            self.T_C_6 = np.load(T_C_6_path)
            self.T_C_6_init = self.T_C_6.copy()
        else:
            self.T_C_6 = np.eye(4)
            self.T_C_6_init = np.eye(4)
        
        self.T_0_W = np.eye(4)
        self.T_6_0_capture = np.eye(4)
        self.T_A_C_init = np.eye(4)

        # --- Tool parameters ---
        tool_cfg = config.get("tool", {})
        self.tactile_sensor_depth = config.get('tactile_sensor_depth', 0.006)
        tx_E = tool_cfg.get("tx_E", 0.155 - self.tactile_sensor_depth)
        tz_E = tool_cfg.get("tz_E", 0.278)
        tx_G = tool_cfg.get("tx_G", 0.0721)
        tz_G = tool_cfg.get("tz_G_one_finger", 0.1041)

        a_tool = tool_cfg.get("a", 0.0205)
        b_tool = tool_cfg.get("b", 0.032)
        c_tool = tool_cfg.get("c", 0.011)
        d_tool = tool_cfg.get("d", 0.026)
        h_tool = tool_cfg.get("h", 0.023)
        rotz_angle = tool_cfg.get("rotz_angle", -np.pi * 0.25)

        self.rvl_tool = RVLTool(a_tool, b_tool, c_tool, d_tool, h_tool, -tx_E, tz_E, rot_z_angle=rotz_angle)
        self.T_TCP_6 = self.rvl_tool.T_TCP_6
        self.py_touch.create_simple_tool(a_tool, b_tool, c_tool, d_tool, h_tool, self.rvl_tool.T_tool_6)
        
        self.T_TCP_G = np.eye(4)
        self.T_TCP_G[:3, 3] = np.array([-tx_G, 0, tz_G])

        # --- Camera Params ---
        camera_cfg = config.get("camera", {})
        camera_fu = camera_cfg.get("fu", 597.9033203125)
        camera_fv = camera_cfg.get("fv", 598.47998046875)
        camera_uc = camera_cfg.get("uc", 323.8436584472656)
        camera_vc = camera_cfg.get("vc", 236.32774353027344)
        camera_w = camera_cfg.get("width", 640)
        camera_h = camera_cfg.get("height", 480)
        self.py_touch.set_camera_params(camera_fu, camera_fv, camera_uc, camera_vc, camera_w, camera_h)

        # --- Touch Config ---
        touch_cfg = config.get("touch", {})
        self.touch_a = touch_cfg.get("a", self.static_depth)
        self.touch_b = touch_cfg.get("b", 0.0)
        self.touch_c = touch_cfg.get("c", 0.005)

        self.correct_on_touch = config.get('correct_on_touch', True)
        self.load_existing_model = config.get('load_existing_model', False)
        
        # --- Door Detection ---
        raw_detection_dir = config.get("detection_base_dir", "~/ferit_ur5_ws/data/online_detection")
        self.detection_base_dir = os.path.expanduser(raw_detection_dir)

        # --- Path Planning Service ---
        self.axis_distance = config.get('axis_distance', 0.01)
        self.static_side_width = config.get('static_side_width', 0.017)
        self.moving_to_static_part_distance = config.get('moving_to_static_part_distance', 0.005)
        self.plan_num_states = config.get('plan_num_states', 37)
        self.plan_target_angle = config.get('plan_target_angle', -90.0)

        # --- FSM state ---
        self.state = FSMState.IDLE

    def create_cabinet_model(self, door_params, axis_pos, T_A_S):
        """Create Cabinet2 model from door parameters."""
        width, height = door_params[0], door_params[1]
        return Cabinet2(
            s=np.array([self.door_thickness, width, height, self.static_depth]),
            r=np.array([-self.door_thickness * 0.5, -width * 0.5]),
            axis_pos=axis_pos,
            T_A_W=T_A_S,
            save_path=None,
            has_handle=False
        )

    def _apply_detection_data(self, detect_data: dict, door_index: int):
        """Parse JSON detection data and build the cabinet model."""
        door_cfg = self.doors[door_index]

        # Extract dimensions (with fallback to config)
        if 'w' in detect_data and 'h' in detect_data:
            width = float(detect_data['w'][0])
            height = float(detect_data['h'][0])
        elif 's' in detect_data:
            width = float(detect_data['s'][0])
            height = float(detect_data['s'][1])
        else:
            rospy.logwarn("[FSM] No size info in detection result. Using config dimensions.")
            width, height = float(door_cfg[0]), float(door_cfg[1])

        # Extract pose (with fallback to config)
        if 'T_A_0' in detect_data:
            T_A_0 = np.array(detect_data['T_A_0']).reshape(4, 4)
        else:
            rospy.logwarn("[FSM] T_A_0 missing in detection result. Using config pose.")
            T_A_0 = np.eye(4)
            T_A_0[:3, 3] = door_cfg[2:5]
            Tz = np.eye(4)
            Tz[:3, :3] = rot_z(np.radians(float(door_cfg[5])))
            T_A_0 = T_A_0 @ Tz

        axis_pos = int(door_cfg[7])
        self.cabinet_model = self.create_cabinet_model([width, height], axis_pos, T_A_0)
        rospy.loginfo(f"[FSM] Cabinet model built from detection: W={width:.3f}, H={height:.3f}")

    def plan_path_for_door(self, door_index: int, T_R_W: np.ndarray = None) -> np.ndarray:
        """
        Call the path planning service for a given door index.
        Returns the Nx6 joint trajectory if found, otherwise None.
        """
        rospy.loginfo(f"[FSM] Calling path planning service for door {door_index}...")

        try:
            rospy.wait_for_service('plan_multi_contact_path', timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("[FSM] 'plan_multi_contact_path' service not available!")
            return None

        plan_srv = rospy.ServiceProxy('plan_multi_contact_path', PlanMultiContactPath)

        door = self.doors[door_index]
        width     = float(door[0])
        height    = float(door[1])
        position  = door[2:5]
        rot_z_deg = float(door[5])
        state_angle = float(door[6])
        axis_pos  = int(door[7])

        T_A_S = np.eye(4)
        T_A_S[:3, 3] = position
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
        T_A_S = T_A_S @ Tz

        if T_R_W is None:
            T_R_W = np.eye(4)

        # Build q_init from current joint values and apply UR5 offsets
        q_init = self.robot.get_current_joint_values().copy()
        q_init[0] += np.pi
        q_init[5] += np.pi
        q_init[q_init >  np.pi] -= (2.0 * np.pi)
        q_init[q_init < -np.pi] += (2.0 * np.pi)

        rx = 0.0
        ry = -(width * 0.5 - self.axis_distance)

        try:
            resp = plan_srv(
                w_door=width,
                h_door=height,
                d_door=float(self.door_thickness),
                rx=rx,
                ry=ry,
                static_side_width=float(self.static_side_width),
                axis_distance=float(self.axis_distance),
                moving_to_static_part_distance=float(self.moving_to_static_part_distance),
                axis_pos=axis_pos,
                T_A_S=T_A_S.flatten().tolist(),
                T_R_W=T_R_W.flatten().tolist(),
                q_init=q_init.tolist(),
                start_angle=state_angle,
                target_angle=float(self.plan_target_angle),
                num_states=int(self.plan_num_states)
            )

            if resp.success:
                trajectory = np.array(resp.joint_trajectory).reshape(resp.num_points, 6)
                rospy.loginfo(f"[FSM] Path found: {resp.num_points} waypoints.")
                return trajectory
            else:
                rospy.logwarn(f"[FSM] Path planning service returned no path for door {door_index}.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"[FSM] Path planning service call failed: {e}")
            return None

    def load_trajectory(self, door_index):
        """Load pre-computed trajectory for a given door config."""
        traj_file = os.path.join(self.traj_dir, f'traj_{door_index}.txt')
        if not os.path.exists(traj_file):
            rospy.logwarn(f"Trajectory file not found: {traj_file}")
            return None
        return np.loadtxt(traj_file, delimiter=',')

    def execute_with_force_monitoring(self, trajectory, force_threshold, 
                                       max_velocity=0.5, max_acceleration=0.5):
        """Execute trajectory with force monitoring (from DoorReplanningFSM)."""
        
        monitor_thread = threading.Thread(
            target=monitor_force_and_cancel, 
            args=(self.robot, force_threshold))
        monitor_thread.start()
        success = self.robot.send_joint_trajectory_action2(
            trajectory, 
            max_velocity=max_velocity, 
            max_acceleration=max_acceleration)
        monitor_thread.join()
        return success

    def set_new_touch_scene(self, state_angle):
        self.py_touch.set_new_scene(self.door_thickness, self.cabinet_model.sy, self.cabinet_model.sz, 
                                    self.cabinet_model.rx, self.cabinet_model.ry, 
                                    self.touch_a, self.touch_b, self.touch_c, state_angle, 
                                    self.T_C_6_init, self.T_A_C_init, self.T_6_0_capture,
                                    self.cabinet_model.sx, self.cabinet_model.sy, self.cabinet_model.sz, 
                                    self.cabinet_model.rx, self.cabinet_model.ry, state_angle, np.eye(4))

    def set_touch(self, T_6contact_0, T_6via_0_pts, touch_type: TouchType, state_angle: float, only_save=False):
        T_6contact_6 = np.linalg.inv(self.T_6_0_capture) @ T_6contact_0
        T_6via_6 = np.linalg.inv(self.T_6_0_capture)[np.newaxis, ...] @ T_6via_0_pts
        V = T_6via_6[1, :3, 3] - T_6via_6[0, :3, 3]
        t = np.linalg.norm(V)
        if t > 0:
            V /= t  # Normalize the vector
            
        b_miss = True if touch_type == TouchType.MISS else False
        if not only_save:
            self.py_touch.set_touch(T_6contact_6, V, t, b_miss)

    def correct_touch_model(self):
        rospy.loginfo("[FSM] Correcting touch model...")
        self.py_touch.correct()
        self.update_model_from_touch()

    def update_model_from_touch(self):
        self.rvl_manipulator.update_model_x()

        T_Arot_6_corrected = self.rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
        T_C_6_corrected = self.rvl_manipulator.get_corrected_camera_pose().astype(np.float64)

        if np.any(np.isnan(T_Arot_6_corrected)) or np.any(np.isnan(T_C_6_corrected)):
            rospy.logerr("[FSM] Corrected poses contain NaN values. Sticking to previous model.")
            return
            
        self.T_C_6 = T_C_6_corrected.copy()
        T_Arot_A = np.eye(4)
        # Using state angle 0 for base update like in original
        self.T_A_C = np.linalg.inv(self.T_C_6) @ T_Arot_6_corrected @ np.linalg.inv(T_Arot_A)

        T_D_Arot = self.rvl_manipulator.get_corrected_pose_D_Arot().astype(np.float64)
        self.cabinet_model.T_D_Arot = T_D_Arot.copy()

        self.cabinet_model.T_A_W = self.robot.T_0_W @ self.T_6_0_capture @ self.T_C_6 @ self.T_A_C
        rospy.loginfo(f"[FSM] Corrected cabinet model pose updated.")
        
        T_D_0 = self.rvl_manipulator.get_corrected_pose_D_0().astype(np.float64)
        self.T_D_S = self.robot.T_0_W @ T_D_0
        self.rvl_manipulator.set_pose_DD_S(self.T_D_S)
        self.rvl_manipulator.set_environment_from_touch()

    def run_experiment(self, door_index):
        """Run a single door-opening experiment with force monitoring."""
        door = self.doors[door_index]
        width, height = door[0], door[1]
        position = door[2:5]
        rot_z_deg = door[5]
        axis_pos = door[7]

        # Build cabinet pose
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = position
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
        T_A_S = T_A_S @ Tz

        cabinet_model = self.create_cabinet_model([width, height], axis_pos, T_A_S)

        # Try loading pre-saved trajectory first; if missing, call planning service
        trajectory = self.load_trajectory(door_index)
        if trajectory is None:
            rospy.logwarn(f"[Experiment {door_index}] No saved trajectory. Calling path planning service...")
            trajectory = self.plan_path_for_door(door_index)
            if trajectory is None:
                rospy.logerr(f"[Experiment {door_index}] Path planning failed. Aborting.")
                return False

        # Set touch initial scene to match replanning demands
        self.cabinet_model = cabinet_model
        self.set_new_touch_scene(state_angle=0.0)

        # --- Phase 1: Approach (force-monitored) ---
        rospy.loginfo(f"[Experiment {door_index}] Executing approach...")
        self.robot.force_threshold = self.force_threshold_approach
        approach_success = self.execute_with_force_monitoring(
            trajectory[:3], 
            self.force_threshold_approach,
            max_velocity=0.5, max_acceleration=0.5)

        if not approach_success:
            rospy.logwarn(f"[Experiment {door_index}] Approach failed (Collision bounds).")
            # Could trigger set_touch UNWANTED touch here and correct
            return False
            
        # --- Phase 2: Opening (force-monitored with tactile) ---
        rospy.loginfo(f"[Experiment {door_index}] Executing opening...")
        open_trajectory = trajectory[2:]
        
        tactile_loss_joints = []
        tactile_establish_joints = []
        
        self.robot.tactile_contact_established = False
        
        monitor_thread_loss = threading.Thread(
            target=monitor_tactile_loss_and_remember_joints, 
            args=(self.robot, tactile_loss_joints, 0.3, 4.0, 50.0)
        )
        monitor_thread_establish = threading.Thread(
            target=monitor_tactile_contact_establish,
            args=(self.robot, tactile_establish_joints, 0.5, 4.0, 50.0)
        )
        
        monitor_thread_establish.start()
        monitor_thread_loss.start()
        
        open_success = self.robot.send_joint_trajectory_action2(open_trajectory, max_velocity=0.2, max_acceleration=0.2)
        
        monitor_thread_establish.join()
        monitor_thread_loss.join()

        # Check conditions
        if open_success:
            if len(tactile_establish_joints) > 0:
                rospy.loginfo("[FSM] Tactile contact correctly established.")
                # We can save this as a wanted touch
            else:
                rospy.logwarn("[FSM] Finished but no tactile establishing recorded (assume miss).")
                if self.correct_on_touch:
                    # In real code extract kinematics to set_touch as a MISS 
                    self.correct_touch_model()
                open_success = False
        else:
            if len(tactile_loss_joints) > 0:
                rospy.logwarn("[FSM] Tactile loss detected. Triggering touch replanning.")
                tactile_loss_joints = np.array(tactile_loss_joints)
                
                T_6before_0 = self.robot.get_fwd_kinematics_moveit(
                    tactile_loss_joints[-2] if tactile_loss_joints.shape[0] >= 2 else open_trajectory[0])
                T_6miss_0 = self.robot.get_fwd_kinematics_moveit(tactile_loss_joints[-1])

                self.set_touch(T_6miss_0, np.array((T_6before_0, T_6miss_0)), TouchType.MISS, 0.0)

                if self.correct_on_touch:
                    self.correct_touch_model()
                    rospy.loginfo("[FSM] Replanning triggered & Cabinet Model Updated.")
                    
        return open_success

    def run_single_experiment(self, door_index: int) -> bool:
        self.state = FSMState.INITIALIZE
        self.current_door_index = door_index
        max_replan_attempts = 3
        replan_count = 0
        self._planned_trajectory = None

        while not rospy.is_shutdown():
            rospy.loginfo("[FSM] State: %s (door %d)" % (self.state.value, door_index))

            # ---------------------------------------------------------------
            # INITIALIZE: detect door or load model, then build cabinet model
            # ---------------------------------------------------------------
            if self.state == FSMState.INITIALIZE:

                save_dir = os.path.join(self.detection_base_dir, f"door_{door_index}")
                model_path = os.path.join(save_dir, "models", "doorModel.json")

                # Try loading existing model if flag is set
                if self.load_existing_model and os.path.exists(model_path):
                    rospy.loginfo(f"Loading existing model from {model_path}...")
                    try:
                        with open(model_path, 'r') as f:
                            detect_data = json.load(f)
                        self._apply_detection_data(detect_data, door_index)
                        rospy.loginfo("[FSM] Model loaded. Moving to APPROACH.")
                        self.state = FSMState.APPROACH
                        continue
                    except Exception as e:
                        rospy.logwarn(f"Failed to load model: {e}. Falling back to prompt.")

                # Interactive prompt
                while not rospy.is_shutdown():
                    key = input("[FSM] Press 'p' (manual config) or 'c' (camera detect): ").strip().lower()

                    if key == 'p':
                        door = self.doors[door_index]
                        width, height = door[0], door[1]
                        position = door[2:5]
                        rot_z_deg = door[5]
                        T_A_S = np.eye(4)
                        T_A_S[:3, 3] = position
                        Tz = np.eye(4)
                        Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
                        T_A_S = T_A_S @ Tz
                        self.cabinet_model = self.create_cabinet_model([width, height], door[7], T_A_S)
                        rospy.loginfo("[FSM] Cabinet model created from config.")
                        break

                    elif key == 'c':
                        rospy.loginfo("[FSM] Calling Door Detection Service...")
                        try:
                            rospy.wait_for_service('detect_door', timeout=5.0)
                            detect_door_srv = rospy.ServiceProxy('detect_door', DetectDoor)

                            curr_joints = self.robot.get_current_joint_values()
                            try:
                                T_6_0 = self.robot.get_tcp_pose()
                            except AttributeError:
                                T_6_0 = self.robot.get_current_tool_pose()

                            resp = detect_door_srv(
                                trigger=True,
                                base_dir=save_dir,
                                joint_values=curr_joints,
                                t_6_0=T_6_0.flatten().tolist()
                            )

                            if resp.success:
                                detect_data = json.loads(resp.result_json)
                                rospy.loginfo("[FSM] Door detected successfully!")
                                self._apply_detection_data(detect_data, door_index)
                                break
                            else:
                                rospy.logwarn("[FSM] Detection failed. Try again.")

                        except rospy.ROSException:
                            rospy.logerr("[FSM] 'detect_door' service not available!")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"[FSM] Detection service call failed: {e}")
                    else:
                        rospy.logwarn("[FSM] Invalid key. Press 'p' or 'c'.")

                self.state = FSMState.APPROACH

            # ---------------------------------------------------------------
            # APPROACH: plan multi-contact path for the detected cabinet model
            # ---------------------------------------------------------------
            elif self.state == FSMState.APPROACH:

                rospy.loginfo("[FSM] Calling PlanMultiContactPath service...")
                trajectory = self.plan_path_for_door(door_index)

                if trajectory is None:
                    rospy.logerr("[FSM] Could not obtain a trajectory. Moving to ERROR.")
                    self.state = FSMState.ERROR
                    continue

                self._planned_trajectory = trajectory
                self.set_new_touch_scene(state_angle=0.0)
                rospy.loginfo(f"[FSM] Path ready ({self._planned_trajectory.shape[0]} waypoints). Moving to OPENING.")
                self.state = FSMState.OPENING

            # ---------------------------------------------------------------
            # OPENING: execute trajectory with force/tactile monitoring
            # ---------------------------------------------------------------
            elif self.state == FSMState.OPENING:

                trajectory = self._planned_trajectory
                if trajectory is None:
                    rospy.logerr("[FSM] No trajectory in OPENING state. Moving to ERROR.")
                    self.state = FSMState.ERROR
                    continue

                # --- Phase 1: Approach waypoints (first 3) ---
                rospy.loginfo("[FSM] Executing approach phase...")
                approach_success = self.execute_with_force_monitoring(
                    trajectory[:3],
                    self.force_threshold_approach,
                    max_velocity=0.5,
                    max_acceleration=0.5
                )

                if not approach_success:
                    rospy.logwarn("[FSM] Approach phase failed (force exceeded). Moving to REPLAN.")
                    self.state = FSMState.REPLAN
                    continue

                # --- Phase 2: Opening (remaining waypoints) ---
                rospy.loginfo("[FSM] Executing opening phase...")
                open_trajectory = trajectory[2:]

                tactile_loss_joints = []
                tactile_establish_joints = []
                self.robot.tactile_contact_established = False

                monitor_thread_loss = threading.Thread(
                    target=monitor_tactile_loss_and_remember_joints,
                    args=(self.robot, tactile_loss_joints, 0.3, 4.0, 50.0)
                )
                monitor_thread_establish = threading.Thread(
                    target=monitor_tactile_contact_establish,
                    args=(self.robot, tactile_establish_joints, 0.5, 4.0, 50.0)
                )

                monitor_thread_establish.start()
                monitor_thread_loss.start()

                open_success = self.robot.send_joint_trajectory_action2(
                    open_trajectory, max_velocity=0.2, max_acceleration=0.2)

                monitor_thread_establish.join()
                monitor_thread_loss.join()

                # --- Evaluate outcome ---
                if open_success and len(tactile_establish_joints) > 0:
                    rospy.loginfo("[FSM] Door opened successfully with tactile contact!")
                    self.state = FSMState.RECORD_RESULT

                elif open_success and len(tactile_establish_joints) == 0:
                    rospy.logwarn("[FSM] Finished but no tactile contact established (miss).")
                    if self.correct_on_touch:
                        self.correct_touch_model()
                    self.state = FSMState.REPLAN

                else:
                    # Tactile loss during opening
                    if len(tactile_loss_joints) > 0:
                        rospy.logwarn("[FSM] Tactile loss detected. Triggering touch correction.")
                        tactile_loss_arr = np.array(tactile_loss_joints)

                        T_6before_0 = self.robot.get_fwd_kinematics_moveit(
                            tactile_loss_arr[-2] if tactile_loss_arr.shape[0] >= 2 else open_trajectory[0]
                        )
                        T_6miss_0 = self.robot.get_fwd_kinematics_moveit(tactile_loss_arr[-1])

                        self.set_touch(
                            T_6miss_0,
                            np.array([T_6before_0, T_6miss_0]),
                            TouchType.MISS,
                            0.0
                        )

                        if self.correct_on_touch:
                            self.correct_touch_model()
                            rospy.loginfo("[FSM] Cabinet model corrected from touch.")
                    else:
                        rospy.logwarn("[FSM] Opening failed with no tactile data.")

                    self.state = FSMState.REPLAN

            # ---------------------------------------------------------------
            # TOUCH_CORRECT: correct model, then replan
            # ---------------------------------------------------------------
            elif self.state == FSMState.TOUCH_CORRECT:
                rospy.loginfo("[FSM] Correcting touch model...")
                self.correct_touch_model()
                self.state = FSMState.REPLAN

            # ---------------------------------------------------------------
            # REPLAN: call path planning service with updated cabinet model
            # ---------------------------------------------------------------
            elif self.state == FSMState.REPLAN:
                if replan_count >= max_replan_attempts:
                    rospy.logwarn(f"[FSM] Max replan attempts ({max_replan_attempts}) reached.")
                    self.state = FSMState.RECORD_RESULT
                    continue

                replan_count += 1
                rospy.loginfo(f"[FSM] Replanning attempt {replan_count}/{max_replan_attempts}...")

                # Use corrected cabinet model pose for replanning
                T_R_W = self.cabinet_model.T_A_W if hasattr(self.cabinet_model, 'T_A_W') else np.eye(4)
                trajectory = self.plan_path_for_door(door_index, T_R_W=T_R_W)

                if trajectory is not None:
                    self._planned_trajectory = trajectory
                    self.set_new_touch_scene(state_angle=0.0)
                    self.state = FSMState.OPENING
                else:
                    rospy.logwarn(f"[FSM] Replan attempt {replan_count} failed.")
                    if replan_count >= max_replan_attempts:
                        self.state = FSMState.RECORD_RESULT

            # ---------------------------------------------------------------
            # RECORD_RESULT
            # ---------------------------------------------------------------
            elif self.state == FSMState.RECORD_RESULT:
                door_opened = (replan_count < max_replan_attempts)
                rospy.loginfo(f"[FSM] Experiment done. door_opened={door_opened}, replans={replan_count}")
                self.state = FSMState.DONE

            # ---------------------------------------------------------------
            # DONE / ERROR
            # ---------------------------------------------------------------
            elif self.state == FSMState.DONE:
                return True

            elif self.state == FSMState.ERROR:
                rospy.logerr("[FSM] Experiment ended in ERROR state.")
                return False

    def run(self):
        """Main loop over all door configurations."""
        results_data = read_csv_DataFrame(self.results) if os.path.exists(self.results) else None

        if results_data is not None:
            success_indices = results_data.index[
                (results_data['path_found'] == True) &
                (results_data['traj_success'] == True) &
                (results_data['contact_free'] == True) &
                (results_data['door_opened'] == True)
            ].tolist()
        else:
            success_indices = list(range(self.doors.shape[0]))

        for idx in success_indices:
            rospy.loginfo(f"=== Starting experiment for door {idx} ===")
            success = self.run_single_experiment(idx)
            rospy.loginfo(f"=== Door {idx} result: {'SUCCESS' if success else 'FAILED'} ===")


if __name__ == '__main__':
    cfg_path = os.path.join(os.path.dirname(__file__), '../cfg/config.yaml')
    with open(cfg_path, 'r') as f:
        config = yaml.safe_load(f)
    
    controller = MultiContactForceController(config)
    controller.run()