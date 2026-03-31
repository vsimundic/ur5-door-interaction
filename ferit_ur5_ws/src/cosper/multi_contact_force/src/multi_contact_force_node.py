#!/usr/bin/env python
"""
Fused multi-contact path planning with sensor-based door opening.

Flow:
1. Detect door via camera service
2. For each cabinet, create a Cabinet model
3. Plan multi-contact path via service
4. Execute with force monitoring
5. On failure -> replan
"""

import rospy
import os, sys
import numpy as np
import yaml
import threading
import json
from enum import Enum
from door_detection.srv import DetectDoor, DetectDoorState
from path_planning.srv import PlanMultiContactPath

from core.real_ur5_controller import UR5Controller
from core.transforms import rot_z
from gazebo_push_open.cabinet_model2 import Cabinet2

# Reuse utilities from force_manipulation
sys.path.append(os.path.join(os.path.dirname(__file__), 
    '../../force_manipulation/src'))
from force_utils import monitor_force_and_cancel


class FSMState(Enum):
    IDLE         = "IDLE"
    INITIALIZE   = "INITIALIZE"
    DETECT_STATE  = "DETECT_STATE"
    PLAN         = "PLAN"
    OPENING      = "OPENING"
    RECORD_RESULT = "RECORD_RESULT"
    DONE         = "DONE"
    ERROR        = "ERROR"


class MultiContactForceController:

    def __init__(self, config: dict):
        rospy.init_node('multi_contact_force_node')

        # --- Robot ---
        self.robot = UR5Controller(rvl_cfg_path=config['rvl_cfg_path'])
        self.T_R_W = np.eye(4)

        # --- Camera parameters ---
        self.T_C_6 = np.load(config['T_C_6_path'])

        # --- Cabinet parameters ---
        self.door_thickness = config.get('door_thickness', 0.018)
        self.static_depth   = config.get('static_depth', 0.4)

        # --- Load door configurations ---
        self.door_configs_path = config['door_configs_path']
        self.doors = np.load(self.door_configs_path)

        # --- Force thresholds ---
        self.force_threshold_approach = config.get('force_threshold_approach', 30.0)
        self.force_threshold_opening  = config.get('force_threshold_opening', 40.0)

        # --- Door Detection ---
        raw_detection_dir = config.get("detection_base_dir", "~/data/online_detection")
        self.detection_base_dir = os.path.expanduser(raw_detection_dir)
        if not os.path.exists(self.detection_base_dir):
            os.makedirs(self.detection_base_dir)
        self.state_detection_dir = config.get("state_detection_dir", "state_detection")

        self.load_existing_models = config.get('load_existing_models', True)

        # --- Path Planning Service parameters ---
        self.axis_distance                  = config.get('axis_distance', 0.01)
        self.static_side_width              = config.get('static_side_width', 0.017)
        self.moving_to_static_part_distance = config.get('moving_to_static_part_distance', 0.005)
        self.plan_num_states                = config.get('plan_num_states', 37)
        self.plan_target_angle              = config.get('plan_target_angle', -90.0)
        self.axis_pos                       = -1
        self.door_state_angle               = 0.0


        # --- Robot pose data ---
        self.joint_values_detection = None
        self.T_6_0_detection = None

        # --- FSM state ---
        self.state          = FSMState.IDLE
        self.cabinet_model  = None

    # ------------------------------------------------------------------
    # Cabinet model
    # ------------------------------------------------------------------

    def create_cabinet_model(self, door_params, axis_pos, T_A_W):
        """Create Cabinet2 model from door parameters."""
        width, height, rx, ry = door_params
        return Cabinet2(
            s=np.array([self.door_thickness, width, height, self.static_depth]),
            r=np.array([rx, ry]),
            axis_pos=axis_pos,
            T_A_W=T_A_W,
            save_path=None,
            has_handle=False
        )

    def _apply_detection_data(self, detect_data: dict, door_index: int):
        """Parse JSON detection data and build the cabinet model."""
        door_cfg = self.doors[door_index]

        # Extract dimensions (with fallback to config)
        if 'w' in detect_data and 'h' in detect_data:
            width  = float(detect_data['w'][0])
            height = float(detect_data['h'][0])
        elif 's' in detect_data:
            width  = float(detect_data['s'][0])
            height = float(detect_data['s'][1])
        else:
            rospy.logwarn("[FSM] No size info in detection result. Using config dimensions.")
            width, height = float(door_cfg[0]), float(door_cfg[1])

        if 'r' in detect_data:
            rx = float(detect_data['r'][0])
            ry = float(detect_data['r'][1])
        else:
            rospy.logwarn("[FSM] No door panel info in detection result. Using config info.")
            rx, ry = float(0.0), float(-0.5*width)

        if 'joint_values' in detect_data:
            self.joint_values_detection = detect_data['joint_values']
            self.T_6_0_detection = self.robot.get_fwd_kinematics_moveit(self.joint_values_detection)

        # Extract pose
        if 'T_A_C' in detect_data:
            R_A_C = np.array(detect_data['R']).reshape(3, 3)
            t_A_C = np.array(detect_data['t']).reshape(3)
            T_A_C = np.eye(4)
            T_A_C[:3, :3] = R_A_C
            T_A_C[:3, 3] = t_A_C
            T_A_0 = self.T_6_0_detection @ self.T_C_6 @ T_A_C
        else:
            rospy.logwarn("[FSM] T_A_0 missing in detection result. Using config pose.")
            T_A_0 = np.eye(4)
            T_A_0[:3, 3] = door_cfg[2:5]

        T_A_W = self.T_R_W @ T_A_0

        axis_pos = int(self.axis_pos)
        self.cabinet_model = self.create_cabinet_model([width, height, rx, ry], axis_pos, T_A_W)
        rospy.loginfo(f"[FSM] Cabinet model built: W={width:.3f}, H={height:.3f}")

    # ------------------------------------------------------------------
    # Path planning
    # ------------------------------------------------------------------

    def plan_path_for_door(self, door_index: int, T_R_W: np.ndarray = None) -> np.ndarray:
        """Call the path planning service using self.cabinet_model."""
        rospy.loginfo(f"[FSM] Calling path planning service for door {door_index}...")

        if self.cabinet_model is None:
            rospy.logerr("[FSM] No cabinet model available. Run INITIALIZE first.")
            return None

        try:
            rospy.wait_for_service('plan_multi_contact_path', timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("[FSM] 'plan_multi_contact_path' service not available!")
            return None

        plan_srv = rospy.ServiceProxy('plan_multi_contact_path', PlanMultiContactPath)

        width    = float(self.cabinet_model.sy)
        height   = float(self.cabinet_model.sz)
        T_A_S    = self.cabinet_model.T_A_W.copy()
        axis_pos = int(self.cabinet_model.axis_pos)
        # rx       = float(self.cabinet_model.rx)
        # ry       = float(self.cabinet_model.ry)
        rx       = 0.0
        ry       = -0.5 * width

        # state_angle is an experiment parameter, not a model property
        # state_angle = float(self.doors[door_index][6])
        state_angle = self.door_state_angle

        if T_R_W is None:
            T_R_W = np.eye(4)

        # Build q_init and apply UR5 offsets
        q_init = self.robot.get_current_joint_values().copy()
        q_init[0] += np.pi
        q_init[5] += np.pi
        q_init[q_init >  np.pi] -= (2.0 * np.pi)
        q_init[q_init < -np.pi] += (2.0 * np.pi)

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
                rospy.logwarn(f"[FSM] No path found for door {door_index}.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"[FSM] Path planning service call failed: {e}")
            return None

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def execute_with_force_monitoring(self, trajectory, force_threshold,
                                      max_velocity=0.5, max_acceleration=0.5):
        """Execute trajectory with force monitoring."""
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

    # ------------------------------------------------------------------
    # FSM
    # ------------------------------------------------------------------

    def run_single_experiment(self, door_index: int) -> bool:
        self.state             = FSMState.INITIALIZE
        self.current_door_index = door_index
        max_replan_attempts    = 3
        replan_count           = 0
        self._planned_trajectory = None
        self.cabinet_model     = None

        door_detection_save_dir   = os.path.join(self.detection_base_dir, f"door_{door_index}")
        model_path = os.path.join(door_detection_save_dir, "models", "doorModel.json")

        while not rospy.is_shutdown():
            rospy.loginfo("[FSM] State: %s (door %d)" % (self.state.value, door_index))

            # -----------------------------------------------------------
            # INITIALIZE: detect door, build cabinet model
            # -----------------------------------------------------------
            if self.state == FSMState.INITIALIZE:

                if self.load_existing_models and os.path.exists(model_path):
                    rospy.loginfo(f"[FSM] Loading existing model from {model_path}...")
                    try:
                        with open(model_path, 'r') as f:
                            detect_data = json.load(f)
                        self._apply_detection_data(detect_data, door_index)
                        rospy.loginfo("[FSM] Model loaded. Moving to DETECT_STATE.")
                        self.state = FSMState.DETECT_STATE
                        continue

                    except Exception as e:
                        rospy.logwarn(f"[FSM] Failed to load model: {e}. Falling back to prompt.")

                while not rospy.is_shutdown():
                    key = input("[FSM] Press 'c' to detect door via camera: ").strip().lower()

                    if key == 'c':
                        rospy.loginfo("[FSM] Calling Door Detection Service...")
                        
                        try:
                            rospy.wait_for_service('detect_door', timeout=5.0)
                            detect_door_srv = rospy.ServiceProxy('detect_door', DetectDoor)

                            curr_joints = self.robot.get_current_joint_values()
                            T_6_0       = self.robot.get_current_tool_pose()

                            resp = detect_door_srv(
                                trigger=True,
                                base_dir=door_detection_save_dir,
                                joint_values=curr_joints,
                                T_6_0=T_6_0.flatten().tolist()
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
                        rospy.logwarn("[FSM] Invalid key. Press 'c'.")

                self.state = FSMState.DETECT_STATE

            # -----------------------------------------------------------
            # DETECT_STATE: detect door state
            # -----------------------------------------------------------
            elif self.state == FSMState.DETECT_STATE:
                rospy.loginfo("[FSM] Calling DetectDoorState service...")
                state_detection_path = os.path.join(door_detection_save_dir, self.state_detection_dir, "detected_state.json")
                if self.load_existing_models and os.path.exists(state_detection_path):
                    rospy.loginfo(f"[FSM] Loading existing model for state detection from {state_detection_path}...")
                    try:
                        with open(state_detection_path, 'r') as f:
                            detect_data = json.load(f)
                        self.door_state_angle = detect_data.get('state_angle_deg', 0.0)
                        rospy.loginfo(f"[FSM] Loaded state angle: {self.door_state_angle:.2f} deg")
                        self.state = FSMState.PLAN
                        continue

                    except Exception as e:
                        rospy.logwarn(f"[FSM] Failed to load model for state detection: {e}. Falling back to service call.")

                try:
                    rospy.wait_for_service('detect_door_state', timeout=5.0)
                    detect_door_state_srv = rospy.ServiceProxy('detect_door_state', DetectDoorState)

                    # Get current robot joint values
                    current_joint_values = self.robot.get_current_joint_values()

                    # Move the robot to detection joint configuration before state detection
                    self.robot.send_joint_trajectory_action2(
                        np.vstack((np.array(current_joint_values), np.array(self.joint_values_detection))),
                        max_velocity=0.5, max_acceleration=0.5
                    )

                    # Call the service
                    resp = detect_door_state_srv(
                        base_dir=door_detection_save_dir,
                        state_detection_dir=self.state_detection_dir,
                        T_Cdetected_Cstate=np.eye(4).reshape(-1).tolist()
                    )

                    if resp.success:
                        rospy.loginfo("[FSM] Door state detected successfully!")
                        self.door_state_angle = resp.state_angle_deg
                        self.state = FSMState.PLAN
                    else:
                        rospy.logwarn("[FSM] Door state detection failed.")
                        rospy.logwarn("[FSM] Error message: %s" % resp.message)

                except rospy.ROSException:
                    rospy.logerr("[FSM] 'detect_door_state' service not available!")
                except rospy.ServiceException as e:
                    rospy.logerr(f"[FSM] Detection service call failed: {e}")

            # -----------------------------------------------------------
            # PLAN: plan multi-contact path
            # -----------------------------------------------------------
            elif self.state == FSMState.PLAN:

                rospy.loginfo("[FSM] Calling PlanMultiContactPath service...")
                trajectory = self.plan_path_for_door(door_index, self.T_R_W)

                if trajectory is None:
                    if replan_count >= max_replan_attempts:
                        rospy.logerr("[FSM] Max replan attempts reached. Moving to ERROR.")
                        self.state = FSMState.ERROR
                    else:
                        replan_count += 1
                        rospy.logwarn(f"[FSM] Planning failed. Retry {replan_count}/{max_replan_attempts}.")
                    continue

                self._planned_trajectory = trajectory
                rospy.loginfo(f"[FSM] Path ready ({trajectory.shape[0]} waypoints). Moving to OPENING.")
                # self.state = FSMState.OPENING

            # -----------------------------------------------------------
            # OPENING: execute with force monitoring
            # -----------------------------------------------------------
            elif self.state == FSMState.OPENING:

                trajectory = self._planned_trajectory

                # Phase 1: approach
                rospy.loginfo("[FSM] Executing approach phase...")
                approach_success = self.execute_with_force_monitoring(
                    trajectory[:3],
                    self.force_threshold_approach,
                    max_velocity=0.5, max_acceleration=0.5
                )

                if not approach_success:
                    rospy.logwarn("[FSM] Approach failed (force exceeded).")
                    if replan_count < max_replan_attempts:
                        replan_count += 1
                        self.state = FSMState.PLAN
                    else:
                        self.state = FSMState.RECORD_RESULT
                    continue

                # Phase 2: opening
                rospy.loginfo("[FSM] Executing opening phase...")
                open_success = self.execute_with_force_monitoring(
                    trajectory[2:],
                    self.force_threshold_opening,
                    max_velocity=0.2, max_acceleration=0.2
                )

                if not open_success:
                    rospy.logwarn("[FSM] Opening failed (force exceeded).")
                    if replan_count < max_replan_attempts:
                        replan_count += 1
                        self.state = FSMState.PLAN
                        continue

                self.state = FSMState.RECORD_RESULT

            # -----------------------------------------------------------
            # RECORD_RESULT
            # -----------------------------------------------------------
            elif self.state == FSMState.RECORD_RESULT:
                success = (self.state != FSMState.ERROR)
                rospy.loginfo(f"[FSM] Experiment done. replans={replan_count}")
                self.state = FSMState.DONE

            # -----------------------------------------------------------
            # DONE / ERROR
            # -----------------------------------------------------------
            elif self.state == FSMState.DONE:
                return True

            elif self.state == FSMState.ERROR:
                rospy.logerr("[FSM] Experiment ended in ERROR state.")
                return False

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        """Continuous live experiment loop."""
        rospy.loginfo("Commands: door index to run | 'r' repeat last | 'q' quit")

        last_door_index = None

        while not rospy.is_shutdown():
            try:
                print('Last door index: %s' % (last_door_index if last_door_index is not None else "None"))
                key = input("\n[RUN] Enter door index (or 'r' to repeat, 'q' to quit): ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("[RUN] Interrupted. Shutting down.")
                break

            if key == 'q':
                rospy.loginfo("[RUN] Quitting.")
                break

            elif key == 'r':
                if last_door_index is None:
                    rospy.logwarn("[RUN] No previous experiment to repeat.")
                    continue
                door_index = last_door_index
                rospy.loginfo(f"[RUN] Repeating door {door_index}")

            else:
                try:
                    door_index = int(key)
                except ValueError:
                    rospy.logwarn(f"[RUN] Invalid input '{key}'.")
                    continue

                if door_index < 0 or door_index >= self.doors.shape[0]:
                    rospy.logwarn(f"[RUN] Door index {door_index} out of range (0-{self.doors.shape[0]-1}).")
                    continue

            last_door_index = door_index
            rospy.loginfo(f"=== Starting experiment for door {door_index} ===")

            try:
                success = self.run_single_experiment(door_index)
            except Exception as e:
                rospy.logerr(f"[RUN] Unhandled exception: {e}")
                success = False

            rospy.loginfo(f"=== Door {door_index}: {'SUCCESS' if success else 'FAILED'} ===")

        rospy.loginfo("[RUN] Live experiment loop ended.")


if __name__ == '__main__':
    cfg_path = os.path.join(os.path.dirname(__file__), '../cfg/config.yaml')
    with open(cfg_path, 'r') as f:
        config = yaml.safe_load(f)

    controller = MultiContactForceController(config)
    controller.run()