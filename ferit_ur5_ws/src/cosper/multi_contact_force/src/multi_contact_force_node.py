#!/usr/bin/env python
"""
Fused multi-contact path planning with sensor-based door opening.

Flow:
1. Detect door via camera service
2. Detect door state (opening angle)
3. Plan multi-contact path via service
4. Execute approach -> insertion -> opening with force monitoring
5. On failure -> replan
"""

import rospy
import os, sys
import numpy as np
import yaml
import json
import csv
import copy
from enum import Enum
from door_detection.srv import DetectDoor, DetectDoorState
from path_planning.srv import PlanMultiContactPath

from core.real_ur5_controller import UR5Controller
from core.transforms import rot_z
from gazebo_push_open.cabinet_model2 import Cabinet2
from gazebo_push_open.cabinet_model import Cabinet

from trajectory_executor import TrajectoryExecutor

sys.path.append(os.path.join(os.path.dirname(__file__),
    '../../force_manipulation/src'))
from force_utils import TouchType

import RVLPYDDManipulator as rvlpy


class FSMState(Enum):
    IDLE          = "IDLE"
    INITIALIZE    = "INITIALIZE"
    DETECT_STATE  = "DETECT_STATE"
    PLAN          = "PLAN"
    APPROACH      = "APPROACH"
    INSERTION     = "INSERTION"
    OPENING       = "OPENING"
    RECORD_RESULT = "RECORD_RESULT"
    DONE          = "DONE"
    ERROR         = "ERROR"


class MultiContactForceController:

    def __init__(self, config: dict):
        rospy.init_node('multi_contact_force_node')

        # --- Robot ---
        self.robot = UR5Controller(rvl_cfg_path=config['rvl_cfg_path'])
        self.T_R_W = np.eye(4)

        # --- Trajectory executor ---
        use_tactile = config.get('use_tactile', False)
        self.executor = TrajectoryExecutor(self.robot, use_tactile=use_tactile)


        # --- Touch + Path Planning ---
        self.rvl_manipulator = rvlpy.PYDDManipulator()
        self.rvl_manipulator.create(config['rvl_cfg_path'])
        self.py_touch = self.rvl_manipulator.py_touch
        rvl_touch_cfg = config.get('rvl_touch_cfg', None)
        if rvl_touch_cfg:
            self.py_touch.create(rvl_touch_cfg)

        self.T_C_6_init      = None   # set after loading T_C_6
        self.T_A_C_init      = np.eye(4)
        self.T_6_0_capture   = np.eye(4)   # robot pose at detection time
        self.T_D_S           = np.eye(4)   # door-to-world at current state

        tool_cfg = config.get('tool', {})
        self.tactile_sensor_depth = config.get('tactile_sensor_depth', 0.006)
        _tx_E = tool_cfg.get('tx_E', 0.155 - self.tactile_sensor_depth)
        _tz_E = tool_cfg.get('tz_E', 0.278)
        _a = tool_cfg.get('a', 0.0205)
        _b = tool_cfg.get('b', 0.032)
        _c = tool_cfg.get('c', 0.011)
        _d = tool_cfg.get('d', 0.026)
        _h = tool_cfg.get('h', 0.023)
        _rotz = tool_cfg.get('rotz_angle', -np.pi * 0.25)
        # RVLTool builds T_tool_6 from these params
        sys.path.append(os.path.join(os.path.dirname(__file__),
            '../../force_manipulation/src'))
        from force_utils import RVLTool
        _rvl_tool = RVLTool(_a, _b, _c, _d, _h, -_tx_E, _tz_E, rot_z_angle=_rotz)
        self.T_TCP_6 = _rvl_tool.T_TCP_6
        self.py_touch.create_simple_tool(_a, _b, _c, _d, _h, _rvl_tool.T_tool_6)

        camera_cfg = config.get('camera', {})
        self.py_touch.set_camera_params(
            camera_cfg.get('fu', 597.9033203125),
            camera_cfg.get('fv', 598.47998046875),
            camera_cfg.get('uc', 323.8436584472656),
            camera_cfg.get('vc', 236.32774353027344),
            camera_cfg.get('width', 640),
            camera_cfg.get('height', 480),
        )

        touch_cfg = config.get('touch', {})
        self.touch_a = touch_cfg.get('a', config.get('static_depth', 0.4))
        self.touch_b = touch_cfg.get('b', 0.0)
        self.touch_c = touch_cfg.get('c', 0.005)

        # --- Camera parameters ---
        self.T_C_6 = np.load(config['T_C_6_path'])
        R_C_6 = self.T_C_6[:3, :3]
        I_ = R_C_6 @ R_C_6.T
        mean_trace = np.trace(I_) / 3.0
        self.scale_factor = np.sqrt(mean_trace)
        self.T_C_6[:3, :3] /= self.scale_factor
        self.T_C_6_init = self.T_C_6.copy()

        # --- Cabinet parameters ---
        self.door_thickness = config.get('door_thickness', 0.018)
        self.static_depth   = config.get('static_depth', 0.4)
        self.cabinet_mesh_path = config.get('cabinet_mesh_path', '')
        if not os.path.exists(os.path.dirname(self.cabinet_mesh_path)):
            os.makedirs(os.path.dirname(self.cabinet_mesh_path))

        # --- Load door configurations ---
        self.door_configs_path = config['door_configs_path']
        self.doors = np.load(self.door_configs_path)

        # --- Force thresholds ---
        self.force_threshold_approach  = config.get('force_threshold_approach', 30.0)
        self.force_threshold_insertion = config.get('force_threshold_insertion', 7.0)
        self.force_threshold_opening   = config.get('force_threshold_opening', 40.0)

        # --- Velocities / accelerations per phase ---
        self.approach_max_vel  = config.get('approach_max_vel', 1.0)
        self.approach_max_acc  = config.get('approach_max_acc', 0.7)
        self.insertion_max_vel = config.get('insertion_max_vel', 0.05)
        self.insertion_max_acc = config.get('insertion_max_acc', 0.05)
        self.opening_max_vel   = config.get('opening_max_vel', 0.2)
        self.opening_max_acc   = config.get('opening_max_acc', 0.2)

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
        self.T_6_0_detection        = None

        # --- FSM state ---
        self.state         = FSMState.IDLE
        self.cabinet_model = None

        # --- CSV logging ---
        csv_dir = os.path.expanduser(config.get('csv_dir', self.detection_base_dir))
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        self.cabinet_csv_path = os.path.join(csv_dir, 'cabinets_estimation.csv')
        self.touches_csv_path  = os.path.join(csv_dir, 'cabinets_touches.csv')
        self.session_idx  = 0
        self.door_log_idx = 0   # per-door experiment counter (door_index)
        self._init_csv_files()

    # ------------------------------------------------------------------
    # Cabinet model
    # ------------------------------------------------------------------

    def _init_csv_files(self):
        """Create CSV files with headers if they don't exist.
        If the cabinet CSV already exists, read the last session_idx so runs
        are appended with incrementing session indices (same pattern as
        door_replanning_control.py).
        """
        if not os.path.exists(self.cabinet_csv_path):
            with open(self.cabinet_csv_path, 'w') as f:
                header = ('session_idx,door_index,w_door,h_door,rx,ry,'
                          'state_angle,R_A_C,t_A_C,R_C_E,t_C_E,R_E_0,t_E_0')
                f.write(header + '\n')
        else:
            with open(self.cabinet_csv_path, 'r') as f:
                lines = f.readlines()
            if len(lines) > 1:
                try:
                    self.session_idx = int(lines[-1].strip().split(',')[0]) + 1
                except (ValueError, IndexError):
                    self.session_idx = 0

        if not os.path.exists(self.touches_csv_path):
            with open(self.touches_csv_path, 'w') as f:
                header = 'session_idx,door_index,type,R_Ek_E,t_Ek_E,V,t,state_angle'
                f.write(header + '\n')

    def save_cabinet_info_to_csv(self, door_index: int, state_angle: float):
        """Append one row per detection to the cabinet CSV.

        Mirrors save_models_info_for_rvl() in door_replanning_control.py.
        """
        if self.cabinet_model is None:
            rospy.logwarn("[CSV] No cabinet model — skipping cabinet CSV save.")
            return

        w = float(self.cabinet_model.w_door if hasattr(self.cabinet_model, 'w_door')
                  else self.cabinet_model.sy)
        h = float(self.cabinet_model.h_door if hasattr(self.cabinet_model, 'h_door')
                  else self.cabinet_model.sz)

        with open(self.cabinet_csv_path, 'a') as f:
            txt = (f'{self.session_idx},{door_index},'
                   f'{w},{h},'
                   f'{float(self.cabinet_model.rx)},{float(self.cabinet_model.ry)},'
                   f'{state_angle}')
            for v in self.T_A_C_init[:3, :3].reshape(-1):
                txt += f',{v}'
            for v in self.T_A_C_init[:3, 3].reshape(-1):
                txt += f',{v}'
            for v in self.T_C_6_init[:3, :3].reshape(-1):
                txt += f',{v}'
            for v in self.T_C_6_init[:3, 3].reshape(-1):
                txt += f',{v}'
            for v in self.T_6_0_capture[:3, :3].reshape(-1):
                txt += f',{v}'
            for v in self.T_6_0_capture[:3, 3].reshape(-1):
                txt += f',{v}'
            f.write(txt + '\n')
        rospy.loginfo(f"[CSV] Cabinet info saved (session={self.session_idx}, door={door_index}).")

    def save_touch_to_csv(self, T_Ek_E: np.ndarray, V: np.ndarray, t: float,
                          touch_type: TouchType, state_angle: float,
                          door_index: int):
        """Append one touch row to the touches CSV.

        Mirrors save_touches_info_for_rvl() in door_replanning_control.py.
        T_Ek_E is the contact pose expressed in the capture-time end-effector
        frame (frame 6 at detection time), matching the convention used in
        door_replanning_control.
        """
        R_flat = T_Ek_E[:3, :3].reshape(-1)
        t_flat = T_Ek_E[:3, 3].reshape(-1)
        V_flat = V.reshape(-1)

        with open(self.touches_csv_path, 'a') as f:
            txt = f'{self.session_idx},{door_index},{touch_type.value}'
            for v in R_flat:
                txt += f',{v}'
            for v in t_flat:
                txt += f',{v}'
            for v in V_flat:
                txt += f',{v}'
            txt += f',{t},{state_angle}'
            f.write(txt + '\n')
        rospy.loginfo(f"[CSV] Touch saved (session={self.session_idx}, door={door_index}, "
                      f"type={touch_type.value}, state={state_angle:.2f}).")

    def create_cabinet_model(self, door_params, axis_pos, T_A_W):
        """Create cabinet model from door parameters."""
        width, height, rx, ry = door_params
        return Cabinet(
            # door_params=np.array([self.door_thickness, width, height, self.static_depth]), # Cabinet2
            door_params=np.array([width, height, self.door_thickness, self.static_depth]),
            r=np.array([rx, ry]),
            axis_pos=axis_pos,
            T_A_S=T_A_W,
            save_path=None,
            has_handle=False,
            axis_distance=self.axis_distance,
            static_side_width=self.static_side_width,
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
            rx, ry = float(0.0), float(-0.5 * width)

        if 'joint_values' in detect_data:
            self.joint_values_detection = detect_data['joint_values']
            self.T_6_0_detection = self.robot.get_fwd_kinematics_moveit(
                self.joint_values_detection)

        # Extract pose
        if 'T_A_C' in detect_data:
            R_A_C = np.array(detect_data['R']).reshape(3, 3)
            t_A_C = np.array(detect_data['t']).reshape(3)
            T_A_C = np.eye(4)
            T_A_C[:3, :3] = R_A_C
            T_A_C[:3, 3]  = t_A_C
            T_A_0 = self.T_6_0_detection @ self.T_C_6 @ T_A_C
        else:
            rospy.logwarn("[FSM] T_A_0 missing in detection result. Using config pose.")
            T_A_0 = np.eye(4)
            T_A_0[:3, 3] = door_cfg[2:5]

        T_A_W = self.T_R_W @ T_A_0
        axis_pos = int(self.axis_pos)
        self.cabinet_model = self.create_cabinet_model(
            [width, height, rx, ry], axis_pos, T_A_W)
        rospy.loginfo(f"[FSM] Cabinet model built: W={width:.3f}, H={height:.3f}")

        if self.T_6_0_detection is not None:
            self.T_6_0_capture = self.T_6_0_detection.copy()
        if 'T_A_C' in detect_data:
            self.T_A_C_init = T_A_C.copy()
        else:
            self.T_A_C_init = np.linalg.inv(self.T_C_6) @ np.linalg.inv(self.T_6_0_capture) @ T_A_0

    # ------------------------------------------------------------------
    # Trajectory segmentation
    # ------------------------------------------------------------------

    @staticmethod
    def segment_trajectory(trajectory: np.ndarray):
        """
        Split a full trajectory into three phases.

        Convention (from door_replanning_control):
          approach  = trajectory[0:3]   (free-space move to pre-contact)
          insertion = trajectory[2:4]   (slow push into door surface)
          opening   = trajectory[4:]    (sweep the door open)
        """
        if trajectory.shape[0] < 4:
            rospy.logwarn("[FSM] Trajectory too short (%d pts). "
                          "Using full trajectory for all phases.",
                          trajectory.shape[0])
            return trajectory, trajectory, trajectory

        approach  = trajectory[:3]
        insertion = trajectory[2:4]
        opening   = trajectory[4:]
        return approach, insertion, opening

    # ------------------------------------------------------------------
    # Path planning
    # ------------------------------------------------------------------

    def _add_cabinet_model_to_scene(self, state_angle, is_doorless=False, scale_factor=1.0):
        rospy.loginfo("[FSM] Adding cabinet model to scene...")
        # self.cabinet_model.save_door_panel_mesh(self.cabinet_panel_mesh_path)
        self.robot.remove_mesh_from_scene("cabinet_model")

        TArot_A = np.eye(4)
        TArot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
        # T_Arot_W = self.cabinet_model.T_A_W if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.T_A_S @ TArot_A

        if type(self.cabinet_model) == Cabinet:
            self.cabinet_model.update_mesh()

        mesh = copy.deepcopy(self.cabinet_model.static_mesh if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.mesh)
        center = np.array([0.0, 0.0, 0.0])

        # mesh.transform(np.linalg.inv(self.cabinet_model.T_A_O))
        self.cabinet_model.change_door_angle(state_angle)
        self.cabinet_model.create_mesh()
        T_O_D = np.linalg.inv(self.cabinet_model.T_D_A) @ np.linalg.inv(self.cabinet_model.T_A_O)

        mesh_path = self.cabinet_mesh_path
        
        if is_doorless:
            # Add doorless cabinet model to the scene
            # static_mesh.transform(np.linalg.inv(self.cabinet_model.T_A_O_init))
            # center = np.array([0.0, 0.0, 0.0])
            mesh.scale(scale_factor, center)
            self.cabinet_model.save_mesh_without_doors(mesh_path, mesh=mesh, pose=T_O_D)
        else:
            mesh = copy.deepcopy(self.cabinet_model.mesh)
            
            mesh.scale(scale_factor, center)
            self.cabinet_model.save_mesh(mesh_path, mesh=mesh, pose=T_O_D)
    

        self.robot.add_mesh_to_scene(mesh_path, "cabinet_model", self.T_D_S)

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

    def _setup_rvl_manipulator_for_door(self, T_R_W: np.ndarray = None):
        """Push the current cabinet model and robot pose into the persistent
        rvl_manipulator so path planning and touch correction share the same state."""
        if self.cabinet_model is None:
            return

        T_R_W = T_R_W if T_R_W is not None else self.T_R_W
        self.rvl_manipulator.set_robot_pose(T_R_W)

        width  = float(self.cabinet_model.sy if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.w_door)
        height = float(self.cabinet_model.sz if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.h_door)
        T_A_S  = self.cabinet_model.T_A_W.copy() if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.T_A_S.copy()

        self.rvl_manipulator.set_door_model_params(
            float(self.door_thickness),
            width,
            height,
            float(self.cabinet_model.rx),
            float(self.cabinet_model.ry),
            float(self.cabinet_model.axis_pos),
            float(self.static_side_width),
            float(self.moving_to_static_part_distance),
        )
        self.rvl_manipulator.set_door_pose(T_A_S)
        self.rvl_manipulator.set_environment_state(self.door_state_angle)

    def plan_path_for_door(self, door_index: int, T_R_W: np.ndarray = None) -> np.ndarray:
        rospy.loginfo(f"[FSM] Planning path for door {door_index} ...")

        if self.cabinet_model is None:
            rospy.logerr("[FSM] No cabinet model available. Run INITIALIZE first.")
            return None

        # Sync model into the shared instance
        self._setup_rvl_manipulator_for_door(T_R_W)

        # Build q_init with UR5 offsets expected by path2()
        q_init = self.robot.get_current_joint_values().copy()
        q_init[0] += np.pi
        q_init[5] += np.pi
        # q_init[q_init >  np.pi] -= (2.0 * np.pi)
        # q_init[q_init < -np.pi] += (2.0 * np.pi)

        try:
            from numpy.core._exceptions import _ArrayMemoryError
            T_G_0_array, q = self.rvl_manipulator.path2(
                q_init, float(self.plan_target_angle), int(self.plan_num_states), False)

            if T_G_0_array.shape[0] > 1:
                q = q.astype(np.float64)
                q[:, 0] -= np.pi
                q[:, 5] -= np.pi
                # Unwrap BEFORE any clamping so there are no artificial
                # discontinuities introduced by the ±π boundary crossing.
                # q = np.unwrap(q, axis=0)
                rospy.loginfo(f"[FSM] Path found: {q.shape[0]} waypoints.")
                return q
            else:
                rospy.logwarn(f"[FSM] No path found for door {door_index}.")
                return None

        except (_ArrayMemoryError, ValueError, RuntimeError) as e:
            rospy.logerr(f"[FSM] Path planning failed: {e}")
            return None

    def set_new_touch_scene(self, state_angle: float):
        """Initialise py_touch for a new scene (call once after detection)."""
        if self.cabinet_model is None:
            rospy.logerr("[Touch] Cabinet model not available.")
            return
        width  = float(self.cabinet_model.sy if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.w_door)
        height = float(self.cabinet_model.sz if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.h_door)
        rx     = float(self.cabinet_model.rx)
        ry     = float(self.cabinet_model.ry)
        TArot_A = np.eye(4)
        TArot_A[:3, :3] = rot_z(np.radians(state_angle))
        T_Arot_0_gt = self.T_6_0_capture @ self.T_C_6 @ self.T_A_C_init @ TArot_A
        self.py_touch.set_new_scene(
            float(self.door_thickness), width, height, rx, ry,
            self.touch_a, self.touch_b, self.touch_c, state_angle,
            self.T_C_6_init, self.T_A_C_init, self.T_6_0_capture,
            float(self.door_thickness), width, height, rx, ry,
            state_angle, T_Arot_0_gt)
        rospy.loginfo("[Touch] New touch scene set.")

    def set_touch(self, T_6contact_0: np.ndarray, T_6via_0_pts: np.ndarray,
                  touch_type: TouchType):
        """Record one touch observation into py_touch.

        Args:
            T_6contact_0:  4x4 contact pose in robot base frame (frame 0).
            T_6via_0_pts:  (N,4,4) via-point poses in robot base frame — at least
                           two poses; the approach direction V is computed from
                           pts[1] - pts[0].
            touch_type:    TouchType.UNWANTED_TOUCH or TouchType.MISS or TouchType.WANTED_TOUCH.
            state_angle:   Current door opening angle [deg].
        """
        T_6contact_6 = np.linalg.inv(self.T_6_0_capture) @ T_6contact_0
        T_6via_6     = np.linalg.inv(self.T_6_0_capture)[np.newaxis] @ T_6via_0_pts
        V = T_6via_6[1, :3, 3] - T_6via_6[0, :3, 3]
        t = float(np.linalg.norm(V))
        if t > 1e-9:
            V /= t
        b_miss = (touch_type == TouchType.MISS)
        self.py_touch.set_touch(T_6contact_6, V, t, b_miss)
        rospy.loginfo(f"[Touch] Touch recorded (miss={b_miss}, t={t:.4f}).")

        # Persist to CSV
        door_idx = getattr(self, 'current_door_index', -1)
        self.save_touch_to_csv(T_6contact_6, V, t, touch_type,
                               self.door_state_angle, door_idx)

    def correct_touch_model(self):
        """Run touch correction and propagate results into rvl_manipulator + cabinet_model."""
        rospy.loginfo("[Touch] Running py_touch.correct()...")
        self.py_touch.correct()
        self._update_model_from_touch()

    def _update_model_from_touch(self):
        """Pull corrected poses out of rvl_manipulator and update all shared state.

        This mirrors update_model_from_touch() in door_replanning_control.py.
        Because py_touch and manipulator (DDManipulator) live inside the *same*
        PYDDManipulator object, update_model_x() / get_corrected_* work correctly.
        """
        # 1. Pull touch model into manipulator.model_x
        self.rvl_manipulator.update_model_x()

        T_Arot_6_corrected = self.rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
        T_C_6_corrected    = self.rvl_manipulator.get_corrected_camera_pose().astype(np.float64)

        if np.any(np.isnan(T_Arot_6_corrected)) or np.any(np.isnan(T_C_6_corrected)):
            rospy.logerr("[Touch] Corrected poses contain NaN — keeping previous model.")
            return

        # 2. Update camera extrinsics
        self.T_C_6 = T_C_6_corrected.copy()
        self.robot.T_C_6 = self.T_C_6   # keep robot controller in sync

        # 3. Update cabinet pose (T_A_W) — state_angle cancels out as in original
        T_Arot_A = np.eye(4)   # identity: corrections are already in world frame
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(self.door_state_angle))
        T_A_C = np.linalg.inv(self.T_C_6) @ T_Arot_6_corrected @ np.linalg.inv(T_Arot_A)
        T = (self.T_R_W
            @ self.T_6_0_capture
            @ self.T_C_6
            @ T_A_C)
        if type(self.cabinet_model) == Cabinet2:
            self.cabinet_model.T_A_W = T.copy()
        else:
            self.cabinet_model.T_A_S = T.copy()
        rospy.loginfo(f"[Touch] Corrected cabinet T_A_W:\n{self.cabinet_model.T_A_W if type(self.cabinet_model) == Cabinet2 else self.cabinet_model.T_A_S}")

        # 4. Update door-contact frame
        T_D_Arot = self.rvl_manipulator.get_corrected_pose_D_Arot().astype(np.float64)
        if type(self.cabinet_model) == Cabinet2:
            self.cabinet_model.T_D_Arot = T_D_Arot.copy()
        else:
            self.cabinet_model.T_D_A = T_D_Arot.copy()
            self.cabinet_model.T_D_A_init = T_D_Arot.copy()

        # 5. Update T_D_S (door frame in world) and push to manipulator
        T_D_0  = self.rvl_manipulator.get_corrected_pose_D_0().astype(np.float64)
        self.T_D_S = self.T_R_W @ T_D_0
        # self.rvl_manipulator.set_pose_DD_S(self.T_D_S)

        # 6. Update the VN environment model from touch so the *next* path2() call
        #    operates on the corrected geometry — this is the key step that makes
        #    replanning after touch correction work correctly.
        # self.rvl_manipulator.set_environment_from_touch()

        rospy.loginfo("[Touch] Model update from touch complete.")

    def run_single_experiment(self, door_index: int) -> bool:
        self.state               = FSMState.INITIALIZE
        self.current_door_index  = door_index
        max_replan_attempts      = 3
        replan_count             = 0
        self._planned_trajectory = None
        self.cabinet_model       = None
        self.door_state_angle    = 0.0

        # Trajectory segments — populated after PLAN
        approach_traj  = None
        insertion_traj = None
        opening_traj   = None

        door_detection_save_dir = os.path.join(
            self.detection_base_dir, f"door_{door_index}")
        model_path = os.path.join(
            door_detection_save_dir, "models", "doorModel.json")

        while not rospy.is_shutdown():
            rospy.loginfo("[FSM] State: %s (door %d)" % (self.state.value, door_index))

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

            elif self.state == FSMState.DETECT_STATE:

                state_detection_path = os.path.join(
                    door_detection_save_dir,
                    self.state_detection_dir, "detected_state.json")

                if self.load_existing_models and os.path.exists(state_detection_path):
                    rospy.loginfo(
                        f"[FSM] Loading existing state detection from {state_detection_path}...")
                    try:
                        with open(state_detection_path, 'r') as f:
                            detect_data = json.load(f)
                        self.door_state_angle = detect_data.get('state_angle_deg', 0.0)
                        # self.door_state_angle = 0.0
                        rospy.loginfo(
                            f"[FSM] Loaded state angle: {self.door_state_angle:.2f} deg")
                        self.set_new_touch_scene(self.door_state_angle)
                        self._update_model_from_touch()
                        self.save_cabinet_info_to_csv(door_index, self.door_state_angle)
                        self.state = FSMState.PLAN
                        continue
                    except Exception as e:
                        rospy.logwarn(
                            f"[FSM] Failed to load state detection: {e}. Falling back to prompt.")

                while not rospy.is_shutdown():
                    key = input(
                        "[FSM] Door state: 'd' auto-detect | "
                        "'m' manual angle | 's' skip (use 0.0): "
                    ).strip().lower()

                    if key == 'd':
                        try:
                            rospy.wait_for_service('detect_door_state', timeout=5.0)
                            detect_door_state_srv = rospy.ServiceProxy(
                                'detect_door_state', DetectDoorState)

                            # Move robot back to detection pose
                            current_joint_values = self.robot.get_current_joint_values()
                            traj_ = np.vstack((
                                np.array(current_joint_values),
                                np.array(self.joint_values_detection)
                            ))
                            self.executor.execute_simple(traj_, max_velocity=0.5, max_acceleration=0.5)

                            resp = detect_door_state_srv(
                                base_dir=door_detection_save_dir,
                                state_detection_dir=self.state_detection_dir,
                                T_Cdetected_Cstate=np.eye(4).reshape(-1).tolist()
                            )

                            if resp.success:
                                self.door_state_angle = resp.state_angle_deg
                                rospy.loginfo(
                                    f"[FSM] Door state detected: {self.door_state_angle:.2f} deg")
                                break
                            else:
                                rospy.logwarn(
                                    f"[FSM] State detection failed: {resp.message}. Try again.")

                        except rospy.ROSException:
                            rospy.logerr("[FSM] 'detect_door_state' service not available!")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"[FSM] State detection service call failed: {e}")

                    elif key == 'm':
                        try:
                            val = input(
                                "[FSM] Enter door angle: "
                            ).strip()
                            self.door_state_angle = float(val)
                            rospy.loginfo(
                                f"[FSM] Using manual angle: {self.door_state_angle:.2f} deg")
                            break
                        except ValueError:
                            rospy.logwarn("[FSM] Invalid number. Try again.")

                    elif key == 's':
                        self.door_state_angle = 0.0
                        rospy.loginfo("[FSM] Skipping state detection. Using 0.0 deg.")
                        break

                    else:
                        rospy.logwarn("[FSM] Invalid key. Use 'd', 'm', or 's'.")

                self.state = FSMState.PLAN
                # Initialise touch scene 
                self.set_new_touch_scene(self.door_state_angle)
                self._update_model_from_touch()
                self.save_cabinet_info_to_csv(door_index, self.door_state_angle)

            elif self.state == FSMState.PLAN:

                rospy.loginfo(f"[FSM] Planning with state_angle={self.door_state_angle:.2f} ...")
                trajectory = self.plan_path_for_door(door_index, self.T_R_W)

                if trajectory is None:
                    if replan_count >= max_replan_attempts:
                        rospy.logerr("[FSM] Max replan attempts reached. Moving to ERROR.")
                        self.state = FSMState.ERROR
                    else:
                        replan_count += 1
                        rospy.logwarn(
                            f"[FSM] Planning failed. Retry {replan_count}/{max_replan_attempts}.")
                    continue

                self._planned_trajectory = trajectory
                approach_traj, insertion_traj, opening_traj = \
                    self.segment_trajectory(trajectory)

                rospy.loginfo(
                    f"[FSM] Segmented: approach={approach_traj.shape[0]}  "
                    f"insertion={insertion_traj.shape[0]}  "
                    f"opening={opening_traj.shape[0]} waypoints.")

                self.state = FSMState.APPROACH

            elif self.state == FSMState.APPROACH:
                use_moveit = True  # Set to False to use direct joint control for approach
                # Add cabinet model to MoveIt collision scene for approach phase if using MoveIt
                if use_moveit:
                    self._add_cabinet_model_to_scene(self.door_state_angle, is_doorless=False, scale_factor=1.05)

                rospy.loginfo("[FSM] Executing approach phase...")
                result = self.executor.run_approach(
                    approach_traj,
                    force_threshold=self.force_threshold_approach,
                    max_velocity=self.approach_max_vel,
                    max_acceleration=self.approach_max_acc,
                    use_moveit=use_moveit)

                if not result.success:
                    # Debug: Add prompt if I forget to set the play button on the robot
                    forgot_ = input("[FSM] Approach failed. Did you forget to press the play button on the robot? (y/n): ").strip().lower()
                    if forgot_ == 'y':
                        rospy.loginfo("[FSM] User forgot to press play button.")
                        continue  # Loop will retry approach, giving user a chance to press play
                    else:
                        rospy.loginfo("[FSM] User remembered to press play button.")
                        
                    rospy.logwarn("[FSM] Approach failed (force exceeded).")
                    if result.had_collision:
                        rospy.loginfo(f"[FSM] Collision recorded at joints: {result.collision_joints[-1]}")
                        # Correction
                        col_joints = result.collision_joints
                        T_6_0_col = self.robot.get_fwd_kinematics_moveit(col_joints)
                        T_6_0_prev = self.robot.get_fwd_kinematics_moveit(approach_traj[0])
                        via_pts = np.stack([T_6_0_prev, T_6_0_col], axis=0)
                        self.set_touch(T_6_0_col, via_pts, TouchType.UNWANTED_TOUCH)
                        self.correct_touch_model()
                    else:
                        rospy.logwarn("[FSM] No collision detected during approach failure.")
                    if replan_count < max_replan_attempts:
                        replan_count += 1
                        self.state = FSMState.PLAN
                    else:
                        self.state = FSMState.RECORD_RESULT
                    continue

                rospy.loginfo("[FSM] Approach successful. Moving to INSERTION.")
                self.state = FSMState.INSERTION

            elif self.state == FSMState.INSERTION:

                rospy.loginfo("[FSM] Executing insertion phase...")
                result, T_6_0_ft_loss = self.executor.run_insertion(
                    insertion_traj[-1],
                    force_threshold=self.force_threshold_insertion,
                    max_velocity=self.insertion_max_vel,
                    max_acceleration=self.insertion_max_acc)

                if not result.success:
                    rospy.logwarn("[FSM] Insertion failed (force exceeded).")
                    if T_6_0_ft_loss is not None:
                        rospy.loginfo("[FSM] Force drop detected during backup.")

                        T_6_0_start = self.robot.get_fwd_kinematics_moveit(insertion_traj[0])
                        via_pts = np.stack([T_6_0_start, T_6_0_ft_loss], axis=0)
                        self.set_touch(T_6_0_ft_loss, via_pts, TouchType.UNWANTED_TOUCH)
                        self.correct_touch_model()
                    # Back up to approach start
                    self.executor.backup_to_joints(
                        approach_traj[0], force_threshold=30.0)
                    if replan_count < max_replan_attempts:
                        replan_count += 1
                        self.state = FSMState.PLAN
                    else:
                        self.state = FSMState.RECORD_RESULT
                    continue

                rospy.loginfo("[FSM] Insertion successful. Moving to OPENING.")
                self.state = FSMState.OPENING

            elif self.state == FSMState.OPENING:

                rospy.loginfo("[FSM] Executing opening phase...")
                result = self.executor.run_opening(
                    opening_traj,
                    force_threshold=self.force_threshold_opening,
                    max_velocity=self.opening_max_vel,
                    max_acceleration=self.opening_max_acc)

                if not result.success:
                    rospy.logwarn("[FSM] Opening failed (force exceeded).")

                    if result.had_tactile_loss:
                        rospy.loginfo("[FSM] Tactile contact lost - starting recovery.")

                        # 1. Remember where contact was lost
                        T_6_0_loss = self.robot.get_current_tool_pose()
                        
                        # Find the closest point in the opening trajectory to the loss pose
                        dists = np.linalg.norm(opening_traj[:, :3] - T_6_0_loss[:3, 3], axis=1)
                        closest_idx = np.argmin(dists)
                        T_6_0_prev = self.robot.get_fwd_kinematics_moveit(opening_traj[closest_idx])

                        # 2. Back up to the detection pose so the camera sees the door
                        rospy.loginfo("[FSM] Backing up to detection pose...")
                        if self.joint_values_detection is not None:
                            T_6_0_current = self.robot.get_current_tool_pose()
                            current_joints = self.robot.get_current_joint_values()
                            
                            # backup 5 cm in tool frame
                            T_6_0_backup = T_6_0_current.copy()
                            T_6_0_backup[:3, 3] += T_6_0_current[:3, :3] @ np.array([0, 0, -0.05])
                            # joint config closest to the backup pose
                            backup_joints = self.robot.get_closest_ik_solution(T_6_0_backup, current_joints)

                            backup_traj = np.vstack((
                                np.array(current_joints),
                                backup_joints,
                                np.array(self.joint_values_detection)
                            ))
                            # Plan with MoveIt
                            traj, succ = self.robot.plan_to_joint_goals2(backup_traj, max_velocity=0.5, max_acceleration=0.5)
                            if succ:
                                self.executor.execute_simple(
                                    backup_traj, max_velocity=0.5, max_acceleration=0.5)
                            else:
                                rospy.logwarn("[FSM] MoveIt failed to plan backup trajectory. Attempting direct joint interpolation.")
                                self.executor.execute_simple(
                                    backup_traj, max_velocity=0.5, max_acceleration=0.5)
                        else:
                            rospy.logwarn("[FSM] No detection joints stored — skipping backup.")

                        # 3. Re-detect door state (door has moved since initial detection)
                        rospy.loginfo("[FSM] Re-detecting door state after tactile loss...")
                        new_state_angle = None
                        try:
                            rospy.wait_for_service('detect_door_state', timeout=5.0)
                            detect_door_state_srv = rospy.ServiceProxy(
                                'detect_door_state', DetectDoorState)
                            resp = detect_door_state_srv(
                                base_dir=door_detection_save_dir,
                                state_detection_dir=self.state_detection_dir,
                                T_Cdetected_Cstate=np.eye(4).reshape(-1).tolist()
                            )
                            if resp.success:
                                new_state_angle = resp.state_angle_deg
                                rospy.loginfo(
                                    f"[FSM] Re-detected state angle: {new_state_angle:.2f} deg")
                            else:
                                rospy.logwarn(
                                    f"[FSM] State re-detection failed: {resp.message}. "
                                    "Keeping previous angle.")
                        except (rospy.ROSException, rospy.ServiceException) as e:
                            rospy.logwarn(f"[FSM] State re-detection service error: {e}. "
                                          "Keeping previous angle.")

                        if new_state_angle is not None:
                            self.door_state_angle = new_state_angle

                        # 4. Reset touch scene for the new (partially-opened) state
                        self.set_new_touch_scene(self.door_state_angle)
                        self._update_model_from_touch()

                        # 5. Record the remembered contact-loss pose as a MISS touch
                        via_pts = np.stack([T_6_0_prev, T_6_0_loss], axis=0)
                        self.set_touch(T_6_0_loss, via_pts, TouchType.MISS)

                    else: # Contact never happened so no tactile loss
                        rospy.loginfo("[FSM] No tactile loss detected. Contact never happened. Recording a MISS.")
                        T_6contact_0 = self.robot.get_fwd_kinematics_moveit(opening_traj[0])
                        T_TCPcontact_0 = T_6contact_0 @ self.T_TCP_6
                        T_D_0 = np.linalg.inv(self.robot.T_0_W) @ self.T_D_S

                        T_TCPcontact_D = np.linalg.inv(T_D_0) @ T_TCPcontact_0
                        T_TCPcontact_D[2, 3] = 0.0
                        T_TCP_realcontact_0 = T_D_0 @ T_TCPcontact_D
                        T_6realcontact_0 = T_TCP_realcontact_0 @ np.linalg.inv(self.T_TCP_6)
                        self.set_touch(T_6realcontact_0, 
                                    np.array((T_6contact_0, T_6realcontact_0)),
                                    touch_type=TouchType.MISS)

                    # Correct the model with the new touch information
                    self.correct_touch_model()

                    if replan_count < max_replan_attempts:
                        # Force exceeded but no tactile loss — simple replan
                        replan_count += 1
                        self.state = FSMState.PLAN
                        continue

                # Back up along tool Z after opening (success or exhausted replans)
                self.executor.backup_along_tool_z(distance=0.05, force_threshold=30.0)
                self.state = FSMState.RECORD_RESULT

            # -----------------------------------------------------------
            # RECORD_RESULT
            # -----------------------------------------------------------
            elif self.state == FSMState.RECORD_RESULT:
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
                print('Last door index: %s' % (
                    last_door_index if last_door_index is not None else "None"))
                key = input(
                    "\n[RUN] Enter door index (or 'r' to repeat, 'q' to quit): "
                ).strip().lower()
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
                    rospy.logwarn(
                        f"[RUN] Door index {door_index} out of range "
                        f"(0-{self.doors.shape[0]-1}).")
                    continue

            last_door_index = door_index
            rospy.loginfo(f"=== Starting experiment for door {door_index} ===")
            try:
                success = self.run_single_experiment(door_index)
            except Exception as e:
                rospy.logerr(f"[RUN] Unhandled exception: {e}")
                success = False
            rospy.loginfo(
                f"=== Door {door_index}: {'SUCCESS' if success else 'FAILED'} ===")

        rospy.loginfo("[RUN] Live experiment loop ended.")


if __name__ == '__main__':
    cfg_path = os.path.join(os.path.dirname(__file__), '../cfg/config.yaml')
    with open(cfg_path, 'r') as f:
        config = yaml.safe_load(f)

    controller = MultiContactForceController(config)
    controller.run()