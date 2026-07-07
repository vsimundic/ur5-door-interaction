#!/usr/bin/env python
"""
Standalone debug script for py_touch methods.

Replicates the load_session() logic from door_replanning_control_multi_contact_refactored.py:
loads the three CSVs (cabinets_estimation, cabinets_estimation_gt, cabinets_touches),
replays set_new_scene() and set_touch() calls in order, then runs correct().

Usage:
    python debug_touch.py [SESSION_IDX]   # SESSION_IDX = int, default = latest
"""

import os
import sys
import csv
import numpy as np
import yaml

# -- path to force_utils (RVLTool, TouchType) ----------------------------------
sys.path.append(os.path.join(os.path.dirname(__file__),
    '../../force_manipulation/src'))
from force_utils import RVLTool, TouchType

# -- transforms helper ---------------------------------------------------------
import RVLPYDDManipulator as rvlpy


# ==============================================================================
# 1.  Load config
# ==============================================================================
# cfg_path = os.path.join(os.path.dirname(__file__), '../cfg/config.yaml')
cfg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/force_manipulation/cfg/door_replanning_control_multi_contact.yaml'
with open(cfg_path, 'r') as f:
    config = yaml.safe_load(f)

# ==============================================================================
# 2.  Create RVL manipulator and grab py_touch
# ==============================================================================
rvl_manipulator = rvlpy.PYDDManipulator()
rvl_manipulator.create(config['rvl_ddmanipulator_cfg'])
py_touch = rvl_manipulator.py_touch

rvl_touch_cfg = config.get('rvl_touch_cfg', None)
if rvl_touch_cfg:
    py_touch.create(rvl_touch_cfg)
    py_touch.set_results_folder(config.get('rvl_log_dir', './results'))

# ==============================================================================
# 3.  Tool geometry
# ==============================================================================
tool_cfg             = config.get('tool', {})
tactile_sensor_depth = config.get('tactile_sensor_depth', 0.006)
_tx_E = tool_cfg.get('tx_E', 0.155 - tactile_sensor_depth)
_tz_E = tool_cfg.get('tz_E', 0.278)
_a    = tool_cfg.get('a', 0.0205)
_b    = tool_cfg.get('b', 0.032)
_c    = tool_cfg.get('c', 0.011)
_d    = tool_cfg.get('d', 0.026)
_h    = tool_cfg.get('h', 0.023)
_rotz = tool_cfg.get('rotz_angle', -np.pi * 0.25)

rvl_tool = RVLTool(_a, _b, _c, _d, _h, -_tx_E, _tz_E, rot_z_angle=_rotz, rot_y_angle=0.0)
T_TCP_6  = rvl_tool.T_TCP_6
T_tool_6 = rvl_tool.T_tool_6

# np.save(tool_cfg.get('T_tool_6_path'), T_tool_6)

py_touch.create_simple_tool(_a, _b, _c, _d, _h, T_tool_6)
print("[init] Tool geometry set.")

# ==============================================================================
# 4.  Camera intrinsics
# ==============================================================================
camera_cfg = config.get('camera', {})
py_touch.set_camera_params(
    camera_cfg.get('fu',     597.9033203125),
    camera_cfg.get('fv',     598.47998046875),
    camera_cfg.get('uc',     323.8436584472656),
    camera_cfg.get('vc',     236.32774353027344),
    camera_cfg.get('width',  640),
    camera_cfg.get('height', 480),
)
print("[init] Camera intrinsics set.")

# ==============================================================================
# 5.  Touch correction hyper-parameters
# ==============================================================================
touch_cfg = config.get('touch', {})
touch_a   = touch_cfg.get('a', config.get('static_depth', 0.4))
touch_b   = touch_cfg.get('b', 0.0)
touch_c   = touch_cfg.get('c', 0.005)

# ==============================================================================
# 6.  Camera extrinsics (T_C_6) — from calibration file
# ==============================================================================
T_C_6      = np.load(config['T_C_6_path'])
R_C_6 = T_C_6[:3, :3]
I_ = R_C_6 @ R_C_6.T
mean_trace = np.trace(I_) / 3.0
scale_factor = np.sqrt(mean_trace)
T_C_6[:3, :3] /= scale_factor
T_C_6_init = T_C_6.copy()
print(f"[init] T_C_6 loaded from {config['T_C_6_path']}")

# ==============================================================================
# 7.  CSV paths
# ==============================================================================
detection_base_dir = os.path.expanduser(
    config.get('detection_base_dir', '~/data/online_detection'))

cabinets_estimation_filename    = config.get('cabinets_estimation_filename', os.path.join(detection_base_dir, 'cabinets_estimation.csv'))
cabinets_estimation_filename_gt = config.get(
    'cabinets_estimation_filename_gt',
    os.path.join(detection_base_dir, 'cabinets_estimation_gt.csv'))
cabinets_touches_filename       = config.get(
    'cabinets_touches_filename',
    os.path.join(detection_base_dir, 'cabinets_touches.csv'))

door_thickness = config.get('door_thickness', 0.018)


# ==============================================================================
# 8.  Forward kinematics stub (no live robot)
# ==============================================================================
def fwd_kinematics(joint_values) -> np.ndarray:
    q = np.array(joint_values, dtype=np.float64)
    return rvl_manipulator.fwd_kinematics(q).astype(np.float64)


# ==============================================================================
# 9.  CSV helpers — mirrored from load_session() in the main node
# ==============================================================================
def compute_column_indices(header):
    """Map CSV column names to flat integer indices, expanding matrix columns.

    Matrix/vector columns in the header (R_A_C etc.) each occupy multiple
    consecutive data columns; this function accounts for that so callers can
    use symbolic names instead of hard-coded offsets.
    """
    col_idx = {}
    idx = 0
    for name in header:
        if name in {"R_A_C", "R_C_E", "R_E_0", "R_A_S", "R_Ek_E"}:
            col_idx[name] = idx
            idx += 9
        elif name in {"t_A_C", "t_C_E", "t_E_0", "t_A_S", "t_Ek_E", "V"}:
            col_idx[name] = idx
            idx += 3
        else:
            col_idx[name] = idx
            idx += 1
    return col_idx


def get_matrix(base, size, row_, col_idx_):
    start = col_idx_[base]
    return np.array(row_[start:start + size], dtype=np.float64)


# ==============================================================================
# 10.  update_model_from_touch — mirrored from the main node (simplified)
# ==============================================================================
def update_model_from_touch():
    rvl_manipulator.update_model_x()
    T_Arot_6_corr = rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
    T_C_6_corr    = rvl_manipulator.get_corrected_camera_pose().astype(np.float64)
    if np.any(np.isnan(T_Arot_6_corr)) or np.any(np.isnan(T_C_6_corr)):
        print("[warn] Corrected poses contain NaN — model update skipped.")
        return
    print("[update] Model updated from touch.")


# ==============================================================================
# 11.  run_correction — mirrors correct_touch_model in the main node
# ==============================================================================
def run_correction():
    print("[correct] Running py_touch.correct()...")
    py_touch.correct()

    rvl_manipulator.update_model_x()
    T_Arot_6_corr = rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
    T_C_6_corr    = rvl_manipulator.get_corrected_camera_pose().astype(np.float64)

    print("[correct] Corrected cabinet pose (T_Arot_6):\n", T_Arot_6_corr)
    print("[correct] Corrected camera pose  (T_C_6):\n",    T_C_6_corr)
    return T_Arot_6_corr, T_C_6_corr


# ==============================================================================
# 12.  load_session — mirrored from load_session() in the main node
# ==============================================================================
def load_session(session_idx=None, correct_on_touch=False):
    """Replay a session from the three CSVs.

    Reads cabinets_estimation CSV, cabinets_estimation_gt CSV, and
    cabinets_touches CSV (paths come from config / module-level variables).
    For each scene row in the chosen session it calls py_touch.set_new_scene()
    then replays all recorded non-wanted touches via py_touch.set_touch().

    Args:
        session_idx:      Integer session to load; None = latest session.
        correct_on_touch: Call run_correction() after each replayed touch,
                          mirroring CORRECT_ON_TOUCH in the main node.

    Returns:
        The session index that was loaded.
    """
    if not os.path.exists(cabinets_estimation_filename):
        raise RuntimeError(
            f"Estimation CSV not found: {cabinets_estimation_filename}")

    # --- estimation CSV -------------------------------------------------------
    with open(cabinets_estimation_filename, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        col_idx = compute_column_indices(header)
        rows = list(reader)

    if not rows:
        raise RuntimeError("Estimation CSV is empty — run the main node first.")

    last_sess_idx = int(rows[-1][col_idx['session_idx']])
    if session_idx is None:
        session_idx = last_sess_idx
    print(f"[load_session] Loading session {session_idx} "
          f"(latest available: {last_sess_idx})")

    session_rows = [r for r in rows
                    if int(r[col_idx['session_idx']]) == session_idx]
    if not session_rows:
        raise RuntimeError(f"No rows found for session {session_idx}.")

    # --- GT CSV ---------------------------------------------------------------
    gt_rows    = []
    col_idx_gt = {}
    if os.path.exists(cabinets_estimation_filename_gt):
        with open(cabinets_estimation_filename_gt, 'r') as f_gt:
            reader_gt  = csv.reader(f_gt)
            header_gt  = next(reader_gt)
            col_idx_gt = compute_column_indices(header_gt)
            gt_rows    = list(reader_gt)
    else:
        print(f"[warn] GT CSV not found: {cabinets_estimation_filename_gt}")

    # --- touches CSV ----------------------------------------------------------
    touch_rows        = []
    col_idx_touches   = {}
    if os.path.exists(cabinets_touches_filename):
        with open(cabinets_touches_filename, 'r') as f_t:
            reader_t       = csv.reader(f_t)
            header_t       = next(reader_t)
            col_idx_touches = compute_column_indices(header_t)
            touch_rows     = list(reader_t)
    else:
        print(f"[warn] Touches CSV not found: {cabinets_touches_filename}")

    # --- replay ---------------------------------------------------------------
    for row in session_rows:
        # scene_idx column in the header maps to idx_to_save in the data
        # (the header naming in the main node is inverted vs the save order).
        scene_idx   = int(row[col_idx['scene_idx']])

        sx          = float(row[col_idx['sx']])   # door_thickness
        sy          = float(row[col_idx['sy']])   # width
        sz          = float(row[col_idx['sz']])   # height
        rx          = float(row[col_idx['rx']])
        ry          = float(row[col_idx['ry']])
        state_angle = float(row[col_idx['state_angle']])

        R_A_C = get_matrix('R_A_C', 9, row, col_idx).reshape(3, 3)
        t_A_C = get_matrix('t_A_C', 3, row, col_idx)
        R_C_E = get_matrix('R_C_E', 9, row, col_idx).reshape(3, 3)
        t_C_E = get_matrix('t_C_E', 3, row, col_idx)
        R_E_0 = get_matrix('R_E_0', 9, row, col_idx).reshape(3, 3)
        t_E_0 = get_matrix('t_E_0', 3, row, col_idx)

        T_A_C_init = np.eye(4)
        T_A_C_init[:3, :3] = R_A_C
        T_A_C_init[:3, 3]  = t_A_C

        T_C_E_mat = np.eye(4)
        T_C_E_mat[:3, :3] = R_C_E
        T_C_E_mat[:3, 3]  = t_C_E

        T_E_0_capture = np.eye(4)
        T_E_0_capture[:3, :3] = R_E_0
        T_E_0_capture[:3, 3]  = t_E_0

        # GT defaults (fall back to estimated values if GT row is missing)
        sx_gt = sx; sy_gt = sy; sz_gt = sz
        rx_gt = rx; ry_gt = ry
        state_angle_gt = state_angle
        T_Arot_W_gt = np.eye(4)

        for row_gt in gt_rows:
            if int(row_gt[col_idx_gt['session_idx']]) != session_idx:
                continue
            if int(row_gt[col_idx_gt['scene_idx']]) != scene_idx:
                continue

            sx_gt          = float(row_gt[col_idx_gt['sx']])
            sy_gt          = float(row_gt[col_idx_gt['sy']])
            sz_gt          = float(row_gt[col_idx_gt['sz']])
            rx_gt          = float(row_gt[col_idx_gt['rx']])
            ry_gt          = float(row_gt[col_idx_gt['ry']])
            state_angle_gt = float(row_gt[col_idx_gt['state_angle']])

            R_Arot_W_gt = get_matrix('R_A_S', 9, row_gt, col_idx_gt).reshape(3, 3)
            t_Arot_W_gt = get_matrix('t_A_S', 3, row_gt, col_idx_gt)

            T_Arot_W_gt = np.eye(4)
            T_Arot_W_gt[:3, :3] = R_Arot_W_gt
            T_Arot_W_gt[:3, 3]  = t_Arot_W_gt
            break

        print(f"\n[load_session] Scene {scene_idx}: "
              f"W={sy:.4f}  H={sz:.4f}  state_angle={state_angle:.2f}")

        py_touch.set_new_scene(
            sx,  sy,  sz,  rx,  ry,
            touch_a, touch_b, touch_c, state_angle,
            T_C_E_mat, T_A_C_init, T_E_0_capture,
            sx_gt, sy_gt, sz_gt, rx_gt, ry_gt,
            state_angle_gt, T_Arot_W_gt)

        update_model_from_touch()

        # replay touches for this scene
        n_touches = 0
        for row_touch in touch_rows:
            if int(row_touch[col_idx_touches['session_idx']]) != session_idx:
                continue
            if int(row_touch[col_idx_touches['scene_idx']]) != scene_idx:
                continue

            type_touch = TouchType(int(row_touch[col_idx_touches['type']]))
            if type_touch == TouchType.WANTED_TOUCH:
                continue  # skip wanted touches, same as main node

            R_Ek_E = get_matrix('R_Ek_E', 9, row_touch, col_idx_touches).reshape(3, 3)
            t_Ek_E = get_matrix('t_Ek_E', 3, row_touch, col_idx_touches)
            V      = get_matrix('V',      3, row_touch, col_idx_touches)
            t      = float(row_touch[col_idx_touches['t']])
            state_angle_touch = float(row_touch[col_idx_touches['state_angle']])
            b_miss = (type_touch == TouchType.MISS)

            T_Ek_E_mat = np.eye(4)
            T_Ek_E_mat[:3, :3] = R_Ek_E
            T_Ek_E_mat[:3, 3]  = t_Ek_E

            print(f"  [touch] type={type_touch.name:<15} miss={b_miss}  "
                  f"t={t:.4f}  state_angle={state_angle_touch:.2f}")
            py_touch.set_touch(T_Ek_E_mat, V, t, b_miss)
            n_touches += 1

            if correct_on_touch:
                run_correction()

        print(f"  -> {n_touches} touch(es) replayed for scene {scene_idx}.")

    return session_idx


# ==============================================================================
# 13.  Main
# ==============================================================================
if __name__ == '__main__':
    # --------------------------------------------------------------------------
    # Set SESSION_IDX to an integer to pick a specific session,
    # or leave as None to use the latest session.
    # Set CORRECT_ON_TOUCH to True to mirror CORRECT_ON_TOUCH in the main node.
    # --------------------------------------------------------------------------
    SESSION_IDX      = None   # None = latest
    CORRECT_ON_TOUCH = True
    # --------------------------------------------------------------------------

    if len(sys.argv) > 1:
        SESSION_IDX = int(sys.argv[1])

    session_idx = load_session(session_idx=SESSION_IDX,
                               correct_on_touch=CORRECT_ON_TOUCH)

    print(f"\n[done] Session {session_idx} fully replayed.")
    if not CORRECT_ON_TOUCH:
        print("[done] Running final correction...")
        run_correction()

    print("\n=== Done ===")
