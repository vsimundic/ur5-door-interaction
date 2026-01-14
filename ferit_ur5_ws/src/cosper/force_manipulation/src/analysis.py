#!/usr/bin/python

import os
import numpy as np
from core.util import chamfer_distance
from push_force_trajectories import *
from force_utils import *
from gazebo_push_open.cabinet_model2 import Cabinet2
import RVLPYDDManipulator as rvlpy
import matplotlib.pyplot as plt
import csv
import yaml

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


with open(os.path.join(os.path.dirname(__file__), '../cfg/door_replanning_control.yaml'), 'r') as f:
    config = yaml.safe_load(f) # type: dict
    
is_offline = config.get("is_offline", False)
mode = 'offline' if is_offline else 'online'

correct_on_touch = config.get("correct_on_touch", False)

rvl_ddmanipulator_cfg = config.get("rvl_ddmanipulator_cfg", None)
rvl_touch_cfg = config.get("rvl_touch_cfg", None)

rvl_manipulator = rvlpy.PYDDManipulator()
rvl_manipulator.create(rvl_ddmanipulator_cfg)

# Touch initialization
py_touch = rvl_manipulator.py_touch
py_touch.create(rvl_touch_cfg)

touch_cfg = config.get("touch", None)
touch_a = touch_cfg.get("a", 0.0)
touch_b = touch_cfg.get("b", 0.0)
touch_c = touch_cfg.get("c", 0.0)

tool_cfg = config.get("tool", None)
tx_E = tool_cfg.get("tx_E", 0.155 - 0.005)
tz_E = tool_cfg.get("tz_E", 0.278)
a_tool = tool_cfg.get("a", 0.0205)
b_tool = tool_cfg.get("b", 0.032)
c_tool = tool_cfg.get("c", 0.011)
d_tool = tool_cfg.get("d", 0.026)
h_tool = tool_cfg.get("h", 0.023)
rotz_angle = tool_cfg.get("rotz_angle", -np.pi * 0.25) # to get the tcp on the one finger instead of two
rvl_tool = RVLTool(a_tool, b_tool, c_tool, d_tool, h_tool, -tx_E, tz_E, rot_z_angle=rotz_angle)
T_tool_6 = rvl_tool.T_tool_6 #type: np.ndarray
T_TCP_6 = rvl_tool.T_TCP_6 #type: np.ndarray
py_touch.create_simple_tool(a_tool, b_tool, c_tool, d_tool, h_tool, T_tool_6)


cabinet_estimation_filename = config.get("online_cabinet_filename" if not is_offline else "offline_cabinet_filename", None)
cabinet_gt_filename = config.get("online_cabinet_filename_gt" if not is_offline else "offline_cabinet_filename_gt", None)
touches_filename = config.get("online_touches_filename" if not is_offline else "offline_touches_filename", None)
static_depth = config.get("static_depth", 0.0)

analysis_dir = config.get("analysis_dir", None)
if not os.path.exists(analysis_dir):
    os.makedirs(analysis_dir)

chamfer_dists = []
D_dists = []
D_dists_scene = []
Dgt_nums = []

py_touch.set_visualization(False)

i_attempt = 0
i_attempt_per_scene = 0
num_attempts_per_scene = []

with open(cabinet_estimation_filename, 'r') as f:
    reader = csv.reader(f)

    header = next(reader)

    # col_idx = {col: idx for idx, col in enumerate(header)}
    col_idx = compute_column_indices(header)

    rows = list(reader)
    if not rows:
        raise RuntimeError("No session found, please run the script to create a session.")

    session_idx = int(rows[-1][col_idx['session_idx']])

    for idx, row in enumerate(rows):
        sess_idx = int(row[col_idx['session_idx']])
        if sess_idx != session_idx: # load only last session
            continue

        scene_idx = int(row[col_idx['scene_idx']])
        load_idx = int(row[col_idx['load_idx']])

        sx = float(row[col_idx['sx']])
        sy = float(row[col_idx['sy']])
        sz = float(row[col_idx['sz']])

        rx = float(row[col_idx['rx']])
        ry = float(row[col_idx['ry']])

        state_angle = float(row[col_idx['state_angle']])

        R_A_C = get_matrix('R_A_C', 9, row, col_idx).reshape(3, 3)
        t_A_C = get_matrix('t_A_C', 3, row, col_idx)
        R_C_E = get_matrix('R_C_E', 9, row, col_idx).reshape(3, 3)
        t_C_E = get_matrix('t_C_E', 3, row, col_idx)
        R_E_0_capture = get_matrix('R_E_0', 9, row, col_idx).reshape(3, 3)
        t_E_0_capture = get_matrix('t_E_0', 3, row, col_idx)

        T_A_C = np.eye(4)
        T_A_C[:3, :3] = R_A_C
        T_A_C[:3, 3] = t_A_C

        T_A_C_init = T_A_C.copy()

        T_C_E = np.eye(4)
        T_C_E[:3, :3] = R_C_E
        T_C_E[:3, 3] = t_C_E
        T_C_E_init = T_C_E.copy()

        T_E_0_capture = np.eye(4)
        T_E_0_capture[:3, :3] = R_E_0_capture
        T_E_0_capture[:3, 3] = t_E_0_capture

        T_A_W = T_E_0_capture @ T_C_E @ T_A_C
        cabinet_model = Cabinet2(s=np.array([sx, sy, sz, static_depth]),
                                r=np.array([rx, ry]),
                                axis_pos=-1,
                                T_A_W=T_A_W,
                                save_path=None,
                                has_handle=False)
        cabinet_model.change_door_angle(state_angle)
        
        # Load GT cabinet model
        with open(cabinet_gt_filename, 'r') as f_gt:

            reader_gt = csv.reader(f_gt)

            header_gt = next(reader_gt)
            col_idx_gt = compute_column_indices(header_gt)

            rows_gt = list(reader_gt)

            cabinet_gt = None
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
                state_angle_gt = float(row_gt[col_idx_gt['state_angle']])

                R_Arot_W_gt = get_matrix('R_A_S', 9, row_gt, col_idx_gt).reshape(3, 3)
                t_Arot_W_gt = get_matrix('t_A_S', 3, row_gt, col_idx_gt)

                T_Arot_W_gt = np.eye(4)
                T_Arot_W_gt[:3, :3] = R_Arot_W_gt
                T_Arot_W_gt[:3, 3] = t_Arot_W_gt

                T_Arot_A_gt = np.eye(4)
                T_Arot_A_gt[:3, :3] = rot_z(np.deg2rad(state_angle_gt))
                T_A_W_gt = T_Arot_W_gt @ np.linalg.inv(T_Arot_A_gt)

                cabinet_gt = Cabinet2(s=np.array([sx_gt, sy_gt, sz_gt, static_depth]),
                                        r=np.array([rx_gt, ry_gt]),
                                        axis_pos=-1,
                                        T_A_W=T_A_W_gt,
                                        save_path=None,
                                        has_handle=False)
                cabinet_gt.change_door_angle(state_angle_gt)
                # cabinet_gt.change_door_angle(-5.9)

        # Visualization
        T_O_W = cabinet_gt.T_A_W @ np.linalg.inv(cabinet_gt.T_A_O)
        cabinet_mesh_gt = cabinet_gt.create_mesh(has_drawer=False)
        cabinet_mesh_gt.transform(T_O_W)
        cabinet_mesh_gt.compute_vertex_normals()
        cabinet_mesh_gt.paint_uniform_color([0.3, 0.3, 0.3])
        D_mesh_gt = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.03)
        T_D_0_gt = cabinet_gt.T_A_W @ cabinet_gt.T_D_A
        D_mesh_gt.transform(T_D_0_gt)

        # o3d.visualization.draw_geometries([cabinet_mesh_gt, D_mesh_gt], window_name='GT Cabinet Model')

        t_TCP_D_e = generate_tool_line_positions(cabinet_model.sz, z_offset=0.007)
        t_TCP_D_e_gt = generate_tool_line_positions(cabinet_gt.sz, z_offset=0.0)
        T_D_E_gt = np.linalg.inv(T_E_0_capture) @ cabinet_gt.T_A_W @ cabinet_gt.T_D_A
        t_TCP_E_gt = t_TCP_D_e_gt @ T_D_E_gt[:3, :3].T + T_D_E_gt[:3, 3]

        # if scene_idx > 7:
        py_touch.set_visualization(False)
        py_touch.set_new_scene(sx, sy, sz, rx, ry, touch_a, touch_b, touch_c, state_angle, T_C_E, T_A_C_init, T_E_0_capture,
                                    sx_gt, sy_gt, sz_gt, rx_gt, ry_gt, state_angle_gt, T_Arot_W_gt)
        py_touch.set_visualization(False)
        i_attempt_per_scene = 0
        
        # Update model x to py_touch
        rvl_manipulator.update_model_x()
        T_Arot_6_corrected = rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
        T_C_6_corrected = rvl_manipulator.get_corrected_camera_pose().astype(np.float64)
        # Update the poses
        T_Arot_A = np.eye(4)
        T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
        T_C_6 = T_C_6_corrected.copy()
        T_A_C = np.linalg.inv(T_C_6) @ T_Arot_6_corrected @ np.linalg.inv(T_Arot_A)
        T_D_Arot = rvl_manipulator.get_corrected_pose_D_Arot().astype(np.float64)
        cabinet_model.T_D_Arot = T_D_Arot.copy()
        # Update the cabinet model
        cabinet_model.T_A_W = T_E_0_capture @ T_C_6 @ T_A_C
        cabinet_model.change_door_angle(state_angle)

        T_D_E = np.linalg.inv(T_E_0_capture) @ cabinet_model.T_A_W @ cabinet_model.T_D_A
        D_dists_scene.append(np.linalg.norm(T_D_E[:3, 3] - T_D_E_gt[:3, 3]))


        with open(touches_filename, 'r') as f_touches:
            reader_touches = csv.reader(f_touches)
            header_touches = next(reader_touches)
            col_idx_touches = compute_column_indices(header_touches)

            rows_touches = list(reader_touches)
            if not rows_touches:
                continue
            # filter only touches for this session and scene
            rows_touches = [row for row in rows_touches if int(row[col_idx_touches['session_idx']]) == sess_idx and int(row[col_idx_touches['scene_idx']]) == scene_idx]
            num_attempts_per_scene.append(len(rows_touches))
            
            for row_touch in rows_touches:
                touch_sess_idx = int(row_touch[col_idx_touches['session_idx']])
                if touch_sess_idx != sess_idx:
                    continue
                touch_scene_idx = int(row_touch[col_idx_touches['scene_idx']])
                if touch_scene_idx != scene_idx:
                    continue

                i_attempt += 1
                i_attempt_per_scene += 1
                T_D_E = np.linalg.inv(T_E_0_capture) @ cabinet_model.T_A_W @ cabinet_model.T_D_A
                T_D_0 = cabinet_model.T_A_W @ cabinet_model.T_D_A
                T_De_Dgt = np.linalg.inv(T_D_0_gt) @ T_D_0
                t_TCP_Dgt = t_TCP_D_e @ T_De_Dgt[:3, :3].T + T_De_Dgt[:3, 3]
                t_TCP_0 = t_TCP_D_e @ T_D_0[:3, :3].T + T_D_0[:3, 3] # visualization purposes
                mask_indices = np.where(
                    (t_TCP_Dgt[:, 0] < 0.002) &
                    (t_TCP_Dgt[:, 1] > -0.002) &
                    (t_TCP_Dgt[:, 1] < cabinet_gt.sz) &
                    (t_TCP_Dgt[:, 2] > -0.002))[0]
                mask = t_TCP_Dgt[mask_indices]
                Dgt_nums.append(mask.shape[0]/len(t_TCP_Dgt))
                t_TCP_E = t_TCP_D_e @ T_D_E[:3, :3].T + T_D_E[:3, 3]
                d = chamfer_distance(t_TCP_E, t_TCP_E_gt)
                chamfer_dists.append(d)
                D_dists.append(np.linalg.norm(T_D_E[:3, 3] - T_D_E_gt[:3, 3]))

                # Visualization
                geoms = []
                T_O_W = cabinet_model.T_A_W @ np.linalg.inv(cabinet_model.T_A_O)
                cabinet_mesh = cabinet_model.create_mesh(has_drawer=False)
                cabinet_mesh.transform(T_O_W)
                cabinet_mesh.compute_vertex_normals()
                cabinet_mesh.paint_uniform_color([0.3, 0.7, 0.3])
                D_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.03)
                D_mesh.transform(T_D_0)
                geoms.append(cabinet_mesh)
                geoms.append(D_mesh)
                geoms.append(cabinet_mesh_gt)
                geoms.append(D_mesh_gt)

                for i_ in range(t_TCP_0.shape[0]):
                    t_ = t_TCP_0[i_]
                    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.002)
                    sphere.translate(t_)
                    sphere.paint_uniform_color([0.8, 0.2, 0.2])
                    geoms.append(sphere)

                if mask.shape[0] < 1:
                    py_touch.set_visualization(False)
                    # o3d.visualization.draw_geometries(geoms, window_name='Cabinet Model')

                type_touch = TouchType(int(row_touch[col_idx_touches['type']]))
                # if type_touch == TouchType.MISS:
                    # continue  # Skip wanted touches for now

                R_Ek_E = get_matrix('R_Ek_E', 9, row_touch, col_idx_touches).reshape(3, 3)
                t_Ek_E = get_matrix('t_Ek_E', 3, row_touch, col_idx_touches)
                V = get_matrix('V', 3, row_touch, col_idx_touches)
                t = float(row_touch[col_idx_touches['t']])
                state_angle_touch = float(row_touch[col_idx_touches['state_angle']])
                b_miss = True if type_touch == TouchType.MISS else False

                T_Ek_E = np.eye(4)
                T_Ek_E[:3, :3] = R_Ek_E
                T_Ek_E[:3, 3] = t_Ek_E

                # if True:
                if type_touch != TouchType.WANTED_TOUCH:
                    print(f"Attempt {i_attempt}, type: {type_touch})")
                    py_touch.set_touch(T_Ek_E, V, t, b_miss)
                else:
                    print(f"Attempt {i_attempt}, type: {type_touch})") 

                if correct_on_touch:
                    T_Arot_A = np.eye(4)
                    T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
                    
                    if type_touch != TouchType.WANTED_TOUCH:
                        py_touch.correct()

                    # Update model x to py_touch
                    rvl_manipulator.update_model_x()

                    T_Arot_6_corrected = rvl_manipulator.get_corrected_cabinet_pose().astype(np.float64)
                    T_C_6_corrected = rvl_manipulator.get_corrected_camera_pose().astype(np.float64)

                    # Update the poses
                    T_C_6 = T_C_6_corrected.copy()
                    T_A_C = np.linalg.inv(T_C_6) @ T_Arot_6_corrected @ np.linalg.inv(T_Arot_A)

                    T_D_Arot = rvl_manipulator.get_corrected_pose_D_Arot().astype(np.float64)
                    cabinet_model.T_D_Arot = T_D_Arot.copy()

                    # Update the cabinet model
                    cabinet_model.T_A_W = T_E_0_capture @ T_C_6 @ T_A_C

                    cabinet_model.change_door_angle(state_angle)
                
# Calculate Chamfer distance after each touch
T_D_E = np.linalg.inv(T_E_0_capture) @ cabinet_model.T_A_W @ cabinet_model.T_D_A
T_D_0 = cabinet_model.T_A_W @ cabinet_model.T_D_A
T_De_Dgt = np.linalg.inv(T_D_0_gt) @ T_D_0
t_TCP_Dgt = t_TCP_D_e @ T_De_Dgt[:3, :3].T + T_De_Dgt[:3, 3]
mask_indices = np.where(
    (t_TCP_Dgt[:, 0] < 0.002) &
    (t_TCP_Dgt[:, 1] > -0.002) &
    (t_TCP_Dgt[:, 1] < cabinet_gt.sz) &
    (t_TCP_Dgt[:, 2] > -0.002))[0]
mask = t_TCP_Dgt[mask_indices]
Dgt_nums.append(mask.shape[0]/len(t_TCP_Dgt))
chamfer_dists.append(chamfer_distance(t_TCP_E, t_TCP_E_gt))
D_dists.append(np.linalg.norm(T_D_E[:3, 3] - T_D_E_gt[:3, 3]))

# Plot the Chamfer distances
plt.plot(chamfer_dists)
plt.xlabel('Attempt')
plt.ylabel('Chamfer Distance [m]')
plt.title('Chamfer Distance between estimated and GT tool positions')
plt.savefig(os.path.join(analysis_dir, '{}_chamfer_distances.png'.format(mode)))
plt.show()

# Plot the D distances
plt.plot(D_dists)
plt.xlabel('Attempt')
plt.ylabel('D Distance [m]')
plt.title('Distance between estimated and GT tool positions')
plt.savefig(os.path.join(analysis_dir, '{}_D_distances.png'.format(mode)))
plt.show()

# Plot the number of points in the GT tool position
plt.plot(Dgt_nums)
plt.xlabel('Attempt')
plt.ylabel('Number of tool points behind GT door surface')
plt.title('Number of tool points in GT Tool Position')
plt.savefig(os.path.join(analysis_dir, '{}_Dgt_nums.png'.format(mode)))
plt.show()

# Plot the number of attempts per scene
plt.plot(num_attempts_per_scene)
plt.xlabel('Scene')
plt.ylabel('Number of Actions')
plt.title('Number of Actions per Scene')
plt.savefig(os.path.join(analysis_dir, '{}_num_attempts_per_scene.png'.format(mode)))
plt.show()

# Save the results to a CSV file
output_filename = os.path.join(analysis_dir, '{}_analysis_results.csv'.format(mode))
with open(output_filename, 'w', newline='') as f_out:
    writer = csv.writer(f_out)
    writer.writerow(['attempt','chamfer_dist','D_dist','Dgt_percent'])
    for i, (chamfer_dist, D_dist, Dgt_num) in enumerate(zip(chamfer_dists, D_dists, Dgt_nums)):
        writer.writerow([i + 1, chamfer_dist, D_dist, Dgt_num])

output_filename_per_scene = os.path.join(analysis_dir, '{}_analysis_results_per_scene.csv'.format(mode))
with open(output_filename_per_scene, 'w', newline='') as f_out:
    writer = csv.writer(f_out)
    writer.writerow(['Scene', 'num_attempts', 'D_dist'])
    for i, num_attempts in enumerate(num_attempts_per_scene):
        writer.writerow([i, num_attempts, D_dists_scene[i]])