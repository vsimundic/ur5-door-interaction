#!/usr/bin/env python3
import os
import yaml
import numpy as np
import copy
import rospy

from tqdm import tqdm
from core.transforms import *
# from gazebo_push_open.cabinet_model import Cabinet
from gazebo_push_open.cabinet_model2 import Cabinet2
from core.real_ur5_controller import UR5Controller
from push_force_trajectories import *
from force_utils import *
import open3d as o3d
import RVLPYDDManipulator as rvlpy

np.random.seed(12345)


def rotz_multiple(theta_arr):
    num_angles = theta_arr.shape[0]
    Rz = np.zeros((num_angles, 3, 3))
    c = np.cos(theta_arr)
    s = np.sin(theta_arr)
    Rz[:, 0, 0] = c
    Rz[:, 0, 1] = -s
    Rz[:, 1, 0] = s
    Rz[:, 1, 1] = c
    Rz[:, 2, 2] = 1
    return Rz


def line_seg_to_circle_dist_all(cir, p1, p2s):
    """Distances from a point to multiple line segments p1->p2s."""
    p1 = np.array(p1, dtype=float)
    cir = np.array(cir, dtype=float)
    p2s = np.array(p2s, dtype=float)  # shape: (N, 2)

    seg = p2s - p1                  # (N, 2)
    seg_len_sq = np.sum(seg**2, axis=1)
    cir_vec = cir - p1              # (2,)
    dot_val = np.sum(seg * cir_vec, axis=1)
    t = np.where(seg_len_sq > 0, np.clip(dot_val / seg_len_sq, 0, 1), 0)
    proj_pts = p1 + t[:, None] * seg
    dist = np.linalg.norm(proj_pts - cir, axis=1)
    return dist


def load_config():
    cfg_path = os.path.join(os.path.dirname(__file__), '../cfg/door_replanning_control.yaml')
    with open(cfg_path, 'r') as f:
        return yaml.safe_load(f)


def main():
    rospy.init_node('generate_simulation_poses', anonymous=True)
    cfg = load_config() # type: dict

    # === High-level paths/configs ===
    base_dir = cfg.get('base_dir')                          # required by your pipeline
    analysis_dir = cfg.get('analysis_dir', os.path.join(base_dir, 'analysis'))
    os.makedirs(analysis_dir, exist_ok=True)
    simulation_dir = cfg.get('simulation_dir', os.path.join(base_dir, 'simulation'))
    os.makedirs(simulation_dir, exist_ok=True)

    # Files & resources
    rvl_cfg = cfg.get('rvl_ddmanipulator_cfg')
    gripper_mesh_path = cfg.get('gripper_mesh_path')

    # Cabinet mesh output locations (derived)
    cabinet_meshes_dir = os.path.join(simulation_dir, 'cabinet_meshes')
    os.makedirs(cabinet_meshes_dir, exist_ok=True)
    cabinet_static_mesh_path = os.path.join(cabinet_meshes_dir, 'cabinet_static.ply')
    cabinet_panel_mesh_path  = os.path.join(cabinet_meshes_dir, 'cabinet_panel.ply')
    cabinet_mesh_path = os.path.join(cabinet_meshes_dir, 'cabinet_mesh.ply')

    # Optional precomputed collision-free pose file (use if it exists)
    t_g_dd_colfree_path = cfg.get('T_G_DD_all_colfree_path_simulation')

    # === Physical parameters from config ===
    door_thickness = float(cfg.get('door_thickness', 0.018))
    static_depth   = float(cfg.get('static_depth', 0.4))
    min_width      = float(cfg.get('min_width_simulation', 0.2))
    max_width      = float(cfg.get('max_width_simulation', 0.6))
    min_height     = float(cfg.get('min_height_simulation', 0.2))
    max_height     = float(cfg.get('max_height_simulation', 0.8))
    push_latch_mechanism_length = float(cfg.get('push_latch_mechanism_length', 0.046))
    latch_offset = float(cfg.get('latch_offset', 0.01))

    # Tool (TCP) params
    tool_cfg = cfg.get('tool', {}) # type: dict
    grasp_one_finger = bool(cfg.get('grasp_one_finger', True))
    tx_G = float(tool_cfg.get('tx_G', 0.0721))
    tz_G = float(tool_cfg.get('tz_G_one_finger' if grasp_one_finger else 'tz_G_two_fingers',
                              0.1041 if grasp_one_finger else 0.100))
    R_TCP_D = np.array(tool_cfg.get('R_TCP_D', [[0, 0, -1],
                                                [0, -1, 0],
                                                [-1, 0, 0]]), dtype=float)

    # Camera params
    T_C_6_path = cfg.get("T_C_6_path", None)
    T_C_6 = np.load(T_C_6_path) if T_C_6_path else None

    T_6_0_capture = np.eye(4)

    # === Sampling & generation settings (kept as constants; tweak here or add to YAML if you prefer) ===
    n = 100000
    n_saved = 1000
    LOAD_PARAMS = False
    VISUALIZE = False
    SAVE_CABINETS = True

    # Output CSV
    cabinet_configs_path = os.path.join(simulation_dir, 'simulation_cabinet_configs2.csv')
    if SAVE_CABINETS:
        with open(cabinet_configs_path, 'w') as f:
            f.write('idx,sx,sy,sz,rx,ry,state_angle,t_A_0,z_rot,R_E_0_contact,t_E_0_contact\n')

    # === Random dimensions ===
    if not LOAD_PARAMS:
        widths  = np.random.uniform(min_width, max_width, size=(n,))
        heights = np.random.uniform(min_height, max_height, size=(n,))
        np.save(os.path.join(simulation_dir, 'simulation_cabinet_widths.npy'), widths)
        np.save(os.path.join(simulation_dir, 'simulation_cabinet_heights.npy'), heights)
    else:
        widths  = np.load(os.path.join(simulation_dir, 'simulation_cabinet_widths.npy'))
        heights = np.load(os.path.join(simulation_dir, 'simulation_cabinet_heights.npy'))

    axis_pos = -1
    state_angles = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length / (widths - latch_offset)))
    opening_angle = float(cfg.get('opening_angle_simulation', -15.0))
    opening_step_deg = float(cfg.get('opening_step_simulation', 5.0))
    # Dist constraints
    base_size = close_dist_range = 0.3
    far_dist_range = 0.9

    # Door world orientation base
    T_A_W = np.eye(4)
    T_A_W[:3, :3] = np.array([[0, -1, 0],
                              [1,  0, 0],
                              [0,  0, 1]], dtype=float)

    # === Random cabinet world positions ===
    min_x = cfg.get('min_x_simulation', -0.8)
    max_x = cfg.get('max_x_simulation',  0.8)
    min_y = cfg.get('min_y_simulation',  0.0)
    max_y = cfg.get('max_y_simulation',  0.8)
    if not LOAD_PARAMS:
        cabinet_positions = []
        z = heights * 0.5 + 0.01  # above ground
        for i in range(n):
            x = np.random.uniform(min_x, max_x)
            y = np.random.uniform(min_y, max_y)
            cabinet_positions.append([x, y, z[i]])
        cabinet_positions = np.array(cabinet_positions)
        np.save(os.path.join(simulation_dir, 'simulation_cabinet_positions.npy'), cabinet_positions)
    else:
        cabinet_positions = np.load(os.path.join(simulation_dir, 'simulation_cabinet_positions.npy'))

    # Random yaw rotations of cabinets
    z_rotations = np.random.uniform(-np.pi, np.pi, size=(n,))
    np.save(os.path.join(simulation_dir, 'simulation_cabinet_z_rotations.npy'), z_rotations)

    # Door opening interpolation
    n_rots = 20
    # rots = np.linspace(0, np.deg2rad(opening_angle), n_rots)
    # Tz_rots = np.zeros((n_rots, 4, 4))
    # Tz_rots[:, :3, :3] = rotz_multiple(rots)
    # Tz_rots[:, 3, 3] = 1.0

    # TCP offset in gripper frame (from YAML)
    T_TCP_G = np.eye(4)
    T_TCP_G[:3, 3] = np.array([-tx_G, 0.0, tz_G], dtype=float)

    # Generate the line of tool poses along max door height
    T_G_DD_all_full = generate_tool_line_poses(max_height, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)

    # === Robot interface & RVL manipulator ===
    robot = UR5Controller()
    robot.T_0_W = np.eye(4)

    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_cfg)
    rvl_manipulator.set_robot_pose(robot.T_0_W)
    py_touch = rvl_manipulator.py_touch
    rvl_touch_cfg = cfg.get('rvl_touch_cfg', {})
    py_touch.create(rvl_touch_cfg)
    camera_fu = 597.9033203125
    camera_fv = 598.47998046875
    camera_uc = 323.8436584472656
    camera_vc = 236.32774353027344
    camera_w = 640
    camera_h = 480
    py_touch.set_camera_params(
        camera_fu, camera_fv, camera_uc, camera_vc, camera_w, camera_h)
    touch_cfg = cfg.get("touch", None)
    touch_a = touch_cfg.get("a", 0.0)
    touch_b = touch_cfg.get("b", 0.0)
    touch_c = touch_cfg.get("c", 0.0)

    tool_cfg = cfg.get("tool", None)
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

    # Load gripper mesh
    gripper_mesh = o3d.io.read_triangle_mesh(gripper_mesh_path)
    gripper_mesh.compute_vertex_normals()

    # Tool transform from RVL then flip 180° about Z (kept from your script)
    robot.T_G_6 = np.load(cfg.get("T_G_6_path", None))
    # Tz = np.eye(4)
    # Tz[:3, :3] = rot_z(np.pi)
    # Save original then apply flip (optional)
    # np.save(os.path.join(analysis_dir, 'T_G_6.npy'), robot.T_G_6)
    # robot.T_G_6 = Tz @ robot.T_G_6

    # === Collision-free tool poses (load if present, else compute) ===
    if t_g_dd_colfree_path and os.path.exists(t_g_dd_colfree_path):
        T_G_DD_all_colfree = np.load(t_g_dd_colfree_path)
        T_TCP_D_colfree = T_G_DD_all_colfree.reshape(-1, 4, 4) @ T_TCP_G[np.newaxis, ...]
    else:
        # cabinet_model_init = Cabinet(
        #     door_params=np.array([0.4, 0.8, door_thickness, static_depth]),
        #     r=np.array([0., -0.4 * 0.5]),
        #     axis_pos=-1,
        #     T_A_S=np.eye(4),
        #     save_path=None
        # )
        # plate_mesh = copy.deepcopy(cabinet_model_init.dd_plate_mesh)
        # T_O_D = np.linalg.inv(cabinet_model_init.T_D_A_init) @ cabinet_model_init.T_A_O_init
        cabinet_model_init = Cabinet2(s=np.array([door_thickness, max_width, max_height, static_depth]),
                        r=np.array([-door_thickness*0.5, -max_width*0.5]),
                        axis_pos=-1,
                        T_A_W=np.eye(4),
                        save_path=None)
        cabinet_model_init.create_mesh()
        plate_mesh = copy.deepcopy(cabinet_model_init.plate_mesh)
        # T_O_D = np.linalg.inv(cabinet_model_init.T_D_Arot) @ cabinet_model_init.T_Arot_O
        T_O_D = np.linalg.inv(cabinet_model_init.T_D_A) @ np.linalg.inv(cabinet_model_init.T_A_O)

        plate_mesh.transform(T_O_D)

        # For reference/inspection
        o3d.io.write_triangle_mesh(cabinet_panel_mesh_path, plate_mesh)

        T_G_DD_ = T_G_DD_all_full[6000]
        # visualize_poses(self.gripper_mesh, plate_mesh, self.T_G_DD_all_full[0:2000])

        gripper_mesh_ = copy.deepcopy(gripper_mesh)
        gripper_mesh_.transform(T_G_DD_)
        gripper_mesh_.compute_vertex_normals()

        plate_mesh.transform(T_O_D)
        origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.zeros(3))
        plate_mesh = copy.deepcopy(cabinet_model_init.plate_mesh)
        plate_mesh.transform(T_O_D) 
        static_mesh = copy.deepcopy(cabinet_model_init.static_mesh)
        static_mesh.transform(T_O_D)
        o3d.visualization.draw_geometries([origin_rf, plate_mesh, static_mesh, gripper_mesh_])
        visualize_poses(gripper_mesh, plate_mesh, T_G_DD_all_full[0:2000], T_TCP_G)

        collisions = collision_detection_fcl(gripper_mesh, T_G_DD_all_full, plate_mesh)
        collisions = np.array(collisions, dtype=bool)
        collisions = np.invert(collisions)

        T_G_DD_all_colfree = T_G_DD_all_full[collisions, ...]
        # Save if a target path was given
        if t_g_dd_colfree_path:
            os.makedirs(os.path.dirname(t_g_dd_colfree_path), exist_ok=True)
            np.save(t_g_dd_colfree_path, T_G_DD_all_colfree)

        T_TCP_D_colfree = T_G_DD_all_colfree.reshape(-1, 4, 4) @ T_TCP_G[np.newaxis, ...]

    # === Main sampling loop ===
    count = 0
    rvl_manipulator_set = False
    with tqdm(total=n_saved) as pbar:
        for i_cabinet in range(n):
            z_rot = z_rotations[i_cabinet]
            T_A_W_cabinet = np.eye(4)
            T_A_W_cabinet[:3, :3] = rot_z(z_rot)
            T_A_W_cabinet[:3, 3] = cabinet_positions[i_cabinet]

            width  = widths[i_cabinet]
            height = heights[i_cabinet]
            state_angle = state_angles[i_cabinet]

            # Limit by height (filter out tool poses above this door)
            height_mask = np.invert(T_TCP_D_colfree[:, 1, 3] > height)
            T_G_DD_all = T_G_DD_all_colfree[height_mask, ...].copy()
            np.random.shuffle(T_G_DD_all)

            # Distance checks (door axis far enough from base, not too far overall)
            dist_axis = np.linalg.norm(T_A_W_cabinet[:2, 3])
            if not (dist_axis > base_size):
                continue

            # cabinet_model = Cabinet(
            #     door_params=np.array([width, height, door_thickness, static_depth]),
            #     r=np.array([0., -width * 0.5]),
            #     axis_pos=axis_pos,
            #     T_A_S=T_A_W_cabinet,
            #     save_path=None
            # )

            cabinet_model = Cabinet2(s=np.array([door_thickness, width, height, static_depth]),
                            r=np.array([-door_thickness*0.5, -width*0.5]),
                            axis_pos=-1,
                            T_A_W=T_A_W_cabinet,
                            save_path=None)
            cabinet_model.mesh_save_path = cabinet_mesh_path

    
            opening_angle = state_angle - opening_step_deg
            rots = np.linspace(0, np.deg2rad(opening_angle), n_rots)
            Tz_rots = np.zeros((n_rots, 4, 4))
            Tz_rots[:, :3, :3] = rotz_multiple(rots)
            Tz_rots[:, 3, 3] = 1.0

            T_D_W_rots = T_A_W_cabinet[np.newaxis, ...] @ Tz_rots @ cabinet_model.T_D_Arot[np.newaxis, ...]
            is_properly_rotated = np.dot(T_D_W_rots[0, :3, 3], T_D_W_rots[0, :3, 2]) > 0.0
            if not is_properly_rotated:
                continue

            closest_dists = line_seg_to_circle_dist_all(np.zeros((2,)), T_A_W_cabinet[:2, 3], T_D_W_rots[:, :2, 3])
            T_D_W_dists = np.linalg.norm(T_D_W_rots[:, :2, 3], axis=1)
            dist_crit = np.all((closest_dists > close_dist_range) & (T_D_W_dists < far_dist_range))
            if not dist_crit:
                continue

            # Save meshes (static + panel) for RVL FCL
            cabinet_model.save_mesh_without_doors(cabinet_static_mesh_path)
            cabinet_model.save_door_panel_mesh(cabinet_panel_mesh_path)
            if not rvl_manipulator_set:
                # Configure RVL manipulator for this cabinet
                rvl_manipulator.set_door_model_params(
                    cabinet_model.sx,
                    cabinet_model.sy,
                    cabinet_model.sz,
                    cabinet_model.rx,
                    cabinet_model.ry,
                    cabinet_model.axis_pos,
                    cabinet_model.side,
                    cabinet_model.moving_to_static_part_distance
                )
                rvl_manipulator.set_door_pose(cabinet_model.T_A_W)
                # rvl_manipulator.load_cabinet_static_mesh_fcl(cabinet_static_mesh_path)
                # rvl_manipulator.load_cabinet_panel_mesh_fcl(cabinet_panel_mesh_path)
                rvl_manipulator_set = True
                        
            T_Arot_A = np.eye(4)
            T_Arot_A[:3, :3] = rot_z(np.deg2rad(state_angle))
            T_Arot_W = cabinet_model.T_A_W @ T_Arot_A
            py_touch.set_session_params(cabinet_model.sx, cabinet_model.sy, cabinet_model.sz, cabinet_model.rx, cabinet_model.ry, touch_a, touch_b, touch_c, state_angle, T_C_6,
                            cabinet_model.sx, cabinet_model.sy, cabinet_model.sz, cabinet_model.rx, cabinet_model.ry, state_angle, T_Arot_W)
            rvl_manipulator.set_environment_from_touch_gt()
            T_D_S = cabinet_model.T_A_W @ T_Arot_A @ cabinet_model.T_D_Arot
            rvl_manipulator.set_pose_DD_S(T_D_S)
            
            
            # py_touch.set_new_scene(cabinet_model.sx, cabinet_model.sy, cabinet_model.sz, cabinet_model.rx, cabinet_model.ry, touch_a, touch_b, touch_c, state_angle, T_C_6, T_A_C, T_6_0_capture,
            #                 cabinet_model.sx, cabinet_model.sy, cabinet_model.sz, cabinet_model.rx, cabinet_model.ry, state_angle, T_Arot_W)

            # rvl_manipulator.update_model_x()
            # Tz_rot_state = np.eye(4)
            # Tz_rot_state[:3, :3] = rot_z(np.deg2rad(state_angle))
            # T_D_S = cabinet_model.T_A_W @ Tz_rot_state @ cabinet_model.T_D_Arot
            # rvl_manipulator.set_pose_DD_S(T_D_S)
            # rvl_manipulator.set_environment_from_touch()
            
            # Check opening path exists (reusing your helper)
            # approaches, trajectories = generate_trajectories_and_approach3(
            #     T_G_DD_all, 
            #     2, 
            #     state_angle, 
            #     opening_angle, 
            #     cabinet_model, 
            #     rvl_manipulator, 
            #     robot.T_0_W, 
            #     robot,
            #     verbose=False,
            #     visualize=False
            # )            
            approaches = generate_one_trajectory_and_approach(
                T_G_DD_all,
                2,
                state_angle,
                opening_angle,
                cabinet_model,
                rvl_manipulator,
                robot.T_0_W,
                robot,
                verbose=False
            )
            if len(approaches) < 1:
                continue
            
            # Pick a random approach
            # root = approaches[np.random.randint(len(approaches))]
            # T_6_0_traj = root.get_full_trajectory_task_space()
            # T_6_0 = T_6_0_traj[2]
            # root = approaches[0]
            # leaves = [root] if root.is_leaf() else get_all_leaves(root) # type: list[Waypoint]

            contact_q = approaches[0, 2]
            T_6_0 = robot.get_fwd_kinematics_moveit(contact_q)
            T_6_0 = np.array(T_6_0)

            # Optional visualization
            if VISUALIZE:
                cabinet_model.change_door_angle(state_angle)

                T_O_W = T_A_W_cabinet @ np.linalg.inv(cabinet_model.T_A_O)
                mesh = cabinet_model.create_mesh()
                origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                mesh.paint_uniform_color([0.5, 0.5, 0.5])
                mesh_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                mesh_rf.transform(T_O_W)
                mesh.transform(T_O_W)
                mesh.compute_vertex_normals()
                T_6_0_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                T_6_0_rf.transform(T_6_0)
                D_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                D_rf.transform(T_D_S)

                # gripper_mesh_0 = copy.deepcopy(gripper_mesh)
                # gripper_mesh_0.transform(T_6_0_traj[0] @ robot.T_G_6)
                # gripper_mesh_0.compute_vertex_normals()
                # gripper_mesh_0.paint_uniform_color([0.8, 0.1, 0.1])
                # gripper_mesh_1 = copy.deepcopy(gripper_mesh)
                # gripper_mesh_1.transform(T_6_0_traj[1] @ robot.T_G_6)
                # gripper_mesh_1.compute_vertex_normals()
                # gripper_mesh_1.paint_uniform_color([0.1, 0.8, 0.1])
                # gripper_mesh_2 = copy.deepcopy(gripper_mesh)
                # gripper_mesh_2.transform(T_6_0_traj[2] @ robot.T_G_6)
                # gripper_mesh_2.compute_vertex_normals()
                # gripper_mesh_2.paint_uniform_color([0.1, 0.1, 0.8])

                # o3d.visualization.draw_geometries(
                #     [mesh, origin_rf, mesh_rf, gripper_mesh_0, gripper_mesh_1, gripper_mesh_2, D_rf],
                #     window_name='Cabinet', width=800, height=600, left=50, top=50, mesh_show_back_face=True
                # )

            # Save cabinet instance row
            if SAVE_CABINETS:
                with open(cabinet_configs_path, 'a') as f:
                    T_A_0 = np.linalg.inv(robot.T_0_W) @ T_A_W_cabinet
                    t_A_0 = T_A_0[:3, 3].reshape(-1)
                    z_rot_deg = np.rad2deg(z_rotations[i_cabinet])

                    t_6_0 = T_6_0[:3, 3].reshape(-1)
                    R_6_0 = T_6_0[:3, :3].reshape(-1)
                    f.write(f'{i_cabinet},{cabinet_model.sx},{cabinet_model.sy},{cabinet_model.sz},{cabinet_model.rx},{cabinet_model.ry},{state_angle}')
                    for i in range(3):
                        f.write(f',{t_A_0[i]}')
                    f.write(f',{z_rot_deg}')
                    for i in range(9):
                        f.write(f',{R_6_0[i]}')
                    for i in range(3):
                        f.write(f',{t_6_0[i]}')
                    f.write('\n')

            count += 1
            tqdm.write(f'Kept {count}/{n_saved}')
            if count >= n_saved:
                break


if __name__ == '__main__':
    main()
