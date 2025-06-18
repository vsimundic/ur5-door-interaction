import numpy as np
import os
from core.transforms import *
from tqdm import tqdm
from gazebo_push_open.cabinet_model import Cabinet
from core.real_ur5_controller import UR5Controller
from push_force_trajectories import *
from force_utils import *
import pickle 
import open3d as o3d
import RVLPYDDManipulator as rvlpy
import copy
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
    """
    Computes the minimum distances from a point (cir) to multiple line segments.
    Each segment is defined by a fixed starting point p1 and an array of endpoints p2s.
    
    Parameters:
      cir: array-like of shape (2,), the point (e.g., circle center) as (x, y)
      p1: array-like of shape (2,), the fixed endpoint (e.g., door hinge)
      p2s: array-like of shape (N,2), the other endpoints for each rotated door position
      
    Returns:
      A NumPy array of shape (N,) containing the Euclidean distances from cir to the closest 
      point on each segment.
    """
    # Ensure inputs are NumPy arrays with float type
    p1 = np.array(p1, dtype=float)
    cir = np.array(cir, dtype=float)
    p2s = np.array(p2s, dtype=float)  # shape: (N, 2)
    
    # Compute the segment vectors for each rotated door (p2s - p1)
    seg = p2s - p1                # shape: (N,2)
    seg_len_sq = np.sum(seg**2, axis=1)  # shape: (N,)
    
    # Vector from p1 to the circle center (broadcasted)
    cir_vec = cir - p1            # shape: (2,)
    
    # Dot product between cir_vec and each segment vector
    dot_val = np.sum(seg * cir_vec, axis=1)  # shape: (N,)
    
    # Compute projection factor for each segment and clamp between 0 and 1.
    # If a segment is degenerate (length zero), t is set to 0.
    t = np.where(seg_len_sq > 0, np.clip(dot_val / seg_len_sq, 0, 1), 0)
    
    # Compute the projected points along each segment
    proj_pts = p1 + t[:, None] * seg  # shape: (N,2)
    
    # Compute distances from the circle center to each projected point
    dist = np.linalg.norm(proj_pts - cir, axis=1)
    return dist

def main():
    rospy.init_node('generate_simulation_poses', anonymous=True)

    n = 100000
    n_saved = 1000
    LOAD_PARAMS = False
    VISUALIZE = False
    SAVE_CABINETS = True
    base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/RVL_data'
    cabinet_configs_path = os.path.join(base_dir, 'simulation_cabinet_configs.csv')

    if SAVE_CABINETS:
        with open(cabinet_configs_path, 'w') as f:
            f.write('idx,sx,sy,sz,rx,ry,state_angle,t_A_0,z_rot,R_E_0_contact,t_E_0_contact\n')

    # Generate random dimensions for the cabinet doors
    if not LOAD_PARAMS:
        widths = np.random.uniform(0.2, 0.6, size=(n,))    
        heights = np.random.uniform(0.2, 0.8, size=(n,))
        np.save(os.path.join(base_dir, 'simulation_cabinet_widths.npy'), widths)
        np.save(os.path.join(base_dir, 'simulation_cabinet_heights.npy'), heights)
    else:
        widths = np.load(os.path.join(base_dir, 'simulation_cabinet_widths.npy'))
        heights = np.load(os.path.join(base_dir, 'simulation_cabinet_heights.npy'))

    door_thickness = 0.018
    static_depth = 0.4
    axis_pos = -1
    push_latch_mechanism_length = 0.046
    state_angles = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length/widths))
    opening_angle = -45.0
    max_door_height = 0.8

    base_size = close_dist_range = 0.3
    far_dist_range = 0.9

    T_A_W = np.eye(4)
    T_A_W[:3, :3] = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])

    # Random generate cabinet position
    if not LOAD_PARAMS:
        cabinet_positions = []
        z = heights*0.5 + 0.01 # 0.01 is the offset of the cabinet from the ground
        for i in range(n):
            x = np.random.uniform(-0.75, 0.75)
            y = np.random.uniform(0.0, 0.75)
            cabinet_positions.append([x, y, z[i]])
        cabinet_positions = np.array(cabinet_positions)
        np.save(os.path.join(base_dir, 'simulation_cabinet_positions.npy'), cabinet_positions)
    else:
        cabinet_positions = np.load(os.path.join(base_dir, 'simulation_cabinet_positions.npy'))

    # z_rotations = np.random.uniform(0, 0, size=(n,))
    z_rotations = np.random.uniform(-np.pi, np.pi, size=(n,))
    np.save(os.path.join(base_dir, 'simulation_cabinet_z_rotations.npy'), z_rotations)

    n_rots = 50
    rots = np.linspace(0, np.deg2rad(opening_angle), n_rots)
    Tz_rots = np.zeros((n_rots, 4, 4))
    Tz_rots[:, :3, :3] = rotz_multiple(rots)
    Tz_rots[:, 3, 3] = 1

    T_TCP_G = np.eye(4)
    T_TCP_G[:3, 3] = np.array([0.155 * 0.5, 0, 0.100])
    R_TCP_D = np.array([[0, 0, -1],
                        [0, 1, 0],
                        [1, 0, 0]])

    # Generate the tool poses for the gripper for max door height
    T_G_DD_all_full = generate_tool_line_poses(max_door_height, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)

    robot = UR5Controller()
    robot.T_0_W = np.eye(4)
    rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

    rvl_manipulator = rvlpy.PYDDManipulator()
    rvl_manipulator.create(rvl_cfg)
    rvl_manipulator.set_robot_pose(robot.T_0_W)

    base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'
    cabinet_static_mesh_path = os.path.join(base_dir, 'cabinet_model/cabinet_static.ply')
    cabinet_panel_mesh_path = os.path.join(base_dir, 'cabinet_model/cabinet_panel.ply')

    gripper_mesh = o3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply')
    gripper_mesh.compute_vertex_normals()
    robot.T_G_6 = rvl_manipulator.get_T_G_6()
    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.pi)
    np.save(os.path.join(base_dir, 'RVL_data/T_G_6.npy'), robot.T_G_6)
    robot.T_G_6 = Tz @ robot.T_G_6 

    LOAD_COLLISION_POSES = False

    # Collision detection
    if not LOAD_COLLISION_POSES:
        cabinet_model_init = Cabinet(door_params=np.array([0.4, 0.8, door_thickness, static_depth]), 
                                r=np.array([0., -0.4*0.5]),
                                axis_pos=-1,
                                T_A_S=np.eye(4),
                                save_path=None)
        plate_mesh = copy.deepcopy(cabinet_model_init.dd_plate_mesh)
        T_O_D = np.linalg.inv(cabinet_model_init.T_D_A_init) @ cabinet_model_init.T_A_O_init
        plate_mesh.transform(T_O_D) 

        o3d.io.write_triangle_mesh(cabinet_panel_mesh_path, plate_mesh)
        collisions = collision_detection_fcl(gripper_mesh, T_G_DD_all_full, plate_mesh)
        collisions = np.array(collisions, dtype=bool)
        collisions = np.invert(collisions)

        T_G_DD_all_colfree = T_G_DD_all_full[collisions, ...]
        np.save(os.path.join(base_dir, 'simulation_tool_poses.npy'), T_G_DD_all_colfree)
        T_TCP_D_colfree = T_G_DD_all_colfree.reshape(-1, 4, 4) @ T_TCP_G[np.newaxis, ...]
    else:
        T_G_DD_all_colfree = np.load(os.path.join(base_dir, 'simulation_tool_poses.npy'))
        T_TCP_D_colfree = T_G_DD_all_colfree.reshape(-1, 4, 4) @ T_TCP_G[np.newaxis, ...]

    count = 0
    with tqdm(total=n_saved) as pbar:
        for i_cabinet in range(n):
            # i_cabinet = 1 # debug
            z_rot = z_rotations[i_cabinet]
            T_A_W_cabinet = np.eye(4)
            T_A_W_cabinet[:3, :3] = rot_z(z_rot)
            T_A_W_cabinet[:3, 3] = cabinet_positions[i_cabinet]

            width = widths[i_cabinet]
            height = heights[i_cabinet]
            state_angle = state_angles[i_cabinet]

            height_scale_array = np.invert(T_TCP_D_colfree[:, 1, 3] > height) 
            T_G_DD_all = T_G_DD_all_colfree[height_scale_array, ...]

            dist_axis = np.linalg.norm(T_A_W_cabinet[:2, 3])
            close_axis_dist_from_base = dist_axis > base_size 
            if not close_axis_dist_from_base:
                continue

            cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                                    r=np.array([0., -width*0.5]),
                                    axis_pos=axis_pos,
                                    T_A_S=T_A_W_cabinet,
                                    save_path=None)

            T_D_W_rots = T_A_W_cabinet[np.newaxis,...] @ Tz_rots @ cabinet_model.T_D_A_init[np.newaxis,...]
            is_properly_rotated = np.dot(T_D_W_rots[0, :3, 3], T_D_W_rots[0, :3, 2]) > 0.
            if not is_properly_rotated:
                continue
            closest_dists = line_seg_to_circle_dist_all(np.zeros((2,)), T_A_W_cabinet[:2,3], T_D_W_rots[:,:2,3])
            T_D_W_dists = np.linalg.norm(T_D_W_rots[:, :2, 3], axis=1)
            dist_crit = np.all((closest_dists > close_dist_range) & (T_D_W_dists < far_dist_range))
            if not dist_crit:
                continue
            
            cabinet_model.save_mesh_without_doors(cabinet_static_mesh_path)
            cabinet_model.save_door_panel_mesh(cabinet_panel_mesh_path)

            rvl_manipulator.set_door_model_params(
                cabinet_model.d_door,
                cabinet_model.w_door,
                cabinet_model.h_door,
                cabinet_model.rx,
                cabinet_model.ry,
                cabinet_model.axis_pos,
                cabinet_model.static_side_width,
                cabinet_model.moving_to_static_part_distance)
            rvl_manipulator.set_door_pose(cabinet_model.T_A_S)
            rvl_manipulator.load_cabinet_static_mesh_fcl(cabinet_static_mesh_path)
            rvl_manipulator.load_cabinet_panel_mesh_fcl(cabinet_panel_mesh_path)

            # Check if the opening path exists
            approaches = generate_one_trajectory_and_approach(
                T_G_DD_all, 
                9, 
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
            root = approaches[0]
            leaves = [root] if root.is_leaf() else get_all_leaves(root)
            contact_q = leaves[0].q
            T_6_0 = robot.get_fwd_kinematics_moveit(contact_q)
            T_6_0 = np.array(T_6_0)

            # Visualize the cabinet
            if VISUALIZE:
                T_O_W = T_A_W_cabinet @ np.linalg.inv(cabinet_model.T_A_O_init)
                cabinet_model.change_door_angle(state_angle)
                mesh = cabinet_model.create_mesh()
                origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                mesh.paint_uniform_color([0.5, 0.5, 0.5])
                mesh_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                mesh_rf.transform(T_O_W)
                mesh.transform(T_O_W)
                mesh.compute_vertex_normals()
                T_6_0_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                T_6_0_rf.transform(T_6_0)
                gripper_mesh_ = copy.deepcopy(gripper_mesh)
                gripper_mesh_.transform(T_6_0 @ robot.T_G_6)
                o3d.visualization.draw_geometries([mesh, origin_rf, mesh_rf, gripper_mesh_], window_name='Cabinet', width=800, height=600, left=50, top=50, mesh_show_back_face=True)

            if SAVE_CABINETS:
                with open(cabinet_configs_path, 'a') as f:
                    T_A_0 = np.linalg.inv(robot.T_0_W) @ T_A_W_cabinet
                    t_A_0 = T_A_0[:3, 3].reshape(-1)
                    z_rot = np.rad2deg(z_rotations[i_cabinet])

                    t_6_0 = T_6_0[:3, 3].reshape(-1)
                    R_6_0 = T_6_0[:3, :3].reshape(-1)
                    f.write(f'{i_cabinet},{cabinet_model.d_door},{cabinet_model.w_door},{cabinet_model.h_door},{cabinet_model.rx},{cabinet_model.ry},{state_angle}')
                    for i in range(3):
                        f.write(f',{t_A_0[i]}')
                    f.write(f',{z_rot}')
                    for i in range(9):
                        f.write(f',{R_6_0[i]}')
                    for i in range(3):
                        f.write(f',{t_6_0[i]}')
                    f.write('\n')
            count += 1
            pbar.update(1)
            if count >= n_saved:
                break

