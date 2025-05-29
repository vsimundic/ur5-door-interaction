import numpy as np
import os
from core.transforms import *
import tqdm
from gazebo_push_open.cabinet_model import Cabinet
from core.real_ur5_controller import UR5Controller
from push_force_trajectories import generate_tool_line_poses, generate_trajectories_and_approach2
from force_utils import *
import pickle 
import open3d as o3d
import RVLPYDDManipulator as rvlpy


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


np.random.seed(12345)

rospy.init_node('generate_gt_poses', anonymous=True)

n = 100000
n_saved = 10
width = 0.396
height = 0.496
door_thickness = 0.018
static_depth = 0.4
axis_pos = -1
push_latch_mechanism_length = 0.046
state_angle = axis_pos * np.rad2deg(np.arcsin(push_latch_mechanism_length/width))
opening_angle = -45.0

base_size = close_dist_range = 0.3
far_dist_range = 0.9

T_A_W = np.eye(4)
T_A_W[:3, :3] = np.array([[0, -1, 0],
                          [1, 0, 0],
                          [0, 0, 1]])

# Random generate cabinet position
cabinet_positions = []
z = height*0.5 + 0.01 # 0.01 is the offset of the cabinet from the ground
for i in range(n):
    x = np.random.uniform(-0.36, 0.0)
    y = np.random.uniform(0.41, 0.71)
    cabinet_positions.append([x, y, z])
cabinet_positions = np.array(cabinet_positions)

# z_rotations = np.random.uniform(0, 0, size=(n,))
z_rotations = np.random.uniform(-np.pi*0.25, np.pi*0.25, size=(n,))

Tz = np.zeros((n, 4, 4))
Tz[:, :3, :3] = rotz_multiple(z_rotations)
Tz[:, 3, 3] = 1

T_A_W_cabinets = np.zeros((n, 4, 4))
T_A_W_cabinets[:] = T_A_W[np.newaxis, ...] @ Tz
T_A_W_cabinets[:, :3, 3] = cabinet_positions
T_A_W_cabinets[:, 3, 3] = 1

n_rots = 50
rots = np.linspace(0, np.deg2rad(opening_angle), n_rots)
Tz_rots = np.zeros((n_rots, 4, 4))
Tz_rots[:, :3, :3] = rotz_multiple(rots)
Tz_rots[:, 3, 3] = 1

cabinet_model_init = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                            r=np.array([0., -width*0.5]),
                            axis_pos=axis_pos,
                            T_A_S=np.eye(4),
                            save_path=None)
T_D_A = cabinet_model_init.T_D_A.copy()

T_TCP_G = np.eye(4)
T_TCP_G[:3, 3] = np.array([0.155 * 0.5 - 0.007, 0, 0.100])
R_TCP_D = np.array([[0, 0, -1],
                    [0, 1, 0],
                    [1, 0, 0]])

T_G_DD_all = generate_tool_line_poses(height, T_TCP_G, R_TCP_D).reshape(-1, 4, 4)

robot = UR5Controller()
robot.T_0_W = np.eye(4)
rvl_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'

rvl_manipulator = rvlpy.PYDDManipulator()
rvl_manipulator.create(rvl_cfg)
rvl_manipulator.set_robot_pose(robot.T_0_W)

base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'
cabinet_static_mesh_path = os.path.join(base_dir, 'cabinet_model/cabinet_static.ply')
cabinet_panel_mesh_path = os.path.join(base_dir, 'cabinet_model/cabinet_panel.ply')


T_A_W_saved = []
trajectories_per_pose = []
# count_pts = 0
# count_dist = 0
for i_cabinet in tqdm.tqdm(range(n)):
    T_A_W_cabinet = T_A_W_cabinets[i_cabinet]
    
    T_pt1_A = np.eye(4)
    T_pt1_A[:3, 3] = np.array([0., 0., 0.])
    T_pt2_A = np.eye(4)
    T_pt2_A[:3, 3] = np.array([static_depth, 0., 0.])
    T_pt3_A = np.eye(4)
    T_pt3_A[:3, 3] = np.array([static_depth, -width, 0.])
    T_pt4_A = np.eye(4)
    T_pt4_A[:3, 3] = np.array([0., -width, 0.])

    T_pt1_W = T_A_W_cabinet @ T_pt1_A
    T_pt2_W = T_A_W_cabinet @ T_pt2_A
    T_pt3_W = T_A_W_cabinet @ T_pt3_A
    T_pt4_W = T_A_W_cabinet @ T_pt4_A

    pts_crit = T_pt1_W[0, 3] > -0.35 and T_pt2_W[0, 3] > -0.35 and T_pt3_W[0, 3] < 0.35 and T_pt4_W[0, 3] < 0.35
    if not pts_crit:
        # count_pts += 1
        continue

    T_D_W_rots = T_A_W_cabinet[np.newaxis,...] @ Tz_rots @ T_D_A[np.newaxis,...]
    closest_dists = line_seg_to_circle_dist_all(np.zeros((2,)), T_A_W_cabinet[:2,3], T_D_W_rots[:,:2,3])
    T_D_W_dists = np.linalg.norm(T_D_W_rots[:, :2, 3], axis=1)
    dist_crit = np.all((closest_dists > close_dist_range) & (T_D_W_dists < far_dist_range))
    if not dist_crit:
        # count_dist += 1
        continue

    cabinet_model = Cabinet(door_params=np.array([width, height, door_thickness, static_depth]), 
                            r=np.array([0., -width*0.5]),
                            axis_pos=axis_pos,
                            T_A_S=T_A_W_cabinet,
                            save_path=None)

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

    # # Visualize the cabinet
    # T_O_W = T_A_W_cabinet @ np.linalg.inv(cabinet_model.T_A_O_init)
    # mesh = cabinet_model.create_mesh()
    # origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    # mesh.paint_uniform_color([0.5, 0.5, 0.5])
    # mesh_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    # mesh_rf.transform(T_O_W)
    # mesh.transform(T_O_W)
    # mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh, origin_rf, mesh_rf], window_name='Cabinet', width=800, height=600, left=50, top=50, mesh_show_back_face=True)


    # Check if the opening path exists
    _, trajectories = generate_trajectories_and_approach2(
        T_G_DD_all, 
        11, 
        state_angle, 
        opening_angle, 
        cabinet_model, 
        rvl_manipulator, 
        robot.T_0_W, 
        robot
    )

    if trajectories.shape[0] < 1:
        continue

    trajectories_per_pose.append(trajectories)
    T_A_W_saved.append(T_A_W_cabinet)
    print(f"Cabinet {i_cabinet} saved. Total cabinets: {len(T_A_W_saved)}")
    if len(T_A_W_saved) >= n_saved:
        break

print(f"Total cabinets: {len(T_A_W_saved)}")
T_A_W_saved = np.array(T_A_W_saved)

np.save('/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/gt_cabinets/cabinet_gt_poses.npy', T_A_W_saved)
with open('/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/gt_cabinets/cabinet_gt_trajectories.pkl', 'wb') as f:
    pickle.dump(trajectories_per_pose, f)