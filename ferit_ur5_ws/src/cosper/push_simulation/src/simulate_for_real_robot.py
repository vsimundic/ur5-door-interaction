#!/usr/bin/python

import rospy
import os
import rospkg
from core.read_config import read_config
from core.paths_packages import get_package_name_from_node, get_package_path_from_name
from core.ur5_commander import UR5Commander
from DDMan import push
from gazebo_push_open.cabinet_model import Cabinet
# from cabinet_model import generate_cabinet_urdf_from_door_panel, get_cabinet_world_pose
import numpy as np
from core.transforms import rot_z, matrix_to_pose
import RVLPYDDManipulator as rvlpy_dd_man
from trac_ik_python.trac_ik import IK
import roslaunch

if __name__ == '__main__':
    rospy.init_node('node_simulations')
    
    cfg_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/simulations_config.yaml'
    
    config = read_config(cfg_path)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/RVLuser/ferit_ur5_ws/src/ur5_configs/ur5_robotiq_ft_3f_moveit_config/launch/demo_gazebo.launch"])
    launch.start()
    rospy.loginfo('Started')

    rospy.sleep(10)
    # launch.shutdown()
    
    # Robot handler
    robot = UR5Commander()

    #### CABINET ####
    cabinet_door_dims = config['cabinet_door_dims'] # w_door, h_door, static_d, d_door 
    door_params = [np.random.uniform(cabinet_door_dims['min_width'], cabinet_door_dims['max_width']), 
                  np.random.uniform(cabinet_door_dims['min_height'], cabinet_door_dims['max_height']),
                  cabinet_door_dims['depth'],
                  cabinet_door_dims['static_depth']]
    
    cabinet_pose = config['cabinet_pose']
    feasible_poses_args = config['feasible_poses']

    # Cabinet pose in world
    cabinet_position = [np.random.uniform(cabinet_pose['min_x'], cabinet_pose['max_x']),
                        np.random.uniform(cabinet_pose['min_y'], cabinet_pose['max_y']),
                        door_params[1]/2 + 0.018 + 0.005 + 0.005] # half of door height + depth of bottom panel + double static moving part distance
    rotz_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])

    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array(cabinet_position)
    Tz_init = np.eye(4)
    Tz_init[:3, :3] = rot_z(np.radians(90.))
    T_A_S = T_A_S @ Tz_init
    Tz = np.eye(4)
    Tz[:3, :3] = rot_z(np.radians(rotz_deg))
    T_A_S = T_A_S @ Tz


    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array(door_params), 
                            axis_pos=cabinet_pose['axis_pos'],
                            T_A_S=T_A_S,
                            # position=np.array(cabinet_position),
                            # rotz_deg=rotz_deg,
                            save_path=config['cabinet_urdf_save_path'])
    
    # Save cabinet mesh to a file
    cabinet_model.save_mesh_without_doors(config['cabinet_mesh_save_path'])

    # Spawning model in Gazebo
    cabinet_model.delete_model_gazebo()
    cabinet_model.spawn_model_gazebo()

    # Open doors in Gazebo
    theta_deg = config['feasible_poses']['dd_state_deg']
    cabinet_model.open_door_gazebo(theta_deg)
    cabinet_model.change_door_angle(theta_deg)
    cabinet_model.update_mesh()


    #### FEASIBLE POSES ####
    # Get feasible poses
    feasible_poses_args['door_dims'] = np.array(door_params[:3])
    feasible_poses_args['static_depth'] = door_params[3]
    feasible_poses = push.demo_push_poses_ros(**feasible_poses_args)


    # Path planning
    path_planner = rvlpy_dd_man.PYDDManipulator()
    path_planner.create(config['rvl_config_path'])
    path_planner.load_tool_model(config['tool_model_params'])
    path_planner.set_environment_state(config['feasible_poses']['dd_state_deg'])
    # path_planner.load_feasible_tool_contact_poses(feasible_poses_args['feasible_poses_path'])
    path_planner.load_feasible_tool_contact_poses(feasible_poses_args['feasible_poses_path'])
    path_planner.set_robot_pose(robot.T_B_S)
    # T_A_S = cabinet_model.T_O_S @ cabinet_model.T_A_O_init

    # np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_A_S_temp.npy', T_A_S)
    path_planner.set_door_model_params(
                                       cabinet_model.d_door,
                                       cabinet_model.w_door,
                                       cabinet_model.h_door,
                                       0.0, # rx
                                       -(cabinet_model.w_door/2. - cabinet_model.axis_distance), # ry
                                       cabinet_model.axis_pos, # opening direction
                                       cabinet_model.static_side_width,
                                       cabinet_model.moving_to_static_part_distance) 
    path_planner.set_door_pose(T_A_S)



    q_init = robot.get_current_joint_values()
    q_init = np.array(q_init)
    # adjust joint values from ROS
    q_init[0] += np.pi
    q_init[5] += np.pi
    q_init[q_init>np.pi]-=(2.0*np.pi)     
    q_init[q_init<-np.pi]+=(2.0*np.pi)

    T_G_0_array, q = path_planner.path2(np.array(q_init))

    if T_G_0_array.shape[0] == 1:
        print('Path not found')
        exit()
    # adjust joint values to ROS
    q[:, 0] += np.pi
    q[:, 5] += np.pi
    q[q>np.pi]-=(2.0*np.pi)     
    q[q<-np.pi]+=(2.0*np.pi)

    executed = robot.send_multiple_joint_space_poses_to_robot(q, execute_time=30, wait=True)


