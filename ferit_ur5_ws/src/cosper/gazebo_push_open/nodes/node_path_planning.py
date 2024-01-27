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
from core.transforms import rot_z
import RVLPYDDManipulator as rvlpy_dd_man

if __name__ == '__main__':
    rospy.init_node('node_path_planning')
    
    try:
        cfg_file = rospy.get_param('config_file')
    except rospy.exceptions.ROSException:
        raise Exception('Could not fetch param.')
    
    config = read_config(cfg_file)
    
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
                        door_params[1]/2 + 0.018 + 0.01] # half of door height + depth of bottom panel + double static moving part distance
    rotz_deg = np.random.uniform(cabinet_pose['rot_angle_min_deg'], cabinet_pose['rot_angle_max_deg'])

    # Create a cabinet object
    cabinet_model = Cabinet(door_params=np.array(door_params), 
                            axis_pos=cabinet_pose['axis_pos'],
                            position=np.array(cabinet_position),
                            rotz_deg=rotz_deg,
                            save_path=config['cabinet_urdf_save_path'])
    
    # Save cabinet mesh to a file
    cabinet_model.save_mesh(config['cabinet_mesh_save_path'])

    # Spawning model in Gazebo
    cabinet_model.delete_model_gazebo()
    cabinet_model.spawn_model_gazebo()
    T_D_S_gazebo = cabinet_model.get_door_panel_RF_wrt_world()

    # Open doors in Gazebo
    theta_deg = config['feasible_poses']['dd_state_deg']
    cabinet_model.open_door_gazebo(theta_deg)
    cabinet_model.change_door_angle(theta_deg)
    cabinet_model.update_mesh()

    # TEST SPHERES
    cabinet_model.delete_model_gazebo_sphere()
    cabinet_model.spawn_model_gazebo_sphere()

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
    T_A_S = cabinet_model.T_O_S @ cabinet_model.T_A_O_init
    np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_A_S_temp.npy', T_A_S)
    path_planner.set_door_model_params(T_A_S,
                                       cabinet_model.d_door,
                                       cabinet_model.w_door,
                                       cabinet_model.h_door,
                                       0.0, # rx
                                    #    -(cabinet_model.w_door/2. - cabinet_model.axis_distance), # ry
                                       -0.14, # ry
                                       1.0, # opening direction
                                       cabinet_model.static_side_width,
                                       cabinet_model.moving_to_static_part_distance) 
    
    T_G_S_init = robot.T_B_S @ robot.T_T_B_home @ robot.T_G_T
    np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_G_S_init_temp.npy', T_G_S_init)
    
    
    # Get points for the whole path
    T_D_S_gazebo = cabinet_model.get_door_panel_RF_wrt_world()
    T_G_0 = path_planner.path2(T_G_S_init)

    path_planner.set_environment_state(config['feasible_poses']['dd_state_deg'])
    T_D_S_rvl = path_planner.get_T_DD_S()


    T_F_S_gazebo = cabinet_model.T_O_S @ cabinet_model.T_F_O
    T_F_S_rvl = path_planner.get_T_F_S()

    path_planner.set_environment_state(0.)
    T_D_S__rvl0 = path_planner.get_T_DD_S()
    # # Add cabinet mesh to rviz
    # robot.add_mesh_to_scene(config['cabinet_mesh_save_path'], 'cabinet', np.linalg.inv(robot.T_B_S) @ cabinet_model.T_O_S)

    for i in range(T_G_0.shape[0]):
        rospy.loginfo('Pose %s' %i)
        T_G_S_ = T_G_0[i]
        T_T_B_ = robot.get_tool_pose_from_gripper_pose(T_G_S_)
        robot.send_pose_to_robot(T_T_B_)

        T_T_B_curr = robot.get_current_tool_pose()
        T_G_S_curr = robot.T_B_S @ T_T_B_curr @ robot.T_G_T


        pass

    # # Passing poses 
    # robot.send_multiple_poses_to_robot(T_T_B_arr, wait=True, cartesian=True)

