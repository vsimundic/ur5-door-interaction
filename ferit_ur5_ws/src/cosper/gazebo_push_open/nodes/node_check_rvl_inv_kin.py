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

if __name__ == '__main__':
    rospy.init_node('node_check_rvl_inv_kin')
    
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
    # np.save('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_A_S_temp.npy', T_A_S)
    path_planner.set_door_model_params(
                                       # T_A_S,
                                       cabinet_model.d_door,
                                       cabinet_model.w_door,
                                       cabinet_model.h_door,
                                       0.0, # rx
                                    #    -(cabinet_model.w_door/2. - cabinet_model.axis_distance), # ry
                                       -0.14, # ry
                                       1.0, # opening direction
                                       cabinet_model.static_side_width,
                                       cabinet_model.moving_to_static_part_distance) 
    path_planner.set_door_pose(T_A_S)


    rospy.loginfo('Move the robot in RViz and press a key.')
    input()

    T_T_B_init = robot.get_current_tool_pose()
    gazebo_q = robot.get_current_joint_values() 
    T_G_B_current = T_T_B_init @ robot.T_G_T
    rvl_q, success = path_planner.inv_kinematics(T_G_B_current[:, :].astype('double'))

    rvl_q = [float(q) for q in list(rvl_q)]
    robot.send_joint_values_to_robot(rvl_q)
    T_T_B_current = robot.get_current_tool_pose()

    print(str(T_T_B_init) + '\n' + str(T_T_B_current))
    print(str(rvl_q) + '\n' + str(gazebo_q))
