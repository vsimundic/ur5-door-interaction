#!/usr/bin/python

import rospy
from core.ur5_commander import UR5Commander
import numpy as np


if __name__ == '__main__':
    rospy.init_node('node_save_robot_home_pose')

    save_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/home_pose_joints.npy'
    save_path_pose = '/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_T_B_home.npy'
    joint_values_init_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/joint_values_init.npy'
    robot = UR5Commander()

    joint_values = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/joint_values_init.npy')
    T_G_T = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_G_T.npy')
    T_B_S = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_B_S.npy')
    
    # T_T_B_home = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_T_B_home.npy')
    # # robot.send_pose_to_robot(T_T_B_home)
    
    # robot.save_current_joint_values(joint_values_init_path)
    
    robot.send_joint_values_to_robot(joint_values=joint_values)
    T_T_B_home = robot.get_current_tool_pose()
    
    a = input('this is just a dummy input')
    # T_T_B = robot.get_current_tool_pose()
    np.save(save_path_pose, T_T_B_home)

    # print(T_T_B)

    # T_G_S = T_B_S @ T_T_B @ T_G_T
    # print(T_G_S)
    
    # rospy.sleep(3)
    # robot.save_current_joint_values(save_path)
    # rospy.loginfo('Joint values saved.')