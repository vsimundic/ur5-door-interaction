#!/usr/bin/python

import rospy
from core.ur5_commander import UR5Commander
import numpy as np


if __name__ == '__main__':
    rospy.init_node('node_save_robot_home_pose')

    save_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/home_pose_joints.npy'
    save_path_pose = '/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_T_B_home.npy'

    robot = UR5Commander()

    joint_values = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/home_pose_joints.npy')

    robot.send_joint_values_to_robot(joint_values=joint_values)

    T_T_B = robot.get_current_tool_pose()

    np.save(save_path_pose, T_T_B)

    print(T_T_B)
    
    rospy.sleep(3)
    robot.save_current_joint_values(save_path)
    rospy.loginfo('Joint values saved.')