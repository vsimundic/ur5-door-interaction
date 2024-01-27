import numpy as np
import rvlpyutil as rvlutil
import os

import RVLPYDDManipulator as rvl

print('Starting...')

manipulator = rvl.PYDDManipulator()
manipulator.create('/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec.cfg')
manipulator.load_tool_model('/home/RVLuser/rvl-linux/modules/RVLMotionDemo/Robotiq3Finger')
manipulator.load_feasible_tool_contact_poses('/home/RVLuser/rvl-linux/data/DDMan/3finger_gripper/feasible_poses.npy')
manipulator.set_robot_pose(np.eye(4))
# T_F_S = np.array([[0.0, 0.0, 1.0, 0.6], [-1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.546], [0, 0, 0, 1]])
# manipulator.set_furniture_pose(T_F_S)
T_A_S = np.array([[-1.0, 0.0, 0.0, 0.6], [0.0, -1.0, 0.0, -0.3], [0.0, 0.0, 1.0, 0.546/2], [0, 0, 0, 1]])
manipulator.set_door_model_params(T_A_S, 0.018, 0.3, 0.5, 0.0, -0.155, 1.0, 0.018, 0.005)
R_G_S_Y = rvlutil.roty(0.75 * np.pi)
R_G_S_Z = rvlutil.rotz(np.pi)
t_0_S_home = np.array([0.3 + 0.0823, 0.0, 0.5])
T_G_S_init = R_G_S_Y @ R_G_S_Z
T_G_S_init[:3,3] = t_0_S_home
manipulator.set_environment_state(11.0)
# print(T_G_S_init)
print('Path planning...')
T_G_0 = manipulator.path2(T_G_S_init)
print('completed.')
manipulator.set_environment_state(0)
T_DD_S = manipulator.get_T_DD_S()
print('T_DD_S')
print(T_DD_S)
T_F_S = manipulator.get_T_F_S()
print('T_F_S')
print(T_F_S)

