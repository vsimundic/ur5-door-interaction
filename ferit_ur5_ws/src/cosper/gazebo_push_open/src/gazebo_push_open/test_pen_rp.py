#!/usr/bin/python

import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
from core.ur5_commander import UR5Commander

def rot_z(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    
    return R

if __name__ == '__main__':
	rospy.init_node('node_test_pen_rp')

	save_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/pen_rp/'
	folder = 'pen_rps'

	T_G_T = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_G_T_pen.npy')
	T_G_T[:3, 3] = np.array([0., 0., 0.310])

	# np.save(os.path.join(save_path, folder, 'T_G_T_pen.npy'), T_G_T)

	T_RP_TCP = np.eye(4)
	T_RP_TCP[:3, 3] = np.array([0.0775, 0., 0.097])
	T_TCP_T = np.eye(4)
	T_TCP_T[:3, :3] = rot_z(np.deg2rad(-45.0))
	T_TCP_T[2, 3] = 0.175
	T_RP_T = T_TCP_T @ T_RP_TCP

	# Robot handler
	robot = UR5Commander()
	T_T_0_list = []
	i = 0

	load = True

	if not load:
		while True:
			key = input('p for pose, q for exit: ')

			if key == 'p':
				T_T_0_pen = robot.get_current_tool_pose()
				np.save(os.path.join(save_path, folder, 'T_T_0_pen.npy'), T_T_0_)
			if key == 'r':
				T_T_0_ = robot.get_current_tool_pose()
				T_T_0_list.append(T_T_0_)
				np.save(os.path.join(save_path, folder, 'T_T_0_rp_%d.npy' % i), T_T_0_)
				i += 1			
			elif key == 'q':
				break
	else:
		T_T_0_pen = np.load(os.path.join(save_path, folder, 'T_T_0_pen.npy'))
		T_T_0_list = [np.load(os.path.join(save_path, folder, f)) for f in os.listdir(os.path.join(save_path, folder)) if os.path.isfile(os.path.join(save_path, folder, f))]

	T_G_0 = T_T_0_pen @ T_G_T

	T_RP_0_list = [T_T_0_ @ T_RP_T for T_T_0_ in T_T_0_list]

	# first analysis
	t_diff = [T_G_0[:3, 3] - T_RP_0_[:3, 3] for T_RP_0_ in T_RP_0_list]

	for t in t_diff:
		print(t)
	print('\n')

	error = np.linalg.norm(t_diff, axis=1)
	print('error:')
	print(error)

	# # second analysis
	# t_G_0_avg = np.array([0., 0., 0.])
	# for T_ in T_G_0_list:
	# 	t_G_0_avg += T_[:3, 3]
	
	# t_G_0_avg /= len(T_G_0_list)
	# print('\n')
	# print(t_G_0_avg)
	# print('\n')
	# t_G_0_res2 = [t_G_0_avg - T_G_0_[:3, 3] for T_G_0_ in T_G_0_list]

	# t_G_0_res2_array = np.stack(t_G_0_res2, axis=0)
	
	# err = np.linalg.norm(t_G_0_res2_array, 1)

	# print('error:')
	# print(err)

	# # for t in t_G_0_res2:
	# # 	print(t)

	# T_G_0_array = np.stack(T_G_0_list, axis=0)
	# T_G_0_array.shape
	# # plot points
	# fig = plt.figure()
	# ax = fig.add_subplot(projection='3d')

	# ax.scatter(T_G_0_array[0:22, 0, 3], T_G_0_array[0:22, 1, 3], T_G_0_array[0:22, 2, 3])
	# plt.show()

	# print(T_G_0_array[:, 2, 3])