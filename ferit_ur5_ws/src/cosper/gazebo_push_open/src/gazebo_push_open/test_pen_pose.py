#!/usr/bin/python

import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
from core.ur5_commander import UR5Commander


if __name__ == '__main__':
	rospy.init_node('node_test_pen')

	save_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/pen_pose/'
	folder = 'pen_poses3'

	T_G_T = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/gazebo_push_open/config/T_G_T_pen.npy')
	T_G_T[:3, 3] = np.array([0., 0., 0.310])

	np.save(os.path.join(save_path, folder, 'T_G_T_pen.npy'), T_G_T)

	# T_RP_TCP = np.eye(4)
	# T_RP_TCP[:3, 3] = np.array([0.0775, 0., 0.097])
	# T_TCP_T = np.eye(4)
	# T_TCP_T[:3, :3] = rot_z(np.deg2rad(-45.0))
	# T_TCP_T[2, 3] = 0.175
	# T_RP_T = T_TCP_T @ T_RP_TCP

	# Robot handler
	robot = UR5Commander()
	T_T_0_list = []
	i = 0

	load = True

	if not load:
		while True:
			
			key = input('p for pose, q for exit: ')

			if key == 'p':
				T_T_0_ = robot.get_current_tool_pose()
				T_T_0_list.append(T_T_0_)
				np.save(os.path.join(save_path, folder, 'T_T_0_pen_%d' % i), T_T_0_)
				i += 1
			elif key == 'q':
				break
	else:
		T_T_0_list = [np.load(os.path.join(save_path, folder, f)) for f in os.listdir(os.path.join(save_path, folder)) if os.path.isfile(os.path.join(save_path, folder, f))]

	T_G_0_list = [T_T_0_ @ T_G_T for T_T_0_ in T_T_0_list]
	T_G_0_list.pop(19)


	# first analysis
	T_G_0_ref = T_G_0_list[0]
	T_G_0_rest = T_G_0_list[1:]

	t_G_0_res = [T_G_0_ref[:3, 3] - T_G_0_[:3, 3] for T_G_0_ in T_G_0_rest]


	for t in t_G_0_res:
		print(t)
	print('\n')

	# second analysis
	t_G_0_avg = np.array([0., 0., 0.])
	for T_ in T_G_0_list:
		t_G_0_avg += T_[:3, 3]
	
	t_G_0_avg /= len(T_G_0_list)
	print('\n')
	print(t_G_0_avg)
	print('\n')
	t_G_0_res2 = [t_G_0_avg - T_G_0_[:3, 3] for T_G_0_ in T_G_0_list]

	t_G_0_res2_array = np.stack(t_G_0_res2, axis=0)
	
	err = np.linalg.norm(t_G_0_res2_array, 1)

	print('error:')
	print(err)

	# for t in t_G_0_res2:
	# 	print(t)

	T_G_0_array = np.stack(T_G_0_list, axis=0)
	T_G_0_array.shape
	# plot points
	fig = plt.figure()
	ax = fig.add_subplot(projection='3d')

	ax.scatter(T_G_0_array[0:22, 0, 3], T_G_0_array[0:22, 1, 3], T_G_0_array[0:22, 2, 3])
	plt.show()

	print(T_G_0_array[:, 2, 3])