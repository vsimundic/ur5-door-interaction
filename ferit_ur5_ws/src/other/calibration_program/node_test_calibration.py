#! /bin/python3
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge
import aruco
import os
import traceback
from core.transforms import pose_to_matrix

from CameraReader import CameraReader
from ArucoDetector import ArucoDetector
from RobotComms import RobotComms

# from CameraCommands import CameraCommands
# from RobotCommands import RobotCommands
# from GripperCommands import GripperCommands
# from ImageCommands import ImageCommands
# from MarkerCommands import MarkerCommands

def print_help():
    print("Test calibration program commands are:")
    print("camera capture - capture image from camera and detect marker")
    print("robot pose - read tool pose from robot")
    print("help - prints this message")


if __name__ == '__main__':

    print('Testing robot-camera calibration program')

    rospy.init_node('node_test_calibration')

    image_topic = '/camera/rgb/image_raw'
    move_group_name = 'arm'
    # camera_params_path = os.path.join(os.path.dirname(__file__), "camera_parameters_asus.yml")
    camera_params_path = os.path.join(os.path.dirname(__file__), "camera_parameters_asus.yml")
    aruco_dict_path = os.path.join(os.path.dirname(__file__), "4x4_1000.dict")
    save_E_T_C_path = os.path.join(os.path.dirname(__file__), "T_C_T.npy")

    camReader = CameraReader(image_topic)
    arucoDetector = ArucoDetector(camera_params_path, aruco_dict_path)
    robotComms = RobotComms(move_group_name)

    T_C_T = np.load(save_E_T_C_path)
    print('TCT:\n', T_C_T)
    T_G_T = np.eye(4)
    # T_G_T[:3, 3] = np.array([0., 0., 0.31])
    # T_G_T[:3, 3] = np.array([0.0018644612449251205, 0.0010584130476441636, 0.30469780762847337])
    # T_G_T[:3, 3] = np.array([2.1034578724762145e-03, -6.9527709363085264e-04,  3.0698323278830231e-01])
    
    # TODO-1: Load 
    # T_G_T[:3, 3] = np.array([0., 0., 1.])
    T_G_T = np.load(os.path.join(os.path.dirname(__file__), 'T_G_T_pen.npy'))
    # np.save(os.path.join(os.path.dirname(__file__), 'T_G_T_pen.npy'), T_G_T)

    T_M_0_array = ''
    T_G_0 = ''
    while True:
        try:
            text = input('>>')
            if 'help' in text:
                print_help()
            elif 'camera capture' == text:
                image = camReader.lastimage.copy()
                markers = arucoDetector.detector.detect(image, arucoDetector.camparam, 0.15)
                T_M_0_array = np.zeros([len(markers), 4, 4])

                robot_pose_ = robotComms.pose()
                T_T_0_ = pose_to_matrix(robot_pose_)
                print('Num of markers detected: %d' % len(markers))
                # np.save('/home/RVLuser/ferit_ur5_ws/src/other/calibration_program/data/Exp-test_extrinsic_calibration-20240216/T_T_0.npy', T_T_0_)



                for i, marker in enumerate(markers):
                    T_M_C = marker.getTransformMatrix()
                    
                    T_M_0 = T_T_0_ @ T_C_T @ T_M_C

                    # np.save('/home/RVLuser/ferit_ur5_ws/src/other/calibration_program/data/Exp-test_extrinsic_calibration-20240216/T_M_C_%d.npy' % marker.id, T_M_C)
                    print(T_M_0)
                    T_M_0_array[i, : , :]  = T_M_0
                print(T_M_0_array)
                print(T_M_0_array.shape)
            elif 'robot pose' == text:
                robot_pose_ = robotComms.pose()
                T_T_0 = pose_to_matrix(robot_pose_)

                T_G_0 = T_T_0 @ T_G_T

                # np.save('/home/RVLuser/ferit_ur5_ws/src/other/calibration_program/data/test_20240215/T_G_0-7.npy', T_G_0)


                print(T_G_0)
            
            elif 'test' == text:
                x = T_M_0_array - T_G_0[np.newaxis, ...]
                print(x)

                # for i in range(T_M_0_array.shape[0]):
                #     T_M_0_ = T_M_0_array[i]
                    


        except Exception as error:
            traceback.print_exc()
