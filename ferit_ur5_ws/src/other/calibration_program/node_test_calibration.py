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
import readline

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
    print("quit - Exit the program")
    print("camera capture - capture image from camera and detect marker")
    print("robot pose - read tool pose from robot")
    print("help - prints this message")


if __name__ == '__main__':
    print('Testing robot-camera calibration program')
    rospy.init_node('node_test_calibration')
    image_topic = '/camera/color/image_raw'
    move_group_name = 'arm'
    # camera_params_path = os.path.join(os.path.dirname(__file__), "camera_parameters_asus.yml")
    camera_params_path = os.path.join(os.path.dirname(__file__), "rgb_IntelRealSense_L550_params.yaml")
    aruco_dict_path = os.path.join(os.path.dirname(__file__), "4x4_1000.dict")
    save_E_T_C_path = os.path.join(os.path.dirname(__file__), "T_C_T.npy")
    camReader = CameraReader(image_topic)
    arucoDetector = ArucoDetector(camera_params_path, aruco_dict_path)
    robotComms = RobotComms(move_group_name)

    T_C_T = np.load(save_E_T_C_path)
    print('TCT:\n', T_C_T)
    T_G_T = np.load(os.path.join(os.path.dirname(__file__), 'T_G_T_pen.npy'))
    T_M_0_array = ''
    T_G_0 = ''
    while True:
        try:
            text = input('>>')
            if 'help' in text:
                print_help()
            if("quit" in text):
                break
            elif 'camera capture' == text:
                image = camReader.lastimage.copy()
                markers = arucoDetector.detector.detect(image, arucoDetector.camparam, 0.15)
                T_M_0_array = np.zeros([len(markers), 4, 4])
                robot_pose_ = robotComms.pose()
                T_T_0_ = pose_to_matrix(robot_pose_)
                print('Num of markers detected: %d' % len(markers))
                for i, marker in enumerate(markers):
                    T_M_C = marker.getTransformMatrix()
                    T_M_0 = T_T_0_ @ T_C_T @ T_M_C
                    print(T_M_0)
                    T_M_0_array[i, : , :]  = T_M_0
                print(T_M_0_array)
                print(T_M_0_array.shape)
            elif 'robot pose' == text:
                robot_pose_ = robotComms.pose()
                T_T_0 = pose_to_matrix(robot_pose_)
                T_G_0 = T_T_0 @ T_G_T
                print(T_G_0)
            elif 'test' == text:
                # Save dists to the 
                dists = np.zeros((len(markers),))
                for i in range(len(markers)):
                    dists[i] = np.linalg.norm(T_M_0_array[i, :3, 3] - T_G_0[:3, 3])
                min_idx = dists.argmin(axis=0) # get minimum index
                print('Marker ID: ', str(min_idx+1))
                print('Distance to marker: %.4f' % np.linalg.norm(T_M_0_array[min_idx, :3, 3] - T_G_0[:3, 3]))
                print('Position differences [x, y, z]: ' + np.array2string(T_M_0_array[min_idx, :3, 3] - T_G_0[:3, 3], separator=','))
        except Exception as error:
            traceback.print_exc()
