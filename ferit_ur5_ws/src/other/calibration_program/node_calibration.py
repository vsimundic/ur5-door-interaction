#! /bin/python3

import rospy
import readline
import traceback
import os
import numpy as np

from CameraReader import CameraReader
from ArucoDetector import ArucoDetector
from RobotComms import RobotComms

from CameraCommands import CameraCommands
from RobotCommands import RobotCommands
from GripperCommands import GripperCommands
from ImageCommands import ImageCommands
from MarkerCommands import MarkerCommands

import izracun


def printHelp():
    print("calibration program commands are:")

    print("camera - commands used to show current image from camera")
    print("robot - commands used to move robot and read position from robot")
    print("gripper - commands use to control gripper")
    print("image - commands used to capture images for calibration")
    print("marker - commands used to capture marker positions for calibration")
    print("calculate - perform calculation")
    print("quit - Exit the program")
    print("q - Exit the program")
    print("help - prints this message")

if __name__ == '__main__':
    print("Running robot-camera calibration program")
    print("use commands camera, robot and gripper to work with specified component")
    print("use commands image and marker to capture values for calculation")
    print("after values are in memory use calculate command to perform calcluation")
    print("use quit or q to exit program")
    
    np.set_printoptions(precision=20, linewidth=100)
    
    rospy.init_node("calibration_program", anonymous=True)

    # image_topic = '/camera/color/image_raw'
    image_topic = '/camera/color/image_raw'
    move_group_name = 'arm'
    # camera_params_path = os.path.join(os.path.dirname(__file__), "camera_parameters_asus.yml")
    camera_params_path = os.path.join(os.path.dirname(__file__), "rgb_IntelRealSense_L550_params.yaml")
    aruco_dict_path = os.path.join(os.path.dirname(__file__), "4x4_1000.dict")
    save_E_T_C_path = os.path.join(os.path.dirname(__file__), "T_C_T.npy")

    camerareader = CameraReader(image_topic)
    arucoDetector = ArucoDetector(camera_params_path, aruco_dict_path)
    robotComms = RobotComms(move_group_name)


    # TODO-1: set the position of the calibration pen w.r.t. robot flange (last float is a fixed 1.0)
    tool_E = np.array([0, 0, 0.3154, 1.0])
    

    # TODO-2: Set the marker size (in meters)
    marker_size = 0.15

    cameraCommands = CameraCommands(camerareader, arucoDetector, robotComms, marker_size=marker_size)
    robotCommands = RobotCommands(robotComms)
    gripperCommands = GripperCommands()

    imageCommands = ImageCommands(camerareader, arucoDetector, robotComms, marker_size=marker_size)
    markerCommands = MarkerCommands(robotComms)
    
    while True:
        try:
            text = input(">>")
            input_split = text.split(' ', 1)
            command = input_split[0]
            rest = ""
            if(len(input_split) == 2):
                rest = input_split[1]
            if(command == "quit" or command == "q"):
                break
            elif(command == "camera"):
                cameraCommands.eval(rest)
            elif(command == "robot"):
                robotCommands.eval(rest)
            elif(command == "gripper"):
                gripperCommands.eval(rest)
            elif(command == "image"):
                imageCommands.eval(rest)
            elif(command == "marker"):
                markerCommands.eval(rest)
            elif(command == "calculate"):
                t_E_T = tool_E
                #calculate(tool_E, markerCommands.markers, imageCommands.images)
                num_iterations = 200
                E_T_C, tool_E = izracun.calculate2(tool_E, markerCommands.markers, imageCommands.images, num_iterations)
                np.save(save_E_T_C_path, E_T_C)
                print("E_T_C")
                print(E_T_C)
                print("tool_E")
                print(tool_E)
                T_G_T_pen = np.eye(4)
                T_G_T_pen[:3, 3] = tool_E[0:3].copy()
                np.save(os.path.join(os.path.dirname(__file__), 'T_G_T_pen.npy'), T_G_T_pen)
                imageCommands.E_T_C = E_T_C
                imageCommands.tool_E = tool_E
                imageCommands.markers = markerCommands.markers
                cameraCommands.E_T_C = E_T_C
                cameraCommands.tool_E = tool_E
            elif(command == "help"):
                printHelp()
            else:
                print("Unknown command!")
                printHelp()
        except Exception as error:
            traceback.print_exc()
