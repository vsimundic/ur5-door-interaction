#!/usr/bin/env python3

import moveit_commander, geometry_msgs.msg, moveit_msgs.msg, rospy, sys
import roslib
from time import time
from trac_ik_python.trac_ik import IK; 
import sys
import socket
import time

import ast
import h5py

sys.path.append("..")
import logging

# import rtde.rtde as rtde
# import rtde.rtde_config as rtde_config


import rtde_receive
from scipy.spatial.transform import Rotation
import numpy as np
import math 

from tf.transformations import quaternion_from_euler,quaternion_matrix, quaternion_from_matrix, euler_from_matrix
import tf

roslib.load_manifest('robotiq_3f_gripper_control')

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

class RobotControl:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)   
        # INVERSE KINEMATICS
        self.ik = IK('base_link', 'tool0', solve_type="Distance"); 



    def setPose(self, x, y, z, quaternion):
        x = float(x)
        y = float(y)
        z = float(z)
        qx, qy, qz, qw = quaternion

        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)
        
        self.move_group.go(goal_joints, wait=True)
        self.move_group.stop()


    def openGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 0
        
        # 3 second delay to let the gripper open
        start_time = time.time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)
            end_time = time.time()
            if float(end_time - start_time) >= 3.0: # 3s
                break

    def closeGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1     # activation
        command.rGTO = 1     # request position
        command.rSPA = 255   # closing speed
        command.rFRA = 200   # force
        command.rATR = 0     # automatic opening if the force is reached (normal = 0)
        command.rMOD = 1     # gripper mode -> 0 = normal, 1 = pinch, 2 = wide, 3 = scissors (needs checking)
        command.rPRA = 200   # 0 = open, 255 = close
        
        # 3 second delay to let the gripper close
        start_time = time.time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)
            end_time = time.time()
            if float(end_time - start_time) >= 3.0: # 3s
                break
    

    def executeTrajectory(self, plan):
        self.move_group.execute(plan, wait=True)


    # def getTCPvalues(self):
    #     #BASE VIEW
    #     ROBOT_HOST = "192.168.22.14" #localhost
    #     ROBOT_PORT = 30004
    #     config_filename = "/home/lukrecia/catkin_ws/src/rtde_examples/RTDE_Python_Client_Library-2.7.2/examples/control_loop_configuration.xml"

    #     keep_running = True

    #     logging.getLogger().setLevel(logging.INFO)

    #     conf = rtde_config.ConfigFile(config_filename)
    #     state_names, state_types = conf.get_recipe("state")
    #     setp_names, setp_types = conf.get_recipe("setp")
    #     watchdog_names, watchdog_types = conf.get_recipe("watchdog")

    #     con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
    #     con.connect()
    #     # get controller version
    #     con.get_controller_version()
    #     # setup recipes
    #     con.send_output_setup(state_names, state_types)
    #     setp = con.send_input_setup(setp_names, setp_types)
    #     watchdog = con.send_input_setup(watchdog_names, watchdog_types)

    #     # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
    #     watchdog.input_int_register_0 = 0

    #     # start data synchronization
    #     if not con.send_start():
    #         sys.exit()

    #     # control loop
    #     move_completed = True
    #     # receive the current state
    #     state = con.receive()

    #     if state is not None:
    #         pose = state.actual_TCP_pose
    #         joints = state.actual_q
    #         print(state.actual_TCP_pose)
    #         print(state.actual_q)

    #     # kick watchdog
    #     con.send(watchdog)
    #     con.send_pause()
    #     con.disconnect()

    #     return pose, joints


def sendURcommand1(pose, joints):
    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("Start connected, ", s)
    s.sendall(("movej(get_inverse_kin(p"+ str(pose)+",qnear= "+str(joints) + "),a=1.4,v=0.2)"+"\n").encode('utf-8'))
    print("End start pose")
    data=s.recv(1024)
    s.close()
    time.sleep(10)



def sendURcommand(pose):
    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("Start connected, ", s)
    s.sendall(("movej(get_inverse_kin(p"+ str(pose)+ "),a=1.4,v=0.2)"+"\n").encode('utf-8'))
    print("End start pose")
    data=s.recv(1024)
    s.close()
    time.sleep(10)


def zeroSensor():
    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    time.sleep(1)
    s.sendall(("zero_ftsensor()" + "\n").encode('utf-8')) 
    time.sleep(2)
    data=s.recv(1024)
    s.close()


def writeToTextFile(positions,joints):
    n=len(positions)
    with open("/home/RVLuser/ur5_ws/src/ao_manipulation/scripts/URscript.txt", "w") as f:
        f.write("def door_open():\n")
        f.write("force_mode(tool_pose(), [1, 1, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.2, 0.2, 0.1, 0.60, 0.60, 0.35])\n")
        for i in range(0,n):
            f.write("movej(get_inverse_kin(p"+ str(positions[i]) +"),a=1.4,v=0.2)\n")
        f.write("end\n")
        f.write("door_open()\n")
        f.close()

def sendURscript():
    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    f = open("/home/RVLuser/ur5_ws/src/ao_manipulation/scripts/URscript.txt", "rb")
    l = f.read()
    s.sendall(l)
 
    print("Ended sending urscript!")
    data=s.recv(1024)
    s.close()


def get_next_door_pose(current_pose, door_width, angle_deg):
    TTB = current_pose.copy()
    print("TTB: \n",TTB)
    RTB = TTB[0:3, 0:3].copy()

    b = 0.28
    a = door_width
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    Rz = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    TA_A = np.identity(4)
    TA_A[0:3, 0:3] = Rz.copy()

    TGA = np.matrix([[0, 0, 1, 0],
                     [-1, 0, 0, -a],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])

    RGB = np.matrix([[-1, 0, 0],
                    [0, 0, -1],
                    [0, -1, 0]])
    # if(flag==1):
    #     RGT = np.multiply(np.linalg.inv(RTB), RGB)
    #     RGT = RTB.T @ RGB
    # else:
    #     RGT=RGT_prev.copy()

    # print("RTB: ", RTB)

    # c, s = np.cos(np.radians(45)), np.sin(np.radians(45))
    # Rz = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    # RGT = Rz.copy()

    # TGT = np.identity(4)
    # TGT[0:3, 0:3] = RGT.copy()
    # TGT[0:3, 3] = np.array([0, 0, b])
    hf = h5py.File('/home/RVLuser/ur5_ws/src/ao_manipulation/scripts/TGT.h5', 'r')
    n1 = hf.get('dataset_1')
    TGT = np.array(n1)
    print("TGT:\n ",TGT)
    print("\nTGT:\n", TGT)

    TT_B = TTB @ TGT @ np.linalg.inv(TGA) @ TA_A @ TGA @ np.linalg.inv(TGT)
    TT_B[2,3] = current_pose[2, 3]
    print("\nTT_B:\n", TT_B)

    return TT_B


def get_pose_vector_from_transformation_matrix(mat):
    pose_vec = 6*[0]
    #rot_obj_temp = Rotation.from_matrix(mat[0:3, 0:3])
    pose_vec[0:3] = np.asarray(mat[0:3, 3]).reshape(-1)

    th = np.arccos((np.trace(mat[0:3,0:3])-1)/2)

    u = 1/ (2*np.sin(th)) * np.array([mat[2,1] -  mat[1,2], mat[0,2] -  mat[2,0],
                                        mat[1,0] -  mat[0,1] ])
    
    #pose_vec[3:] = rot_obj_temp.as_euler('xyz').copy()
    pose_vec[3:]  = th*u


    return pose_vec


def get_transformation_matrix_from_pose_vector(vec):
    mat = np.identity(4)
    euler_angles = vec[3:].copy()
    #rot_obj_temp = Rotation.from_euler('xyz', euler_angles)
    #mat[0:3, 0:3] = np.matrix(rot_obj_temp.as_matrix()).copy()
    th = np.linalg.norm(euler_angles)
    if np.abs(th) > 1e-6:
        k = euler_angles / th
        cs = np.cos(th)
        sn = np.sin(th)
        a = 1.0 - cs
        rot_obj_temp = np.array([[k[0]*k[0]*a + cs, k[1]*k[0]*a-k[2]*sn, k[2]*k[0]*a+k[1]*sn], 
                        [k[0]*k[1]*a+k[2]*sn, k[1]*k[1]*a + cs, k[2]*k[1]*a-k[0]*sn], 
                        [k[0]*k[2]*a-k[1]*sn, k[1]*k[2]*a+k[0]*sn, k[2]*k[2]*a + cs]])
    else:
        rot_obj_temp = np.identity(3)
    print("provjera rot. mat.: ", rot_obj_temp.T @ rot_obj_temp)
    mat[0:3, 0:3] = rot_obj_temp.copy()
    mat[0:3, 3] = vec[0:3].copy()
    return mat


def sample_door_opening_points_and_joints(RC, start_pose_vec, start_joints, door_width, angle_deg, num_points):
    
    c, s = np.cos(np.pi), np.sin(np.pi)
    Rz = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    TRz = np.identity(4)
    TRz[0:3, 0:3] = Rz.copy()


    TTB = get_transformation_matrix_from_pose_vector(start_pose_vec)

    poses = []
    joints = []
    prev_poseTB = TTB
    prev_joints = start_joints
    for i in range(0, num_points):

        TT_B = get_next_door_pose(prev_poseTB, door_width, angle_deg)

        # Rotate 180° about z-axis for ROS 
        TT_B_rot = TRz @ TT_B
        pose_vecT_B = get_pose_vector_from_transformation_matrix(TT_B)
        pose_vecT_B_rot = get_pose_vector_from_transformation_matrix(TT_B_rot)
        qx, qy, qz, qw = quaternion_from_matrix(TT_B_rot)

        current_joints = None
        while (current_joints is None):
            current_joints = RC.ik.get_ik(prev_joints, 
                                          pose_vecT_B_rot[0], 
                                          pose_vecT_B_rot[1], 
                                          pose_vecT_B_rot[2],
                                          qx, qy, qz, qw)

    

        
        current_joints = list(current_joints)

        poses.append(pose_vecT_B)  # Save the non-rotated point
        joints.append(current_joints)

        prev_poseTB = get_transformation_matrix_from_pose_vector(pose_vecT_B).copy()
        prev_joints = current_joints.copy()
    
    return poses, joints

def extract_object_information(string_description):
    split_str=string_description.split("(")
    # for substring in split_str:
    #     print(substring)
    extracted_substrings = [substring.split(")")[0] for substring in split_str]
    # print("________________")
    string_data=[]
    for substring in extracted_substrings:
        s=substring.rsplit(",",1)
        # print(s[0])
        string_data.append(s[0])

    extracted_matrix=[]
    string_data=string_data[1:]
    # print(string_data)
    for matrix in string_data:

        m = ast.literal_eval(matrix)

    # Convert the matrix to a NumPy array
        numpy_matrix = np.array(m)


        # Print the NumPy matrix
        # print(numpy_matrix)
        extracted_matrix.append(numpy_matrix)
    # print(extracted_matrix)
    R=extracted_matrix[0]
    t=extracted_matrix[1]
    s=extracted_matrix[2]
    r=extracted_matrix[3]

    print("R:\n",extracted_matrix[0])
    print("t:\n",extracted_matrix[1])
    print("s:\n",extracted_matrix[2])
    print("r:\n",extracted_matrix[3])

    TAC = np.append(R, t.reshape(-1, 1), axis=1)
    TAC = np.append(TAC, [[0,0,0,1]], axis=0)
    print("TAC:\n",TAC)

    return TAC, s, r
    


def calculate_starting_position():
    print("Calculating start position")
    b = 0.28
    text_file=open("/home/RVLuser/ur5_ws/src/ao_manipulation/models_2/door_model_3.txt","r")
    ao=text_file.read()
    text_file.close()


    TAC, s, r= extract_object_information(ao)
    #Lukina matrica - stara
    # TCT=np.array([[-0.9854697585486856, 0.04544950427596955, 0.04161469708832181, -0.008414572098302112],
    #             [-0.0473755719793784,  -0.9851878558865472, -0.04591872450301393,  0.1781730018968248],
    #             [0.039408065574332476, -0.04782589927461461, 0.9854480063366874, 0.0020840323750682677],
    #             [ 0, 0, 0, 1]])

    TCT = np.array([[-0.9786901906426425,    0.05770529828634073,   0.03676758168288247, -0.01078821015709552 ],
    [-0.0593574405847809,   -0.9782635664950653,   -0.044646774674482546, 0.18152850107216661 ],
    [ 0.034036021511733676, -0.04676257896325065,   0.9793727871559502, -0.001666384428069767],
    [ 0. ,                   0.,                    0. ,                   1.  ]])
    print("r_novi: ",r)
    x_GA = r[0] - 0.033
    # y_GA = s[0] - 0.0319
    y_GA = r[1] - s[0]/2. + 0.027 + 0.0075
    z_GA = s[1]/2. - 0.037 - 0.155/2.
    t_AG = np.array([x_GA,-y_GA,z_GA])

    TGA = np.array([ [0, 0, 1, x_GA],
                     [-1, 0, 0,y_GA],
                     [0, -1, 0, z_GA],
                     [0, 0, 0, 1]])
    
    print("TGA:\n",TGA)
    
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.22.14")
    # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    tcp_position = rtde_r.getActualTCPPose()
    joints=rtde_r.getActualQ()
    print("TCP position: ",tcp_position)
    print("Q: ",joints)
    
    TTB=get_transformation_matrix_from_pose_vector(tcp_position)

    RTB = TTB[0:3, 0:3].copy()
    
    hf = h5py.File('/home/RVLuser/ur5_ws/src/ao_manipulation/scripts/TGT.h5', 'r')
    n1 = hf.get('dataset_1')
    TGT = np.array(n1)
    print("TGT:\n ",TGT)

    TT_B = TTB @ TCT @ TAC @ TGA @ np.linalg.inv(TGT)
    print("TT_B:\n", TT_B)
    #TGT i TCT - problem
    start_pose=get_pose_vector_from_transformation_matrix(TT_B)
    print("Start pose: ",start_pose)
    return start_pose

    #TGT - treba li isto kao i racunanje koraka? - tako je napravljeno


if __name__ == '__main__':
    

    # rtde_r = rtde_receive.RTDEReceiveInterface("192.168.22.14")
    
    # tcp_position = rtde_r.getActualTCPPose()

    # TTB=get_transformation_matrix_from_pose_vector(tcp_position)

    # RTB = TTB[0:3, 0:3].copy()

    # RGB = np.matrix([[-1, 0, 0],
    #                 [0, 0, -1],
    #                 [0, -1, 0]])

    # RGT = np.linalg.inv(RTB) @ RGB

    # TGT = np.identity(4)
    # TGT[0:3, 0:3] = RGT.copy()
    # TGT[0:3, 3] = np.array([0, 0, 0.28])

    # print(TGT)

    # hf = h5py.File('/home/RVLuser/ur5_ws/src/ao_manipulation/scripts/TGT.h5', 'w')

    # hf.create_dataset('dataset_1', data=TGT)

    # hf.close()

    #poz za snimanje modela 3
    # tcp [-0.100073316635339, 0.036553495548184656, 0.40403819297364696, 1.8494595136390326, -0.03014619478780129, -0.14062608169778434]
    #q Q:  [-1.7487309614764612, 0.010847210884094238, -2.4592676798449915, -0.9709461371051233, 1.6296322345733643, 3.042423725128174]

    start_pose=[-0.100073316635339, 0.036553495548184656, 0.40403819297364696, 1.8494595136390326, -0.03014619478780129, -0.14062608169778434]
    joints=[-1.7487309614764612, 0.010847210884094238, -2.4592676798449915, -0.9709461371051233, 1.6296322345733643, 3.042423725128174]
    
    sendURcommand1(start_pose,joints)

    time.sleep(10)


    # RC = RobotControl()

    # # start_pose, start_joints = RC.getTCPvalues()
    
    # # # Hardcoded point and joints
    # # start_pose = [-0.319374397217617, -0.5295626977479386, 0.3861139450603652, 0.8121580324907804, 1.7533175779683356, -1.808949857338796]
    # # start_joints = [0.7558420896530151, -1.145426098500387, 1.5590248107910156, 2.728001117706299, -0.8367646376239222, 0.7906367778778076]
    # # start_pose = [-0.3699745933297053, -0.5186108669590446, 0.3980670206407025, 1.4574027968490066, -0.5868335536521441, 0.6285672381695145]
    # start_joints = [4.2015700340271, -1.9716318289386194, -1.1473305861102503, 3.1599974632263184, -2.0953381697284144, 0.8027371168136597]


    # start_pose=calculate_starting_position()

    # RC.openGripper()
    # time.sleep(1)

    # # # Set the robot in the start pose
    # sendURcommand(start_pose)
    # time.sleep(10)
    # # # FT sensor zeroing
    # zeroSensor()
    # time.sleep(1)
    
    # # # Close gripper
    # RC.closeGripper()
    # time.sleep(2)

    # poses, joints = sample_door_opening_points_and_joints(RC, start_pose, start_joints, door_width=0.35, angle_deg=-10, num_points=4)
    # # print("Pozes: \n",poses)
    # # print("Joints:\n",joints)

    # # # Write points and joints to a txt file
    # writeToTextFile(poses, joints)

    # # # Start manipulation with force mode
    # sendURscript()

    # time.sleep(15)
    # RC.openGripper()