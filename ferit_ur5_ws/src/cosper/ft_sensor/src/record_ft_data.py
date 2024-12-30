#!/usr/bin/python

import numpy as np
import socket
import matplotlib.pyplot as plt
import datetime as dt
import matplotlib.animation as animation
import csv
from time import time 

fig, (ax1, ax2, ax3) = plt.subplots(3)
fig_f, (ax1_f, ax2_f, ax3_f) = plt.subplots(3)
xs = []
ys_x = []
ys_y = []
ys_z = []
# xs_f = []
ys_x_filtered = []
ys_y_filtered = []
ys_z_filtered = []

def get_data_from_ft_sensor(robot_ip):
    sensor_port = 63351
    output_file_location = '/home/RVLuser/ferit_ur5_ws/ft_sensor_data.csv'
    write_object = True
    socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    is_published = False


    try:
        # print('Connecting to UR5 at ' + robot_ip)
        socket_object.connect((robot_ip, sensor_port))

        if write_object is True:
            # print('File Write Location: ' + output_file_location)
            f = open(output_file_location, 'w')
                    
        try:
            # print('Writing in %s, Press Ctrl + Z to stop' % output_file_location)
            
            # while True:
            data = socket_object.recv(1024) # procita bajte
            
            bytes_unpacked_data = data.decode('utf-8')[1:-1]
            dataF = bytes_unpacked_data.split(" , ")
            
            data_list = [float(i) for i in dataF]
            # print(data_list)
            # print("\n")

            return data_list
            #force_torque_values = (self.socket_object.recv(1024).replace("(","*")).replace(")","*")
            
            #value_list = force_torque_values.split("*")
            
            
            #rospy.loginfo(value_list [1])
            ##rospy.loginfo(dataF)



            #self.publisher_object.publish(value_list[1])
            # self.publisher_object.publish(dataF)
            ##self.publisher_rate.sleep()
        

            #if self.write_object is True:       
            #    f.write(strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime()) + ": " +force_torque_values)
                                            
        except KeyboardInterrupt:
                    
            f.close()
            socket_object.close()

            return False

    except Exception as e:

        print("Error: No Connection! Please check your ethernet cable: " + str(e))

        return False

def animate(i):
    global xs, ys_x, ys_y, ys_z
    data_list = get_data_from_ft_sensor('192.168.10.14')
    n = 200

    if len(ys_x) >= n:
        # xs.pop(0)
        ys_x.pop(0)
        ys_y.pop(0)
        ys_z.pop(0)
    else:
        xs.append(len(ys_x))

    ys_x.append(data_list[0])
    ys_y.append(data_list[1])
    ys_z.append(data_list[2])

    ys1 = ys_x[-n:]
    ys2 = ys_y[-n:]
    ys3 = ys_z[-n:]

    ## Moving average
    if len(ys_x_filtered) >= n:
        ys_x_filtered.pop(0)
        ys_y_filtered.pop(0)
        ys_z_filtered.pop(0)
    
    m = 5

    if len(ys_x) > 0:
        ys_x_filtered.append(sum(ys_x[-m:]) / len(ys_x[-m:]))
        ys_y_filtered.append(sum(ys_y[-m:]) / len(ys_y[-m:]))
        ys_z_filtered.append(sum(ys_z[-m:]) / len(ys_z[-m:]))

    ys1_f = ys_x_filtered[-n:]
    ys2_f = ys_y_filtered[-n:]
    ys3_f = ys_z_filtered[-n:]
    
    ax1_f.clear()
    ax2_f.clear()
    ax3_f.clear()
    ax1_f.plot(xs, ys1_f)
    ax2_f.plot(xs, ys2_f)
    ax3_f.plot(xs, ys3_f)
    ax1_f.set_ylabel('Filtered force in x [N]')
    ax2_f.set_ylabel('Filtered force in y [N]')
    ax3_f.set_ylabel('Filtered force in z [N]')
    fig_f.supxlabel('Data points over 10 seconds')

    # Draw x and y lists
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax1.plot(xs, ys1)
    ax2.plot(xs, ys2)
    ax3.plot(xs, ys3)
    ax1.set_ylabel('Force in x [N]')
    ax2.set_ylabel('Force in y [N]')
    ax3.set_ylabel('Force in z [N]')
    fig.supxlabel('Data points over 10 seconds')


def record_to_file(robot_ip='192.168.10.14', file_location='/home/RVLuser/ferit_ur5_ws/ft_sensor_data.csv'):
    sensor_port = 63351
    socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    f = open(file_location, 'w')
    
    # open file
    writer = csv.writer(f, delimiter=',')
    writer.writerow(['x', 'y', 'z', 'Mx', 'My', 'Mz'])

    try:
        socket_object.connect((robot_ip, sensor_port))
        while True:
                data = socket_object.recv(1024)
                bytes_unpacked_data = data.decode('utf-8')[1:-1]
                dataF = bytes_unpacked_data.split(" , ")
                
                data_list = [float(i) for i in dataF]
                writer.writerow(data_list)
        
    except KeyboardInterrupt:
        socket_object.close()
        f.close()

if __name__ == '__main__':
    # ani = animation.FuncAnimation(fig, animate, interval=100)
    # ani2 = animation.FuncAnimation(fig_f, animate, interval=100)
    # plt.show()
    
    file_loc = '/home/RVLuser/ferit_ur5_ws/ft_sensor_data/unsuccessful_6.csv'
    record_to_file(file_location=file_loc)