#!/usr/bin/env python

import socket
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse


class RobotiqFTStreamer:
    def __init__(self):
        self.robot_ip = rospy.get_param("~robot_ip", "192.168.10.14")
        self.sensor_port = rospy.get_param("~sensor_port", 63351)
        self.pub = rospy.Publisher("/robotiq_ft_sensor/wrench", WrenchStamped, queue_size=10)
        self.zero_offset = np.zeros(6)
        self.sock = None
        self.buffer = ""

        self.connect()
        self.zero_sensor()
        self.zero_srv = rospy.Service("~zero", Trigger, self.zero_service_callback)

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        rospy.loginfo("Connecting to Robotiq FT300s at %s:%d", self.robot_ip, self.sensor_port)
        self.sock.connect((self.robot_ip, self.sensor_port))
        rospy.loginfo("Connected to Robotiq FT300s")

    def zero_sensor(self, samples=100, delay=0.01):
        rospy.loginfo("Zeroing force-torque sensor...")
        readings = []
        while len(readings) < samples and not rospy.is_shutdown():
            try:
                data = self.sock.recv(1024).decode("utf-8")
                self.buffer += data

                while "(" in self.buffer and ")" in self.buffer:
                    start = self.buffer.find("(")
                    end = self.buffer.find(")", start)
                    raw = self.buffer[start + 1:end]
                    self.buffer = self.buffer[end + 1:]

                    parts = [float(val.strip()) for val in raw.split(" , ")]
                    if len(parts) == 6:
                        readings.append(parts)
            except Exception as e:
                rospy.logwarn("Zeroing read failed: %s", str(e))
            rospy.sleep(delay)

        if readings:
            self.zero_offset = np.mean(readings, axis=0)
            rospy.loginfo("Zero offset set to: %s", self.zero_offset)
        else:
            self.zero_offset = np.zeros(6)
            rospy.logwarn("Zeroing failed — no valid samples")

    def zero_service_callback(self, req):
        self.zero_sensor()
        return TriggerResponse(success=True, message="Sensor zeroed")

    def read_loop(self):
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            try:
                data = self.sock.recv(1024).decode("utf-8")
                self.buffer += data

                while "(" in self.buffer and ")" in self.buffer:
                    start = self.buffer.find("(")
                    end = self.buffer.find(")", start)
                    raw = self.buffer[start + 1:end]
                    self.buffer = self.buffer[end + 1:]

                    parts = [float(val.strip()) for val in raw.split(" , ")]
                    if len(parts) != 6:
                        continue

                    values = np.array(parts) - self.zero_offset

                    msg = WrenchStamped()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "tool0"
                    msg.wrench.force.x = values[0]
                    msg.wrench.force.y = values[1]
                    msg.wrench.force.z = values[2]
                    msg.wrench.torque.x = values[3]
                    msg.wrench.torque.y = values[4]
                    msg.wrench.torque.z = values[5]

                    self.pub.publish(msg)

            except socket.timeout:
                rospy.logwarn_throttle(5, "FT sensor read timeout")
            except Exception as e:
                rospy.logerr_throttle(5, "FT sensor read error: %s", str(e))
            rate.sleep()

    def close(self):
        if self.sock:
            self.sock.close()
            rospy.loginfo("Disconnected from Robotiq FT300s")


def main():
    rospy.init_node("robotiq_ft_streamer")
    streamer = RobotiqFTStreamer()
    try:
        streamer.read_loop()
    finally:
        streamer.close()


if __name__ == "__main__":
    main()