#!/usr/bin/env python

import rospy
import numpy as np
from core.real_ur5_controller import UR5Controller  # Adjust import to your actual file structure

def main():
    rospy.init_node("ur5_test_controller_node")

    robot = UR5Controller()

    # Optional: Set a different force threshold (in Newtons)
    robot.force_threshold = 20.0

    current_joints = robot.get_current_joint_values()

    # Define a test joint trajectory
    joint_trajectory = np.array([
        current_joints,
        [-1.710107151662008, -1.1084988752948206, -1.8538802305804651, -0.18004209200014287, 1.708940029144287, 0.7857682108879089],
        [-1.6249944041561524, -1.7971465878518353, -1.252117500754386, -0.093408492478134, 1.623307999220554, 0.7856725229956512],
        [-1.710107151662008, -1.1084988752948206, -1.8538802305804651, -0.18004209200014287, 1.708940029144287, 0.7857682108879089],
        [-1.6249944041561524, -1.7971465878518353, -1.252117500754386, -0.093408492478134, 1.623307999220554, 0.7856725229956512]
    ])

    rospy.loginfo("Starting joint trajectory test...")

    # robot.zero_ft_sensor()

    success = robot.send_joint_trajectory_action(
        joint_points=joint_trajectory,
        max_velocity=0.5,
        max_acceleration=1.0,
        force_check_interval=0.02,
    )

    if success:
        rospy.loginfo("Trajectory completed successfully.")
    else:
        rospy.logwarn("Trajectory failed or was interrupted (possibly by force threshold).")

    robot.shutdown()

if __name__ == "__main__":
    main()