#!/usr/bin/env python

import rospy
import numpy as np
import os
import yaml
import sys
import RVLPYDDManipulator as rvlpy_dd_man
from numpy.core._exceptions import _ArrayMemoryError

from path_planning.srv import PlanMultiContactPath, PlanMultiContactPathResponse

class RVLPathPlanningService:
    def __init__(self, config: dict):
        rospy.init_node("plan_multi_contact_path_service")
        
        # Load RVL config (adjust path loading logic based on your setup)
        self.rvl_config = config.get("rvl_config_path", "/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec.cfg")
        if "rvl_config_path" in config:
            self.rvl_config = config["rvl_config_path"]
            
        if not os.path.exists(self.rvl_config):
            rospy.logerr(f"RVL config not found: {self.rvl_config}")

        self.srv = rospy.Service('plan_multi_contact_path', PlanMultiContactPath, self.handle_plan_path)
        rospy.loginfo("RVL Path Planning Service Ready.")

    def handle_plan_path(self, req: PlanMultiContactPath):
        rospy.loginfo("Received RVL path planning request...")
        
        # Convert flat arrays to numpy matrices
        T_R_W = np.array(req.T_R_W).reshape(4, 4)
        T_A_W = np.array(req.T_A_S).reshape(4, 4)
        q_init = np.array(req.q_init)

        # Setup Path Planner
        path_planner = rvlpy_dd_man.PYDDManipulator()
        path_planner.create(self.rvl_config)
        
        # Print cabinet info for debugging
        rospy.loginfo(f"Door dimensions (d,w,h): ({req.d_door}, {req.w_door}, {req.h_door})")
        rospy.loginfo(f"Door rotation (rx, ry): ({req.rx}, {req.ry})")
        rospy.loginfo(f"Axis position: {req.axis_pos}")
        rospy.loginfo(f"Static side width: {req.static_side_width}")
        rospy.loginfo(f"Moving to static part distance: {req.moving_to_static_part_distance}")
        # Print pose info
        rospy.loginfo(f"Door pose (T_A_W):\n{T_A_W}")

        path_planner.set_robot_pose(T_R_W)
        path_planner.set_door_model_params(
            req.d_door, 
            req.w_door, 
            req.h_door, 
            req.rx, 
            req.ry, 
            req.axis_pos, 
            req.static_side_width, 
            req.moving_to_static_part_distance
        )
        path_planner.set_door_pose(T_A_W)
        path_planner.set_environment_state(req.start_angle)

        success = False
        flat_q = []
        num_points = 0

        # Run Planning
        try:
            T_G_0_array, q = path_planner.path2(q_init, req.target_angle, req.num_states, False)
            
            if T_G_0_array.shape[0] > 1:
                success = True
                num_points = q.shape[0]
                
                # Apply UR5 specific unwrap adjustments
                q[:, 0] -= np.pi
                q[:, 5] -= np.pi
                q[q > np.pi] -= (2.0 * np.pi)
                q[q < -np.pi] += (2.0 * np.pi)
                q[1:] = np.unwrap(q[1:], axis=0)

                flat_q = q.flatten().tolist()
                rospy.loginfo(f"Path successfully found with {num_points} waypoints.")
            else:
                rospy.logwarn("Path planner returned array of size 1 (No path found).")

        except (_ArrayMemoryError, ValueError, RuntimeError) as e:
            rospy.logerr(f"Exception during path planning: {e}")

        # Cleanup memory
        del path_planner

        return PlanMultiContactPathResponse(
            success=success, 
            joint_trajectory=flat_q, 
            num_points=num_points
        )

if __name__ == "__main__":
    cfg_path = os.path.join(os.path.dirname(__file__), '../config/config_multi-c_our_handleless_axis_left_one_finger.yaml')
    
    if os.path.exists(cfg_path):
        with open(cfg_path, 'r') as f:
            loaded_config = yaml.safe_load(f)

    try:
        server = RVLPathPlanningService(loaded_config)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass