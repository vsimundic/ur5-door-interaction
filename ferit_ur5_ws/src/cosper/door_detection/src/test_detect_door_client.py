#!/usr/bin/env python

import rospy
import numpy as np
import json
import os
from door_detection.srv import DetectDoor, DetectDoorRequest

def test_door_detection():
    rospy.init_node("test_detect_door_client")
    
    rospy.loginfo("Waiting for 'detect_door' service...")
    rospy.wait_for_service('detect_door')
    rospy.loginfo("Service found. Preparing to call...")

    try:
        detect_door_srv = rospy.ServiceProxy('detect_door', DetectDoor)
        
        # 1. Setup Request Parameters
        trigger = True
        # Use a safe temp directory for the test
        base_dir = os.path.expanduser("~/ferit_ur5_ws/data/test_door_detection")
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)
            rospy.loginfo(f"Created test base directory: {base_dir}")
        
        # Dummy Joint Values (e.g., UR5 has 6 joints)
        joint_values = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        
        # Dummy 4x4 TCP transform (Identity matrix), flattened to 1D list of 16 elements
        t_6_0 = np.eye(4).flatten().tolist()

        # 2. Call the Service
        rospy.loginfo("Calling service, please wait while images are captured...")
        resp = detect_door_srv(trigger=trigger, base_dir=base_dir, joint_values=joint_values, T_6_0=t_6_0)
        
        # 3. Handle the Response
        if resp.success:
            rospy.loginfo("Service call SUCCESS!")
            
            # Parse the returned JSON string back into a dictionary to read it
            result_dict = json.loads(resp.result_json)
            print("-" * 50)
            print("DETECTION RESULT:")
            # Pretty print the json output
            print(json.dumps(result_dict, indent=2))
            print("-" * 50)
        else:
            rospy.logwarn("Service call FAILED (returned success=False).")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    test_door_detection()