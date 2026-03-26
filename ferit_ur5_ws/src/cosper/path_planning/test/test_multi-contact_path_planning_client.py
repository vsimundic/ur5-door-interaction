#!/usr/bin/env python

import rospy
import os
import numpy as np
from rospkg import RosPack

from core.util import read_csv_DataFrame
from core.transforms import rot_z
from core.real_ur5_controller import UR5Controller

# Import the newly defined service
from path_planning.srv import PlanMultiContactPath

def test_path_planning_client():
    rospy.init_node('test_rvl_path_client')
    
    # Make sure this string matches the one you registered in your server node
    service_name = 'plan_multi_contact_path' 
    
    rospy.loginfo(f"Waiting for '{service_name}' service...")
    rospy.wait_for_service(service_name)
    plan_srv = rospy.ServiceProxy(service_name, PlanMultiContactPath)
    rospy.loginfo("Service found!")

    # Setup paths
    rp = RosPack()
    pkg_path = rp.get_path('path_planning')
    workspace_path = os.path.dirname(pkg_path[:pkg_path.find('/src/')])

    read_results_path = os.path.join(workspace_path, 'data', 'multi-contact/simulations/results_multi-c_our_handleless_real3.csv')
    door_configs_path = os.path.join(workspace_path, 'data', 'multi-contact/cabinet_configurations_axis_left_real3.npy')

    # Load data
    data = read_csv_DataFrame(read_results_path)
    doors = np.load(door_configs_path)

    # success_data = data.loc[((data['path_found'] == True) & 
    #                         (data['traj_success'] == True) & 
    #                         (data['contact_free'] == True) & 
    #                         (data['door_opened'] == True))] 
    success_data = data.loc[data['path_found'] == False] 
    n_tests = 5  # Just test the first 5 successful ones
    exps = success_data.head(n_tests).axes[0].tolist()
    
    # Static parameters matching test_simulation_real_robot.py
    door_thickness = 0.018
    static_depth = 0.35
    static_side_width = 0.017
    axis_distance = 0.01
    
    T_R_W = np.eye(4)

    for i_idx, i in enumerate(exps):
        rospy.loginfo(f"--- Testing Cabinet Index {i} ({i_idx+1}/{len(exps)}) ---")
        
        door = doors[i, :]
        width = door[0]
        height = door[1]
        position = door[2:5]
        rot_z_deg = door[5]
        state_angle = door[6]
        axis_pos = int(door[7])

        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(position)
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(rot_z_deg))
        T_A_S = T_A_S @ Tz

        # Get initial joint values and apply offsets required for path planning
        q_init = np.array([0., -np.pi*0.5, 0., -np.pi*0.5, 0., 0.])
        q_init[0] += np.pi
        q_init[5] += np.pi
        q_init[q_init > np.pi] -= (2.0 * np.pi)     
        q_init[q_init < -np.pi] += (2.0 * np.pi)

        # Calculate new parameters for the Multi-Contact service
        rx = 0.0
        ry = -(float(width) * 0.5 - axis_distance)
        moving_to_static_part_distance = 0.005

        # Call the path planning service
        try:
            resp = plan_srv(
                w_door=float(width),
                h_door=float(height),
                d_door=float(door_thickness),
                rx=float(rx),
                ry=float(ry),
                static_side_width=float(static_side_width),
                axis_distance=float(axis_distance),
                moving_to_static_part_distance=float(moving_to_static_part_distance),
                axis_pos=axis_pos,
                T_A_S=T_A_S.flatten().tolist(),
                T_R_W=T_R_W.flatten().tolist(),
                q_init=q_init.tolist(),
                start_angle=float(state_angle),
                target_angle=-90.0,
                num_states=37
            )

            if resp.success:
                trajectory = np.array(resp.joint_trajectory).reshape(resp.num_points, 6)
                rospy.loginfo(f"Success! Path found with {resp.num_points} waypoints.")
                rospy.loginfo(f"First Waypoint: {np.round(trajectory[0], 3)}")
                rospy.loginfo(f"Last Waypoint: {np.round(trajectory[-1], 3)}")
            else:
                rospy.logwarn(f"Failed to find path for cabinet {i}.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed for cabinet {i}: {e}")
            
        rospy.sleep(1.0) # Small pause between tests

if __name__ == '__main__':
    try:
        test_path_planning_client()
    except rospy.ROSInterruptException:
        pass