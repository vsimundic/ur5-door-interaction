#!/usr/bin/env python

import rospy
from core.ur5_commander import UR5Commander
import numpy as np
import socket

def create_urscript_with_force_monitoring(q_goal, force_threshold, pc_ip, pc_port, script_path):
    """
    Creates a URScript that moves to a goal joint position and stops if Z-force exceeds a threshold.
    
    Args:
        q_goal (list): Joint values for the goal position.
        force_threshold (float): Force threshold in Newtons.
        pc_ip (str): IP of the PC to send force info.
        pc_port (int): Port on the PC to send force info.
        script_path (str): Path to save the URScript file.
    """

    # Format joint goal for URScript
    q_goal_str = "[" + ", ".join([f"{q:.6f}" for q in q_goal]) + "]"

    urscript = f"""def force_monitored_move():
    textmsg("Starting monitored move to joint goal.")
    
    target = {q_goal_str}
    threshold = {force_threshold}
    reached = False

    # Move to joint goal
    movej(target, a=1.2, v=0.25)

    while not reached:
        force = get_tcp_force()
        textmsg("Fz: ", force[2])
        
        if abs(force[2]) > threshold:
            textmsg("Force exceeded! Z-force: ", force[2])
            stopj(2.0)
            break
        end

        current = get_actual_joint_positions()
        diff = norm(current - target)
        if diff < 0.01:
            reached = True
        end

        sleep(0.05)
    end

    # Final force report
    final_force = get_tcp_force()
    textmsg("Final Z-force: ", final_force[2])

    socket_open("{pc_ip}", {pc_port})
    socket_send_string(str(final_force[2]))
    socket_close()

    textmsg("Motion done or stopped due to force.")
end

force_monitored_move()
"""

    # Save to file
    with open(script_path, 'w') as f:
        f.write(urscript)

    print(f"URScript saved to {script_path}")


def main():
    rospy.init_node('stop_on_force_node')
    
    script_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/force_manipulation/script.script'
    
    # Get robot handle
    robot = UR5Commander()

    # Get current tool pose
    T_6_0 = robot.get_current_tool_pose()
    q_init = robot.get_current_joint_values()

    # WARNING: CURRENT POSE MUST NOT BE WITHIN 0.3m FROM THE ROBOT BASE
    # IN THAT CASE, THE ROBOT WILL NOT REACH POSE AND GIVE THIS WARNING:
    # "Force mode not possible in singularity"

    # Copy the current tool pose and move in y-direction for 30 cm 
    T_6_0_goal = T_6_0.copy()
    T_6_0_goal[1, 3] += 0.3

    q_goal = robot.get_inverse_kin(q_init, T_6_0_goal)

    # robot.send_multiple_joint_space_poses_to_robot2([q_init, q_goal])
    # robot.generate_URScript(np.array([q_goal]), with_force_mode=False, script_path=script_path)
    create_urscript_with_force_monitoring(q_goal, 10, robot.pc_ip, robot.pc_port, script_path)

    robot.send_URScript(True, script_path=script_path)

if __name__ == '__main__':
    main()