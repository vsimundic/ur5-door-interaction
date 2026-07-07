#!/usr/bin/env python
"""
Reusable trajectory execution with force and tactile monitoring.
Extracted from door_replanning_control.py for use in multi_contact_force_node.py.
"""

import rospy
import numpy as np
import threading
from enum import Enum
from core.real_ur5_controller import UR5Controller

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__),
    '../../force_manipulation/src'))
from force_utils import (
    monitor_force_and_cancel,
    monitor_force_and_cancel_remember_joints,
    monitor_force_drop_and_remember_joints,
    monitor_tactile_loss_and_remember_joints,
    monitor_tactile_contact_establish,
    chebyshev_distance
)


class ExecutionResult:
    """Result of a trajectory execution phase."""
    def __init__(self, success=False, collision_joints=None,
                 tactile_establish_joints=None, tactile_loss_joints=None,
                 phase=""):
        self.success = success
        self.collision_joints = collision_joints or []
        self.tactile_establish_joints = tactile_establish_joints or []
        self.tactile_loss_joints = tactile_loss_joints or []
        self.phase = phase

    @property
    def had_collision(self):
        return len(self.collision_joints) > 0

    @property
    def had_tactile_contact(self):
        return len(self.tactile_establish_joints) > 0

    @property
    def had_tactile_loss(self):
        return len(self.tactile_loss_joints) > 0


class TrajectoryExecutor:
    """
    Handles trajectory execution with force and tactile monitoring.
    Owns no state about the experiment — the FSM passes everything in.
    """

    def __init__(self, robot: UR5Controller, use_tactile: bool = False,
                 visualize: bool = True):
        self.robot = robot
        self.use_tactile = use_tactile

    # ------------------------------------------------------------------
    # Low-level execution primitives
    # ------------------------------------------------------------------

    def execute_simple(self, trajectory, max_velocity=0.5, max_acceleration=0.5):
        """Execute a trajectory without any monitoring."""
        return self.robot.send_joint_trajectory_action2(
            trajectory, max_velocity=max_velocity, max_acceleration=max_acceleration)

    def execute_force_only(self, trajectory, force_threshold,
                           max_velocity=0.5, max_acceleration=0.5):
        """Execute with force monitoring. Returns (success, collision_joints)."""
        collision_joints = []
        monitor_thread = threading.Thread(
            target=monitor_force_and_cancel_remember_joints,
            args=(self.robot, collision_joints, force_threshold, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        success = self.robot.send_joint_trajectory_action2(
            trajectory,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration)
        monitor_thread.join()
        return success, collision_joints

    def execute_force_simple(self, trajectory, force_threshold,
                             max_velocity=0.5, max_acceleration=0.5):
        """Execute with force monitoring (no joint recording)."""
        monitor_thread = threading.Thread(
            target=monitor_force_and_cancel,
            args=(self.robot, force_threshold))
        monitor_thread.start()
        success = self.robot.send_joint_trajectory_action2(
            trajectory,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration)
        monitor_thread.join()
        return success

    def execute_force_and_tactile(self, trajectory, force_threshold=40.0,
                                   tactile_loss_threshold=0.15,
                                   tactile_establish_threshold=0.5,
                                   tactile_timeout=4.0,
                                   tactile_grace_period=1.5,
                                   max_velocity=0.2, max_acceleration=0.2):
        """
        Execute with both force and tactile monitoring.
        Returns ExecutionResult with all relevant joint snapshots.
        """
        tactile_loss_joints = []
        tactile_establish_joints = []

        self.robot.tactile_contact_established = False

        # Force monitor
        force_thread = threading.Thread(
            target=monitor_force_and_cancel,
            args=(self.robot, force_threshold))

        # Tactile loss monitor
        tactile_loss_thread = threading.Thread(
            target=monitor_tactile_loss_and_remember_joints,
            args=(self.robot, tactile_loss_joints,
                  tactile_loss_threshold, tactile_timeout, 50.0,
                  tactile_grace_period))

        # Tactile establish monitor
        tactile_establish_thread = threading.Thread(
            target=monitor_tactile_contact_establish,
            args=(self.robot, tactile_establish_joints,
                  tactile_establish_threshold, tactile_timeout, 50.0))

        force_thread.start()
        tactile_loss_thread.start()
        tactile_establish_thread.start()

        success = self.robot.send_joint_trajectory_action2(
            trajectory,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration)

        force_thread.join()
        tactile_loss_thread.join()
        tactile_establish_thread.join()

        return ExecutionResult(
            success=success,
            tactile_establish_joints=tactile_establish_joints,
            tactile_loss_joints=tactile_loss_joints,
            phase="opening")

    def execute_force_drop_monitor(self, trajectory, drop_threshold=2.0,
                                    low_force_threshold=1.5):
        """Execute while monitoring for force drop (used during backup)."""
        ft_loss_joints = []
        monitor_thread = threading.Thread(
            target=monitor_force_drop_and_remember_joints,
            args=(self.robot, ft_loss_joints,
                  drop_threshold, low_force_threshold, 50.0))
        monitor_thread.start()
        rospy.sleep(0.05)
        self.robot.send_joint_trajectory_action2(
            trajectory, max_velocity=0.05, max_acceleration=0.05)
        monitor_thread.join()

        ft_loss_joints = np.array(ft_loss_joints) if ft_loss_joints else np.array([])
        T_6_0_ft_loss = None
        if ft_loss_joints.shape[0] == 1:
            T_6_0_ft_loss = self.robot.get_fwd_kinematics_moveit(
                ft_loss_joints[0].tolist())
        return ft_loss_joints, T_6_0_ft_loss

    # ------------------------------------------------------------------
    # High-level phase execution
    # ------------------------------------------------------------------

    def run_approach(self, approach_traj, force_threshold=30.0,
                     max_velocity=1.0, max_acceleration=0.7,
                     use_moveit=True):
        """
        Execute approach phase.
        Returns ExecutionResult.
        """
        current = self.robot.get_current_joint_values()
        full_traj = np.vstack((current, approach_traj))

        traj_to_exec = full_traj
        if use_moveit:
            planned, ok = self.robot.plan_to_joint_goals2(full_traj)
            if ok:
                traj_to_exec = planned

        self.robot.visualize_trajectory(traj_to_exec, start_joints=current)
        self.zero_sensor()
        success, collision_joints = self.execute_force_only(
            traj_to_exec, force_threshold,
            max_velocity, max_acceleration)

        return ExecutionResult(
            success=success,
            collision_joints=collision_joints,
            phase="approach")

    def run_insertion(self, insertion_target, force_threshold=7.0,
                      max_velocity=0.05, max_acceleration=0.05):
        """
        Execute insertion phase (slow push into door surface).
        Returns (ExecutionResult, T_6_0_ft_loss).
        """
        current = self.robot.get_current_joint_values()
        insertion_traj = np.vstack((current, insertion_target))

        self.robot.visualize_trajectory(insertion_traj, start_joints=current)
        self.zero_sensor()
        success, collision_joints = self.execute_force_only(
            insertion_traj, force_threshold,
            max_velocity, max_acceleration)

        T_6_0_ft_loss = None

        if not success and len(collision_joints) > 0:
            # Backup and monitor for force drop
            backup_traj = np.vstack((
                self.robot.get_current_joint_values(),
                insertion_traj[0]))
            _, T_6_0_ft_loss = self.execute_force_drop_monitor(
                backup_traj,
                drop_threshold=force_threshold - 1.0,
                low_force_threshold=1.5)

        return ExecutionResult(
            success=success,
            collision_joints=collision_joints,
            phase="insertion"), T_6_0_ft_loss

    def run_opening(self, opening_traj, force_threshold=40.0,
                    max_velocity=0.2, max_acceleration=0.2,
                    tactile_grace_period=1.5):
        """
        Execute opening phase with force (and optionally tactile) monitoring.
        Returns ExecutionResult.
        """
        if self.use_tactile:
            self.robot.visualize_trajectory(opening_traj, start_joints=self.robot.get_current_joint_values())
            return self.execute_force_and_tactile(
                opening_traj, force_threshold,
                tactile_grace_period=tactile_grace_period,
                max_velocity=max_velocity,
                max_acceleration=max_acceleration)
        else:
            self.robot.visualize_trajectory(opening_traj, start_joints=self.robot.get_current_joint_values())
            success, collision_joints = self.execute_force_only(
                opening_traj, force_threshold,
                max_velocity, max_acceleration)
            return ExecutionResult(
                success=success,
                collision_joints=collision_joints,
                phase="opening")

    def backup_along_tool_z(self, distance=0.03, force_threshold=30.0):
        """Back up along the tool Z axis."""
        T_cur = self.robot.get_current_tool_pose()
        T_backup = T_cur.copy()
        T_backup[:3, 3] -= distance * T_backup[:3, 2]
        q_current = self.robot.get_current_joint_values()
        q_backup = self.robot.get_closest_ik_solution(T_backup, q_current)
        if q_backup is not None:
            # Check Chebyshev distance
            cheb_dist = chebyshev_distance(q_current, q_backup)
            if cheb_dist > 0.5:
                rospy.logwarn(f"Chebyshev distance to backup solution is {cheb_dist:.2f}. Aborting backup.")
                return False
            
            backup_traj = np.vstack((q_current, q_backup))
            self.execute_force_simple(
                backup_traj, 
                force_threshold,
                max_velocity=0.1, 
                max_acceleration=0.1)
            return True
        return False

    def backup_to_joints(self, target_joints, force_threshold=30.0):
        """Back up to a specific joint configuration."""
        current = self.robot.get_current_joint_values()
        backup_traj = np.vstack((current, target_joints))
        return self.execute_force_simple(
            backup_traj, force_threshold,
            max_velocity=0.1, max_acceleration=0.1)

    def zero_sensor(self):
        """Zero the force-torque sensor."""
        rospy.loginfo("[Executor] Zeroing force sensor...")
        self.robot.zero_ft_sensor()