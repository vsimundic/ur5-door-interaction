            # ---------------------------------------------------------------
            # APPROACH: plan multi-contact path for the detected cabinet model
            # ---------------------------------------------------------------
            elif self.state == FSMState.APPROACH:

                rospy.loginfo("[FSM] Calling PlanMultiContactPath service...")
                trajectory = self.plan_path_for_door(door_index)

                if trajectory is None:
                    rospy.logerr("[FSM] Could not obtain a trajectory. Moving to ERROR.")
                    self.state = FSMState.ERROR
                    continue

                self._planned_trajectory = trajectory
                self.set_new_touch_scene(state_angle=0.0)
                rospy.loginfo(f"[FSM] Path ready ({self._planned_trajectory.shape[0]} waypoints). Moving to OPENING.")
                self.state = FSMState.OPENING

            # ---------------------------------------------------------------
            # OPENING: execute trajectory with force/tactile monitoring
            # ---------------------------------------------------------------
            elif self.state == FSMState.OPENING:

                trajectory = self._planned_trajectory
                if trajectory is None:
                    rospy.logerr("[FSM] No trajectory in OPENING state. Moving to ERROR.")
                    self.state = FSMState.ERROR
                    continue

                # --- Phase 1: Approach waypoints (first 3) ---
                rospy.loginfo("[FSM] Executing approach phase...")
                approach_success = self.execute_with_force_monitoring(
                    trajectory[:3],
                    self.force_threshold_approach,
                    max_velocity=0.5,
                    max_acceleration=0.5
                )

                if not approach_success:
                    rospy.logwarn("[FSM] Approach phase failed (force exceeded). Moving to REPLAN.")
                    self.state = FSMState.REPLAN
                    continue

                # --- Phase 2: Opening (remaining waypoints) ---
                rospy.loginfo("[FSM] Executing opening phase...")
                open_trajectory = trajectory[2:]

                tactile_loss_joints = []
                tactile_establish_joints = []
                self.robot.tactile_contact_established = False

                monitor_thread_loss = threading.Thread(
                    target=monitor_tactile_loss_and_remember_joints,
                    args=(self.robot, tactile_loss_joints, 0.3, 4.0, 50.0)
                )
                monitor_thread_establish = threading.Thread(
                    target=monitor_tactile_contact_establish,
                    args=(self.robot, tactile_establish_joints, 0.5, 4.0, 50.0)
                )

                monitor_thread_establish.start()
                monitor_thread_loss.start()

                open_success = self.robot.send_joint_trajectory_action2(
                    open_trajectory, max_velocity=0.2, max_acceleration=0.2)

                monitor_thread_establish.join()
                monitor_thread_loss.join()

                # --- Evaluate outcome ---
                if open_success and len(tactile_establish_joints) > 0:
                    rospy.loginfo("[FSM] Door opened successfully with tactile contact!")
                    self.state = FSMState.RECORD_RESULT

                elif open_success and len(tactile_establish_joints) == 0:
                    rospy.logwarn("[FSM] Finished but no tactile contact established (miss).")
                    if self.correct_on_touch:
                        self.correct_touch_model()
                    self.state = FSMState.REPLAN

                else:
                    # Tactile loss during opening
                    if len(tactile_loss_joints) > 0:
                        rospy.logwarn("[FSM] Tactile loss detected. Triggering touch correction.")
                        tactile_loss_arr = np.array(tactile_loss_joints)

                        T_6before_0 = self.robot.get_fwd_kinematics_moveit(
                            tactile_loss_arr[-2] if tactile_loss_arr.shape[0] >= 2 else open_trajectory[0]
                        )
                        T_6miss_0 = self.robot.get_fwd_kinematics_moveit(tactile_loss_arr[-1])

                        self.set_touch(
                            T_6miss_0,
                            np.array([T_6before_0, T_6miss_0]),
                            TouchType.MISS,
                            0.0
                        )

                        if self.correct_on_touch:
                            self.correct_touch_model()
                            rospy.loginfo("[FSM] Cabinet model corrected from touch.")
                    else:
                        rospy.logwarn("[FSM] Opening failed with no tactile data.")

                    self.state = FSMState.REPLAN

            # ---------------------------------------------------------------
            # TOUCH_CORRECT: correct model, then replan
            # ---------------------------------------------------------------
            elif self.state == FSMState.TOUCH_CORRECT:
                rospy.loginfo("[FSM] Correcting touch model...")
                self.correct_touch_model()
                self.state = FSMState.REPLAN

            # ---------------------------------------------------------------
            # REPLAN: call path planning service with updated cabinet model
            # ---------------------------------------------------------------
            elif self.state == FSMState.REPLAN:
                if replan_count >= max_replan_attempts:
                    rospy.logwarn(f"[FSM] Max replan attempts ({max_replan_attempts}) reached.")
                    self.state = FSMState.RECORD_RESULT
                    continue

                replan_count += 1
                rospy.loginfo(f"[FSM] Replanning attempt {replan_count}/{max_replan_attempts}...")

                # Use corrected cabinet model pose for replanning
                T_R_W = self.cabinet_model.T_A_W if hasattr(self.cabinet_model, 'T_A_W') else np.eye(4)
                trajectory = self.plan_path_for_door(door_index, T_R_W=T_R_W)

                if trajectory is not None:
                    self._planned_trajectory = trajectory
                    self.set_new_touch_scene(state_angle=0.0)
                    self.state = FSMState.OPENING
                else:
                    rospy.logwarn(f"[FSM] Replan attempt {replan_count} failed.")
                    if replan_count >= max_replan_attempts:
                        self.state = FSMState.RECORD_RESULT

            # ---------------------------------------------------------------
            # RECORD_RESULT
            # ---------------------------------------------------------------
            elif self.state == FSMState.RECORD_RESULT:
                door_opened = (replan_count < max_replan_attempts)
                rospy.loginfo(f"[FSM] Experiment done. door_opened={door_opened}, replans={replan_count}")
                self.state = FSMState.DONE

