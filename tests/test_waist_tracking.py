"""Offline FK/IK regressions using the X1 model, without commanding a robot."""
import os
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pybullet as bullet

try:
    import rclpy
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import Header
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from adapters.core.arm_teleop_math import orientation_error, quaternion_from_axis_angle
except ImportError:
    RobotArmTeleopNode = None


@unittest.skipIf(RobotArmTeleopNode is None, "ROS/PyBullet dependencies unavailable")
class WaistTrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Node currently uses the default context; isolate its DDS domain.
        rclpy.init(domain_id=167)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_sim_unconstrained_3dof_head_tracking(self):
        """Test unconstrained 1:1 head target mapping and 3-DOF sagittal tracking in simulation mode."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = RobotArmTeleopNode(str(path), backend="v23")
        try:
            head_link = node.link_by_name["camera_head"]
            pitch_axis = np.asarray(node.body_config["pitch_axis_world"], dtype=float)
            pitch_axis /= np.linalg.norm(pitch_axis)
            root_rot = np.asarray(
                bullet.getMatrixFromQuaternion(node._root_pose()[1]), dtype=float
            ).reshape(3, 3)
            fwd_axis = root_rot[:, 0]

            for delta in (
                [0.0, 0.0, 0.1],
                [0.0, 0.1, 0.0],
                [0.0, 0.08, 0.08],
                [0.0, 0.0, -0.1],
                [0.0, -0.1, 0.0],
                [0.0, -0.08, -0.08],
            ):
                with self.subTest(head_delta=delta):
                    node.joint_state.update(node.initial_joints)
                    node.last_command.update(node.initial_joints)
                    node._sync_model()
                    head_rot = (
                        quaternion_from_axis_angle([1.0, 0.0, 0.0], delta[2])
                        if delta[2] != 0
                        else np.array([0.0, 0.0, 0.0, 1.0])
                    )
                    node.body.head_pose = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
                    node._engage_body()
                    node.body.head_pose = (np.array(delta), head_rot)
                    node._update_body_target()

                    initial_head = node._link_pose(head_link)
                    mapped_target = node._world_pose(node._root_pose(), node.body.target_base)

                    # 1:1 竖直高度无约束映射校验
                    self.assertAlmostEqual(
                        mapped_target[0][2] - initial_head[0][2],
                        delta[1],
                        places=5,
                    )

                    for _ in range(400):
                        node._sync_model()
                        previous = np.array([node.joint_state[k] for k in node.body.joint_names])
                        command = {}
                        node._update_body(command)
                        q = np.array([command[k] for k in node.body.joint_names])
                        max_step = float(node.body_config["max_joint_velocity"]) / float(
                            node.control["rate_hz"]
                        )
                        self.assertLessEqual(np.max(abs(q - previous)), max_step + 1e-9)
                        node.joint_state.update(command)
                        node.last_command.update(command)

                    node._sync_model()
                    actual_head = node._link_pose(head_link)
                    fwd_error = abs(float(fwd_axis @ (mapped_target[0] - actual_head[0])))
                    lift_error = abs(float(mapped_target[0][2] - actual_head[0][2]))
                    pitch_error = abs(
                        float(pitch_axis @ orientation_error(mapped_target[1], actual_head[1]))
                    )

                    self.assertLess(fwd_error, 0.003)
                    self.assertLess(lift_error, 0.003)
                    self.assertLess(pitch_error, 0.003)
                    print(
                        "3-DOF sim head tracking delta",
                        delta,
                        "fwd err mm",
                        round(fwd_error * 1000, 2),
                        "lift err mm",
                        round(lift_error * 1000, 2),
                        "pitch err deg",
                        round(np.rad2deg(pitch_error), 3),
                    )
        finally:
            node.destroy_node()

    def test_torso_2dof_tracking(self):
        """Test torso 2-DOF lift/pitch tracking mode with custom scales."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = RobotArmTeleopNode(str(path), backend="v23")
        try:
            node.body_config["unconstrained"] = False
            node.body_config["target_link"] = "zhi_Link"
            node.body.target_link_index = node.link_by_name["zhi_Link"]
            node.body_config["ik_mode"] = "2dof"

            for delta in ([0.0, 0.1, 0.0], [0.0, 0.0, 0.1]):
                with self.subTest(head_delta=delta):
                    node.joint_state.update(node.initial_joints)
                    node.last_command.update(node.initial_joints)
                    node._sync_model()
                    head_rot = (
                        quaternion_from_axis_angle([1.0, 0.0, 0.0], delta[2])
                        if delta[2] != 0
                        else np.array([0.0, 0.0, 0.0, 1.0])
                    )
                    node.body.head_pose = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
                    node._engage_body()
                    node.body.head_pose = (np.array(delta), head_rot)
                    node._update_body_target()
                    initial_torso = node.body.reference_torso
                    mapped_target = node._world_pose(node._root_pose(), node.body.target_base)

                    self.assertAlmostEqual(
                        mapped_target[0][2] - initial_torso[0][2],
                        delta[1] * float(node.body_config["height_scale"]),
                        places=5,
                    )
                    for _ in range(400):
                        node._sync_model()
                        command = {}
                        node._update_body(command)
                        node.joint_state.update(command)
                        node.last_command.update(command)
                    node._sync_model()
                    actual = node._link_pose(node.body.torso_index)
                    target = node._world_pose(node._root_pose(), node.body.target_base)
                    lift_error = abs(float(target[0][2] - actual[0][2]))
                    pitch_error = abs(
                        float(
                            np.dot(
                                np.array([-1.0, 0.0, 0.0]),
                                orientation_error(target[1], actual[1]),
                            )
                        )
                    )
                    self.assertLess(lift_error, 0.003)
                    self.assertLess(pitch_error, 0.003)
        finally:
            node.destroy_node()

    def test_post_homing_independent_waist_control_without_right_clutch(self):
        """Verify waist can be controlled after homing using left Grip alone while arms wait for rearm."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = RobotArmTeleopNode(str(path), backend="v23")
        try:
            node.hardware_ready = True
            node.joint_state.update(node.initial_joints)
            node.last_command.update(node.initial_joints)
            node.joint_state_stamp = node._monotonic()
            node._sync_model()

            # 1. Trigger homing and then complete it
            node.solver_rearm.start_homing()
            node.homing = False
            node.solver_rearm.homing_complete()
            self.assertEqual(node.solver_rearm.state, "wait_feedback")

            # 2. Feedback arrives and reset is published
            cutoff_ns = int(node.get_clock().now().nanoseconds)
            self.assertTrue(node.solver_rearm.observe_feedback())
            node.solver_rearm.reset_published(cutoff_ns)

            # 3. Solver reset ack arrives
            self.assertTrue(node.solver_rearm.observe_reset_ack(cutoff_ns + 10))
            self.assertEqual(node.solver_rearm.state, "wait_solver")

            # 4. Fresh solver output arrives via _generic_command_callback
            arm_msg = JointState()
            arm_msg.header = Header(stamp=node.get_clock().now().to_msg())
            arm_msg.header.stamp.nanosec = (cutoff_ns + 20) % 1_000_000_000
            arm_msg.header.stamp.sec = (cutoff_ns + 20) // 1_000_000_000
            all_arm_names = node.arms["right"].joint_names + node.arms["left"].joint_names
            # Solver sends arbitrary arm values (different from initial_joints)
            arm_msg.name = all_arm_names
            arm_msg.position = [1.23] * len(all_arm_names)

            published_commands = []
            node.command_pub.publish = lambda msg: published_commands.append(msg)

            # Left clutch is NOT pressed, right clutch is NOT pressed yet
            node._generic_command_callback(arm_msg)
            # Fresh solver output was observed -> state is WAIT_RELEASE
            self.assertEqual(node.solver_rearm.state, "wait_release")
            self.assertTrue(node.solver_rearm.blocked)
            # Arm positions should NOT be 1.23, but held at initial_joints
            self.assertTrue(len(published_commands) > 0)
            last_pub = dict(zip(published_commands[-1].name, published_commands[-1].position))
            for name in all_arm_names:
                self.assertAlmostEqual(last_pub[name], node.initial_joints[name])

            # 5. Now operator presses LEFT grip alone (left clutch = 1.0, right clutch = 0.0)
            now = node._monotonic()
            left_joy = Joy()
            left_joy.axes = [0.0, 1.0, 0.0, 0.0]  # clutch axis 1 = 1.0
            node.arms["left"].joy = left_joy
            node.arms["left"].joy_stamp = now

            right_joy = Joy()
            right_joy.axes = [0.0, 0.0, 0.0, 0.0]  # clutch axis 1 = 0.0
            node.arms["right"].joy = right_joy
            node.arms["right"].joy_stamp = now

            # Engage at initial head pose
            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            node.body.head_stamp = now
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())

            # Verify left clutch engaged body
            self.assertTrue(node.body.active)
            self.assertFalse(node.arms["right"].active)
            self.assertFalse(node.arms["left"].active)
            self.assertTrue(node.solver_rearm.blocked)  # arms still blocked!

            # Advance head pose to generate tracking delta
            now += 0.02
            node.body.head_pose = (np.array([0.0, 0.08, 0.08]), np.array([0.0, 0.0, 0.0, 1.0]))
            node.body.head_stamp = now
            node.arms["left"].joy_stamp = now
            node.arms["right"].joy_stamp = now
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())

            # Send another solver frame
            published_commands.clear()
            arm_msg.header.stamp = node.get_clock().now().to_msg()
            node._generic_command_callback(arm_msg)

            self.assertTrue(len(published_commands) > 0)
            last_pub = dict(zip(published_commands[-1].name, published_commands[-1].position))
            # Verify arms are STILL held at initial joints, NOT solver's 1.23!
            for name in all_arm_names:
                self.assertAlmostEqual(last_pub[name], node.initial_joints[name])
            # Verify waist joints ARE published and differ from initial_joints!
            waist_changed = any(
                abs(last_pub[name] - node.initial_joints[name]) > 1e-4
                for name in node.body.joint_names
            )
            self.assertTrue(waist_changed, "Waist must track head even while arms wait for rearm!")

            # 6. Now operator presses RIGHT grip (rearm arms)
            now = node._monotonic()
            right_joy.axes[1] = 1.0
            node.arms["right"].joy_stamp = now
            node.arms["left"].joy_stamp = now
            for side in ("right", "left"):
                node.arms[side].pose_stamp = now
                node.arms[side].pose = (np.array([0.2, 0.2, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]))
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertFalse(node.solver_rearm.blocked)
            self.assertTrue(node.arms["right"].active)
            self.assertTrue(node.arms["left"].active)
            self.assertTrue(node.body.active)
        finally:
            node.destroy_node()
