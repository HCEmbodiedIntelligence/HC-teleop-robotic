"""Offline FK/IK regressions using the X1 model, without commanding a robot."""
import os
import tempfile
import yaml
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

    def make_enabled_waist_node(self, path):
        # Waist tests explicitly enable the feature; launch mode is not a preset.
        config = yaml.safe_load(path.read_text())
        config["body"].update(enabled=True, waist_joint_names=["leg_1", "leg_2", "zhi"], ik_iterations=20)
        config["robot"]["urdf_path"] = str(path.parent / config["robot"]["urdf_path"])
        with tempfile.TemporaryDirectory() as directory:
            test_path = Path(directory) / "arm_teleop.yaml"
            test_path.write_text(yaml.safe_dump(config))
            return RobotArmTeleopNode(str(test_path), backend="v23")

    def test_sim_unconstrained_3dof_head_tracking(self):
        """Test unconstrained 1:1 head target mapping and 3-DOF sagittal tracking in simulation mode."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = self.make_enabled_waist_node(path)
        try:
            node.body_config["unconstrained"] = True
            node.body_config["target_link"] = "camera_head"
            node.body.target_link_index = node.link_by_name["camera_head"]
            node.body_config["ik_mode"] = "3dof"
            node.body_config["target_position_deadband"] = 0.0
            node.body_config["target_orientation_deadband"] = 0.0
            node.body_config["target_filter_alpha"] = 1.0
            node.body_config["ik_posture_gain"] = 0.0
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
            node = self.make_enabled_waist_node(path)
        try:
            node.body_config["unconstrained"] = False
            node.body_config["target_link"] = "zhi_Link"
            node.body.target_link_index = node.link_by_name["zhi_Link"]
            node.body_config["ik_mode"] = "2dof"
            node.body_config["target_position_deadband"] = 0.0
            node.body_config["target_orientation_deadband"] = 0.0
            node.body_config["target_filter_alpha"] = 1.0
            node.body_config["ik_posture_gain"] = 0.0

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
            node = self.make_enabled_waist_node(path)
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
            # No clutch owns the robot, so even a zero/hold arm command must
            # not be emitted while the solver is waiting for rearm.
            self.assertEqual(published_commands, [])

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
            # Left Grip owns only the body; no arm hold targets are mixed in.
            for name in all_arm_names:
                self.assertNotIn(name, last_pub)
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

    def test_waist_deadband_jitter_rejection(self):
        """Verify that micro-jitter within deadbands produces zero joint motion."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = self.make_enabled_waist_node(path)
        try:
            node.joint_state.update(node.initial_joints)
            node.last_command.update(node.initial_joints)
            node._sync_model()

            # Ensure deadbands are active
            self.assertGreater(float(node.body_config.get("target_position_deadband", 0.0)), 0.01)
            self.assertGreater(float(node.body_config.get("target_orientation_deadband", 0.0)), 0.02)

            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            node._engage_body()

            # Simulate jitter within deadband (< 0.02 m, < 0.05 rad)
            jitters = [
                ([0.01, 0.005, -0.008], 0.02),
                ([-0.008, 0.012, 0.004], -0.03),
                ([0.005, -0.010, 0.005], 0.015),
                ([0.0, 0.014, 0.0], -0.025),
            ]
            for pos_jitter, rot_jitter in jitters:
                q_rot = quaternion_from_axis_angle([1.0, 0.0, 0.0], rot_jitter)
                node.body.head_pose = (np.array(pos_jitter), q_rot)
                node._update_body_target()
                command = {}
                node._update_body(command)
                for name in node.body.joint_names:
                    self.assertAlmostEqual(
                        command[name],
                        node.initial_joints[name],
                        places=4,
                        msg=f"Joint {name} moved during micro-jitter within deadband!",
                    )
        finally:
            node.destroy_node()

    def test_waist_pitch_glance_stability_and_no_knee_collapse(self):
        """Verify that natural head pitch tilt in 2-DOF mode does not cause knee collapse or solver divergence."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = self.make_enabled_waist_node(path)
        try:
            node.joint_state.update(node.initial_joints)
            node.last_command.update(node.initial_joints)
            node._sync_model()

            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            node._engage_body()

            # Operator tilts head down by 20 deg (~0.35 rad) with negligible head translation (pure neck glance)
            head_pitch_rad = np.deg2rad(20.0)
            target_rot = quaternion_from_axis_angle([1.0, 0.0, 0.0], head_pitch_rad)
            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), target_rot)

            # Stream 120 ticks of control to allow velocity ramp-up to steady state
            for _ in range(120):
                node._sync_model()
                command = {}
                node._update_body(command)
                node.joint_state.update(command)
                node.last_command.update(command)

            # 1. Knee joint (leg_2) must NOT collapse to 0 (home is ~1.2, limit is 0)
            self.assertGreater(
                command["leg_2"],
                0.7,
                f"Knee joint leg_2 collapsed! value: {command['leg_2']}",
            )

            # 2. Torso joint (zhi) must not slam to forward limit
            self.assertLess(
                command["zhi"],
                0.2,
                f"Torso joint zhi folded excessively! value: {command['zhi']}",
            )

            # 3. Robot target link pitch should have tracked the tilt smoothly
            node._sync_model()
            target_link_idx = (
                node.body.target_link_index
                if node.body.target_link_index is not None
                else node.body.torso_index
            )
            actual_link = node._link_pose(target_link_idx)
            mapped_target = node._world_pose(node._root_pose(), node.body.target_base)
            pitch_axis = np.asarray(node.body_config["pitch_axis_world"], dtype=float)
            pitch_axis /= np.linalg.norm(pitch_axis)
            pitch_err = float(pitch_axis @ orientation_error(mapped_target[1], actual_link[1]))
            self.assertLess(
                abs(pitch_err),
                0.01,
                f"Target link pitch tracking error too large: {pitch_err}",
            )

            # 4. Operator returns head upright
            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            for _ in range(120):
                node._sync_model()
                command = {}
                node._update_body(command)
                node.joint_state.update(command)
                node.last_command.update(command)

            # Verify joints return cleanly back to home
            for name in node.body.joint_names:
                self.assertAlmostEqual(
                    command[name],
                    node.initial_joints[name],
                    delta=0.08,
                    msg=f"Joint {name} did not return to home pose!",
                )
        finally:
            node.destroy_node()

    def test_sim_waist_initializes_and_resets_to_home(self):
        """Verify SimRobotController initializes folding_waist to initial_joints and resets on reset_home()."""
        from simulation.general_sim_robot_control_node_ros2 import SimRobotController
        cfg_path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/vr_configs.yml"
        sim_node = SimRobotController(str(cfg_path), init_debug=False, headless=True)
        try:
            expected_home = [0.5, 1.2, -0.6]
            self.assertEqual(len(sim_node.folding_waist.joint_pos), 3)
            for actual, exp in zip(sim_node.folding_waist.joint_pos, expected_home):
                self.assertAlmostEqual(actual, exp, places=3)

            # Perturb waist joints
            sim_node.reset_j(sim_node.folding_waist.joint_index, [0.0, 0.0, 0.0])
            for actual in sim_node.folding_waist.joint_pos:
                self.assertAlmostEqual(actual, 0.0, delta=0.05)

            # Call reset_home
            sim_node.reset_home()
            for actual, exp in zip(sim_node.folding_waist.joint_pos, expected_home):
                self.assertAlmostEqual(actual, exp, places=3)
        finally:
            sim_node.destroy_node()

    def test_teleop_node_startup_homing_when_waist_deviated(self):
        """Verify RobotArmTeleopNode triggers homing on startup if waist joints deviate from home."""
        from std_msgs.msg import Bool
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = self.make_enabled_waist_node(path)
        try:
            perturbed_state = dict(node.initial_joints)
            perturbed_state["leg_1"] = 0.0
            perturbed_state["leg_2"] = 0.0
            perturbed_state["zhi"] = 0.0

            node.joint_state.update(perturbed_state)
            node.last_command.update(perturbed_state)

            ready_msg = Bool()
            ready_msg.data = True
            node._hardware_ready_callback(ready_msg)

            # Homing should have been triggered
            self.assertTrue(node.homing)

            # Step control ticks until homing completes (max 200 ticks = 2.0s)
            now = 100.0
            for _ in range(200):
                now += 0.01
                cmd = node._command_seed()
                node._generic_control_tick(now, feedback_fresh=True, command=cmd)
                node.joint_state.update(node.last_command)
                if not node.homing:
                    break

            self.assertFalse(node.homing)
            for name in ["leg_1", "leg_2", "zhi"]:
                self.assertAlmostEqual(
                    node.last_command[name],
                    node.initial_joints[name],
                    delta=0.08,
                    msg=f"Waist joint {name} did not home on startup!",
                )
        finally:
            node.destroy_node()

    def test_a_button_resume_waist_safety_and_edge_rearm(self):
        """Verify A button resume keeps waist output disabled, overwrites last_command with feedback,
        and strictly requires a fresh left Grip rising edge before waist commands can be sent."""
        from std_msgs.msg import Bool, String
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = self.make_enabled_waist_node(path)
        try:
            node.hardware_ready = True
            feedback_waist = {"leg_1": 0.51, "leg_2": 1.21, "zhi": -0.59}
            node.joint_state.update(node.initial_joints)
            node.joint_state.update(feedback_waist)
            node.joint_state_stamp = node._monotonic()

            # Set a stale last_command that differs significantly from feedback
            stale_waist = {"leg_1": 0.85, "leg_2": 1.55, "zhi": -0.25}
            node.last_command.update(node.initial_joints)
            node.last_command.update(stale_waist)

            # Left Grip is currently held down by operator
            now = node._monotonic()
            left_joy = Joy()
            left_joy.axes = [0.0, 1.0, 0.0, 0.0]  # clutch axis 1 = 1.0 (held)
            node.arms["left"].joy = left_joy
            node.arms["left"].joy_stamp = now
            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            node.body.head_stamp = now

            # Step 1: Emergency stop occurs
            stop_msg = Bool(data=True)
            node._stop_callback(stop_msg)
            self.assertFalse(node.enabled)
            self.assertFalse(node.waist_output_enabled)
            self.assertTrue(node.waist_clutch_rearm_required)

            # Step 2: Operator presses VR controller A button to resume while STILL holding left Grip
            # Encode a VR frame with resume_requested=True and left Grip held
            import json
            vr_json = {
                "poses": {
                    "head": {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]},
                    "left": {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]},
                    "right": {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]},
                },
                "inputs": {
                    "left": {"grip": 1.0, "trigger": 0.0, "held_mask": 2},
                    "right": {"grip": 0.0, "trigger": 0.0, "pressed_mask": 1, "held_mask": 1},  # A button
                },
            }
            node._vr_data_callback(String(data=json.dumps(vr_json)))

            # Verification 1: Teleop re-enabled, but waist output MUST remain closed
            self.assertTrue(node.enabled)
            self.assertFalse(node.waist_output_enabled, "Waist output must remain disabled after A resume!")
            self.assertTrue(node.waist_clutch_rearm_required, "Rearm must be required after A resume!")

            # Verification 2: Current waist feedback MUST overwrite stale last_command
            for name, expected_val in feedback_waist.items():
                self.assertAlmostEqual(
                    node.last_command[name],
                    expected_val,
                    places=5,
                    msg=f"last_command for {name} was not overwritten with feedback upon A resume!",
                )

            # Verification 3: Holding left Grip without releasing CANNOT activate waist
            now += 0.02
            node.arms["left"].joy_stamp = now
            node.body.head_stamp = now
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertFalse(node.waist_output_enabled, "Waist must NOT activate without a fresh rising edge!")
            self.assertFalse(node.body.active, "Body must NOT engage when rearm is required!")

            # Step 3: Operator releases left Grip
            now += 0.02
            left_joy.axes[1] = 0.0
            node.arms["left"].joy = left_joy
            node.arms["left"].joy_stamp = now
            node.body.head_stamp = now
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertFalse(node.waist_output_enabled)
            self.assertFalse(node.body.active)
            self.assertTrue(node.waist_clutch_rearm_required, "Rearm requirement stays active until rising edge")

            # Step 4: Operator presses left Grip down (new rising edge)
            now += 0.02
            left_joy.axes[1] = 1.0
            node.arms["left"].joy_stamp = now
            node.body.head_stamp = now
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertTrue(node.waist_output_enabled, "Waist output must be enabled after new rising edge!")
            self.assertFalse(node.waist_clutch_rearm_required, "Rearm requirement must clear after rising edge")
            self.assertTrue(node.body.active, "Body must engage on new rising edge!")
        finally:
            node.destroy_node()

    def test_waist_command_not_periodically_sent_when_grip_released(self):
        """Verify that when left Grip is released, old waist targets are NOT periodically published
        in both internal IK (_control_tick) and external IK (_generic_command_callback) modes."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
            node = self.make_enabled_waist_node(path)
        try:
            node.hardware_ready = True
            node.joint_state.update(node.initial_joints)
            node.last_command.update(node.initial_joints)
            node.joint_state_stamp = node._monotonic()
            node._sync_model()

            waist_joint_set = set(node.body.joint_names)
            published_msgs = []
            node.command_pub.publish = lambda msg: published_msgs.append(msg)

            # Setup joys: right Grip held (operating arms), left Grip held (operating waist)
            now = node._monotonic()
            node.arms["right"].joy = Joy(axes=[0.0, 1.0, 0.0, 0.0])
            node.arms["right"].joy_stamp = now
            node.arms["right"].pose = (np.array([0.2, 0.2, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]))
            node.arms["right"].pose_stamp = now

            node.arms["left"].joy = Joy(axes=[0.0, 1.0, 0.0, 0.0])
            node.arms["left"].joy_stamp = now
            node.arms["left"].pose = (np.array([-0.2, 0.2, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]))
            node.arms["left"].pose_stamp = now
            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            node.body.head_stamp = now

            # --- Test 1: External IK mode (_generic_control_tick + _generic_command_callback) ---
            node.external_ik = True
            self.assertFalse(node.solver_rearm.blocked)
            for arm in node.arms.values():
                arm.active = False

            # When left Grip IS held, waist should be included
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertTrue(node.body.active)
            all_arm_names = node.arms["right"].joint_names + node.arms["left"].joint_names
            arm_msg = JointState()
            arm_msg.header = Header(stamp=node.get_clock().now().to_msg())
            arm_msg.name = all_arm_names
            arm_msg.position = [0.1] * len(all_arm_names)

            published_msgs.clear()
            node._generic_command_callback(arm_msg)
            self.assertTrue(len(published_msgs) > 0)
            self.assertTrue(waist_joint_set.issubset(set(published_msgs[-1].name)))

            # Now RELEASE left Grip (right Grip remains held for arms)
            now += 0.02
            node.arms["left"].joy.axes[1] = 0.0
            node.arms["left"].joy_stamp = now
            node.arms["right"].joy_stamp = now
            node.body.head_stamp = now
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertFalse(node.body.active)

            # Send multiple external solver ticks
            for _ in range(5):
                published_msgs.clear()
                arm_msg.header.stamp = node.get_clock().now().to_msg()
                node._generic_command_callback(arm_msg)
                self.assertTrue(len(published_msgs) > 0)
                published_names = set(published_msgs[-1].name)
                # Waist joints MUST NOT be sent when left Grip is released!
                waist_in_msg = waist_joint_set.intersection(published_names)
                self.assertEqual(
                    waist_in_msg,
                    set(),
                    f"Waist joints {waist_in_msg} were periodically sent after left Grip was released!",
                )

            # --- Test 2: Internal IK mode (_control_tick) ---
            node.external_ik = False
            from adapters.core.arm_teleop_node import LEGACY_IK_DEFAULTS
            for k, v in LEGACY_IK_DEFAULTS.items():
                node.control.setdefault(k, v)
            # When left Grip is released and right Grip held:
            published_msgs.clear()
            node._control_tick()
            self.assertTrue(len(published_msgs) > 0)
            published_names = set(published_msgs[-1].name)
            waist_in_msg = waist_joint_set.intersection(published_names)
            self.assertEqual(
                waist_in_msg,
                set(),
                f"In internal IK, waist joints {waist_in_msg} were periodically sent while left Grip was released!",
            )

            # Now press left Grip in internal IK mode (rising edge)
            now += 0.02
            node.arms["left"].joy.axes[1] = 1.0
            node.arms["left"].joy_stamp = now
            node.arms["right"].joy_stamp = now
            node.body.head_stamp = now
            published_msgs.clear()
            node._control_tick()
            self.assertTrue(len(published_msgs) > 0)
            published_names = set(published_msgs[-1].name)
            self.assertTrue(
                waist_joint_set.issubset(published_names),
                "Waist joints must be published when left Grip is engaged in internal IK!",
            )
        finally:
            node.destroy_node()
