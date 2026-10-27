"""Unit tests for right hand controller joystick chassis yaw rotation."""
import os
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml

try:
    import rclpy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Float64MultiArray
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
except ImportError:
    RobotArmTeleopNode = None


@unittest.skipIf(RobotArmTeleopNode is None, "ROS/PyBullet dependencies unavailable")
class RightStickBaseControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=168)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_right_stick_yaw_control_with_and_without_left_clutch(self):
        """Test right stick yaw rotation when left clutch is held and independently without clutch."""
        path = Path(__file__).resolve().parents[1] / "adapters/robots/x1/arm_teleop.yaml"
        config = yaml.safe_load(path.read_text())
        # This case exercises the supported legacy joystick mode, independently
        # of the stock head-pose base follower and enabled waist.
        config["body"].update({
            "enabled": False, "waist_joint_names": [],
            "joint_lower": [], "joint_upper": [],
            "max_angular_velocity": 0.45,
            "max_linear_velocity": 0.35,
            "max_lateral_velocity": 0.25,
            "stick_deadzone": 0.08,
            "stick_x_axis": 2,
            "stick_y_axis": 3,
            "stick_forward_direction": -1.0,
            "stick_lateral_direction": -1.0,
            "right_stick_yaw_enabled": True,
            "right_stick_yaw_axis": 2,
            "right_stick_yaw_deadzone": 0.08,
            "right_stick_yaw_direction": -1.0,
        })
        config["body"]["head_pose_base"]["enabled"] = False
        # Relative model paths still resolve from the bundled profile directory.
        robot_urdf = Path(config["robot"]["urdf_path"])
        if not robot_urdf.is_absolute():
            config["robot"]["urdf_path"] = str((path.parent / robot_urdf).resolve())
        with tempfile.TemporaryDirectory() as temporary_directory:
            legacy_path = Path(temporary_directory) / "arm_teleop.yaml"
            legacy_path.write_text(yaml.safe_dump(config))
            with patch.dict(os.environ, {"HC_TELEOP_MODE": "sim"}):
                node = RobotArmTeleopNode(str(legacy_path), backend="v23")
        try:
            self.assertIsNone(node.head_base_follower)
            self.assertTrue({"leg_1", "leg_2", "zhi"}.isdisjoint(node.controlled_names))
            node.hardware_ready = True
            node.joint_state.update(node.initial_joints)
            node.last_command.update(node.initial_joints)
            node._sync_model()

            published_base_commands: list[list[float]] = []

            def mock_base_pub(msg):
                published_base_commands.append(list(msg.data))

            node.base_pub.publish = mock_base_pub

            now = 100.0
            left_joy = Joy()
            left_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            right_joy = Joy()
            right_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            node.arms["left"].joy = left_joy
            node.arms["left"].joy_stamp = now
            node.arms["right"].joy = right_joy
            node.arms["right"].joy_stamp = now
            node.body.head_pose = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            node.body.head_stamp = now

            # 1. Neutral state: no clutch, sticks centered -> base should stop
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertFalse(node.base_zero_pending)

            # 2. Independent right stick rotation (NO left clutch):
            # Push right stick full left: axes[2] = -1.0
            published_base_commands.clear()
            now += 0.02
            right_joy.axes[2] = -1.0
            node.arms["right"].joy_stamp = now
            node.arms["left"].joy_stamp = now

            # Deflect left stick with left_stick_independent_base=False (clutched mode test)
            node.body_config["left_stick_independent_base"] = False
            left_joy.axes[int(node.body_config["stick_x_axis"])] = 1.0
            left_joy.axes[int(node.body_config["stick_y_axis"])] = float(node.body_config["stick_forward_direction"])

            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())

            # Base must be active!
            self.assertTrue(len(published_base_commands) > 0)
            latest = published_base_commands[-1]
            yaw, fwd, lat = latest

            # With right_stick_yaw_direction=-1.0 and stick_yaw=-1.0 (left), yaw must be positive (~0.45 rad/s)
            self.assertAlmostEqual(yaw, 0.45, places=3)
            # Forward and lateral MUST remain strictly 0.0 since left clutch was not held!
            self.assertEqual(fwd, 0.0)
            self.assertEqual(lat, 0.0)
            # Body/waist must NOT be activated!
            self.assertFalse(node.body.active)

            # Test independent left stick translation when left_stick_independent_base=True
            node.body_config["left_stick_independent_base"] = True
            published_base_commands.clear()
            now += 0.02
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())
            self.assertTrue(len(published_base_commands) > 0)
            _, fwd_indep, _ = published_base_commands[-1]
            self.assertAlmostEqual(fwd_indep, 0.35, places=2)
            self.assertFalse(node.body.active)  # Waist still not activated!

            # Reset left stick for subsequent right-stick tests
            left_joy.axes[2] = 0.0
            left_joy.axes[3] = 0.0

            # 3. Independent right stick rotation to the right (axes[2] = 1.0)
            published_base_commands.clear()
            now += 0.02
            right_joy.axes[2] = 1.0
            node.arms["right"].joy_stamp = now
            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())

            self.assertTrue(len(published_base_commands) > 0)
            latest = published_base_commands[-1]
            yaw, fwd, lat = latest
            # Turning right must produce negative yaw (-0.45 rad/s)
            self.assertAlmostEqual(yaw, -0.45, places=3)
            self.assertEqual(fwd, 0.0)
            self.assertEqual(lat, 0.0)

            # 4. Squeeze left clutch (left Grip = 1.0):
            # Now both translation (left stick) and rotation (right stick) should be active simultaneously!
            published_base_commands.clear()
            now += 0.02
            left_joy.axes[1] = 1.0  # Grip pressed
            # Use the configured stick axes; negative Y is forward.
            x_axis = int(node.body_config["stick_x_axis"])
            y_axis = int(node.body_config["stick_y_axis"])
            left_joy.axes[y_axis] = -1.0  # forward
            left_joy.axes[x_axis] = 0.0
            right_joy.axes[2] = -1.0     # turn left

            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())

            self.assertTrue(len(published_base_commands) > 0)
            latest = published_base_commands[-1]
            yaw, fwd, lat = latest
            self.assertAlmostEqual(yaw, 0.45, places=3)
            self.assertGreater(fwd, 0.2)  # Forward translation is active!
            self.assertFalse(node.body.active)  # The explicit legacy fixture keeps the waist disabled.

            # 5. Deadzone check: right stick inside deadzone (< 0.08) produces 0 yaw
            published_base_commands.clear()
            now += 0.02
            left_joy.axes[1] = 0.0  # release left clutch
            left_joy.axes[y_axis] = 0.0  # center left stick
            right_joy.axes[2] = 0.04  # within deadzone
            node.arms["right"].joy_stamp = now
            node.arms["left"].joy_stamp = now

            node._generic_control_tick(now, feedback_fresh=True, command=node._command_seed())

            # Base must receive zero stop command
            self.assertTrue(len(published_base_commands) > 0)
            latest = published_base_commands[-1]
            self.assertEqual(latest, [0.0, 0.0, 0.0])
        finally:
            node.destroy_node()

