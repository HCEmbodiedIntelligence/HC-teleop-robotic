import unittest
from pathlib import Path

import yaml
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time

from adapters.solver_plugins.motion_server import MotionServerTargetAdapter
from adapters.solver_plugins.registry import (
    available_plugins,
    motion_server_resources,
    resolve_plugin,
)


ROOT = Path(__file__).resolve().parents[1]
OPENARM = ROOT / "adapters" / "robots" / "openarmx"


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, _message_type, topic, _qos):
        publisher = _Publisher(topic)
        self.publishers[topic] = publisher
        return publisher


class SolverPluginTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load(
            (OPENARM / "arm_teleop.yaml").read_text(encoding="utf-8")
        )

    def test_registry_selects_configured_or_overridden_backend(self):
        self.assertEqual(
            [plugin.name for plugin in available_plugins()],
            ["v23", "motion_server"],
        )
        self.assertEqual(resolve_plugin(self.config).name, "v23")
        self.assertEqual(
            resolve_plugin(self.config, "motion-server").name,
            "motion_server",
        )

    def test_motion_server_targets_are_split_by_arm_and_frame(self):
        node = _Node()
        adapter = MotionServerTargetAdapter(node, self.config, object())
        target = Pose()
        target.position.x = 0.42
        adapter.publish(Time(sec=1), {"right": target})

        right = node.publishers["/teleop/right_arm/servo_p"].messages
        left = node.publishers["/teleop/left_arm/servo_p"].messages
        self.assertEqual(len(right), 1)
        self.assertEqual(len(left), 0)
        self.assertEqual(right[0].header.frame_id, "openarmx_right_link0")
        self.assertAlmostEqual(right[0].pose.position.x, 0.42)

    def test_motion_server_output_stays_behind_command_mux(self):
        motion = yaml.safe_load(
            (OPENARM / "motion_server" / "motion_control.yaml").read_text(
                encoding="utf-8"
            )
        )
        params = motion["humanoid_motion_control"]["ros__parameters"]
        self.assertEqual(params["joint_state_endpoint"], "/hc_teleop/joint_states")
        self.assertEqual(params["joint_command_endpoint"], "/hc_teleop/joint_cmd_vr")

        middleware = yaml.safe_load(
            (ROOT / "middleware" / "config.yaml").read_text(encoding="utf-8")
        )
        mux = middleware["ros"]["command_mux"]
        self.assertEqual(mux["vr_topic"], params["joint_command_endpoint"])
        self.assertEqual(mux["output_topic"], "/hc_teleop/joint_cmd")
        controller_source = (
            ROOT / "adapters" / "core" / "arm_teleop_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("if self.motion_server_backend", controller_source)
        self.assertIn("else self.create_publisher", controller_source)

    def test_openarm_profile_contains_all_motion_server_resources(self):
        settings = self.config["motion_server"]
        for key in ("motion_params", "channel_config", "sdk_config", "tool_config"):
            self.assertTrue((OPENARM / settings[key]).is_file(), key)

        channels = yaml.safe_load(
            (OPENARM / settings["channel_config"]).read_text(encoding="utf-8")
        )["channels"]
        self.assertEqual({item["kind"] for item in channels}, {"servo_p"})
        self.assertEqual(
            {item["base_frame"] for item in channels},
            {"openarmx_left_link0", "openarmx_right_link0"},
        )
        resources = motion_server_resources(OPENARM / "arm_teleop.yaml")
        self.assertEqual(len(resources), 5)
        self.assertTrue(all(path.is_file() for path in resources))

    def test_launchers_use_the_sibling_humanoid_workspace(self):
        root_launcher = (ROOT / "start_teleop.sh").read_text(encoding="utf-8")
        adapter_launcher = (ROOT / "adapters" / "start.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('WORKSPACE_ROOT="$(cd -- "${PROJECT_ROOT}/.."', root_launcher)
        self.assertIn('${WORKSPACE_ROOT}/humanoid', root_launcher)
        self.assertIn('${WORKSPACE_ROOT}/humanoid', adapter_launcher)
        self.assertNotIn('${HOME}/humanoid', adapter_launcher)
        self.assertIn('${HUMANOID_ROOT}/.sdk_deps', adapter_launcher)


if __name__ == "__main__":
    unittest.main()
