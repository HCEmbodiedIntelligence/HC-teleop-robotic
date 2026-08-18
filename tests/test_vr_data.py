import json
import tempfile
import unittest
from pathlib import Path

try:
    import aiohttp  # noqa: F401
except ImportError:
    aiohttp = None

from hc_teleop_middleware.config import validate_config
from hc_teleop_middleware.protocol import ControllerInput, Pose, PosePacket


@unittest.skipIf(aiohttp is None, "aiohttp is installed by install.sh")
class VrDataTests(unittest.TestCase):
    def test_runtime_publishes_one_complete_vrdata_message(self):
        from hc_teleop_middleware.app import MiddlewareRuntime

        class RosCapture:
            def __init__(self):
                self.calls = []

            def publish(self, *args):
                self.calls.append(args)

        with tempfile.TemporaryDirectory() as directory:
            runtime = MiddlewareRuntime(validate_config({}), Path(directory))
            runtime.ros = RosCapture()
            packet = PosePacket(
                protocol_version=2,
                sequence=8,
                vr_timestamp=12.5,
                flags=7,
                head=Pose((0.0, 1.0, 2.0), (0.0, 0.0, 0.0, 1.0)),
                left=Pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)),
                right=Pose((4.0, 5.0, 6.0), (0.0, 0.0, 0.0, 1.0)),
                left_input=ControllerInput(held_mask=1, pressed_mask=1, trigger=0.4),
                right_input=ControllerInput(grip=0.8),
            )
            runtime._on_pose(packet)

            self.assertEqual(len(runtime.ros.calls), 1)
            topic, msg_type, fields = runtime.ros.calls[0]
            self.assertEqual(topic, "/vrdata")
            self.assertEqual(msg_type, "std_msgs/msg/String")
            payload = json.loads(fields["data"])
            self.assertEqual(payload["sequence"], 8)
            self.assertEqual(payload["poses"]["right"]["position"], [4.0, 5.0, 6.0])
            self.assertEqual(payload["inputs"]["left"]["pressed"], ["primary"])


if __name__ == "__main__":
    unittest.main()
