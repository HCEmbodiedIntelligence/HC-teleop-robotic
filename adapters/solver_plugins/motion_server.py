"""Motion Server target-protocol adapter.

The core controller computes targets in each arm's configured task base.  This
adapter only translates those targets to Motion Server's per-arm ServoP topics;
it does not solve IK or republish joint commands.
"""
from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from geometry_msgs.msg import Pose, PoseStamped
from rclpy.qos import QoSProfile


class MotionServerTargetAdapter:
    def __init__(self, node: Any, config: Mapping[str, Any], qos: QoSProfile):
        settings = config.get("motion_server")
        if not isinstance(settings, Mapping):
            raise ValueError("motion_server backend requires a motion_server config section")
        channels = settings.get("servo_p")
        if not isinstance(channels, Mapping):
            raise ValueError("motion_server.servo_p must define left and right channels")
        self._publishers = {}
        self._frames = {}
        for side in ("left", "right"):
            channel = channels.get(side)
            if not isinstance(channel, Mapping):
                raise ValueError(f"motion_server.servo_p.{side} must be an object")
            topic = str(channel.get("topic", ""))
            frame = str(channel.get("frame_id", ""))
            if not topic.startswith("/") or not frame:
                raise ValueError(
                    f"motion_server.servo_p.{side} requires topic and frame_id"
                )
            self._publishers[side] = node.create_publisher(PoseStamped, topic, qos)
            self._frames[side] = frame

    def publish(self, stamp: Any, targets: Mapping[str, Pose]) -> None:
        for side in ("left", "right"):
            pose = targets.get(side)
            if pose is None:
                continue
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = self._frames[side]
            message.pose = pose
            self._publishers[side].publish(message)
