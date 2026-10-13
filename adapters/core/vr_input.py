from __future__ import annotations

import json
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy


@dataclass(frozen=True)
class DecodedVrFrame:
    poses: dict[str, PoseStamped]
    inputs: dict[str, Joy]
    resume_requested: bool


def decode_vr_frame(payload: str) -> DecodedVrFrame | None:
    """Convert the unified /vrdata JSON envelope into typed ROS inputs."""
    try:
        value = json.loads(payload)
        if not isinstance(value, dict):
            return None
        tracking = value.get("tracking", {})
        poses = value.get("poses", {})
        inputs = value.get("inputs", {})
        if not all(isinstance(item, dict) for item in (tracking, poses, inputs)):
            return None

        decoded_poses: dict[str, PoseStamped] = {}
        for name in ("head", "left", "right"):
            source = poses.get(name)
            if not tracking.get(name) or not isinstance(source, dict):
                continue
            position = source.get("position")
            quaternion = source.get("quaternion")
            if not isinstance(position, list) or len(position) != 3:
                continue
            if not isinstance(quaternion, list) or len(quaternion) != 4:
                continue
            result = PoseStamped()
            (
                result.pose.position.x,
                result.pose.position.y,
                result.pose.position.z,
            ) = [float(item) for item in position]
            (
                result.pose.orientation.x,
                result.pose.orientation.y,
                result.pose.orientation.z,
                result.pose.orientation.w,
            ) = [float(item) for item in quaternion]
            decoded_poses[name] = result

        decoded_inputs: dict[str, Joy] = {}
        resume_requested = False
        for side in ("left", "right"):
            source = inputs.get(side)
            if not isinstance(source, dict):
                continue
            primary = source.get("primary_axis", [0.0, 0.0])
            secondary = source.get("secondary_axis", [0.0, 0.0])
            if not isinstance(primary, list) or len(primary) != 2:
                continue
            if not isinstance(secondary, list) or len(secondary) != 2:
                continue
            held_mask = int(source.get("held_mask", 0))
            pressed_mask = int(source.get("pressed_mask", 0))
            resume_requested = resume_requested or bool(
                side == "right" and ((pressed_mask & 1) or (held_mask & 1))
            )
            joy = Joy()
            joy.axes = [
                float(source.get("trigger", 0.0)),
                float(source.get("grip", 0.0)),
                float(primary[0]),
                float(primary[1]),
                float(secondary[0]),
                float(secondary[1]),
            ]
            joy.buttons = [
                int(bool(held_mask & (1 << index))) for index in range(11)
            ]
            decoded_inputs[side] = joy
        return DecodedVrFrame(decoded_poses, decoded_inputs, resume_requested)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
