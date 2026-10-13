"""Explicit per-topic transport policy for large local image samples."""
from typing import Any


def subscription_qos(item: dict[str, Any]):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    if item.get("qos_reliability") == "reliable":
        # A single latest sample avoids building a stale-video backlog.
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    return qos_profile_sensor_data
