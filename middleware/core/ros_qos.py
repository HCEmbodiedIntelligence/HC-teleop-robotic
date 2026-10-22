"""Explicit per-topic transport policy for large local image samples."""
from typing import Any


def subscription_qos(item: dict[str, Any]):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

    if item.get("topic") == "/tf_static":
        # Each recording creates a new subscriber. Retrieve the retained tree
        # from all static broadcasters, including those started before recording.
        return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)

    if item.get("qos_reliability") == "reliable":
        # A single latest sample avoids building a stale-video backlog.
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    return qos_profile_sensor_data
