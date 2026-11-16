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
        # State telemetry topics need a modest buffer to prevent dropping during JSON/WS dispatch.
        depth = 10 if item.get("type") in {
            "sensor_msgs/msg/JointState",
            "geometry_msgs/msg/PoseArray",
            "geometry_msgs/msg/PoseStamped",
        } else 1
        return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)
    return qos_profile_sensor_data


def recording_subscription_qos(item: dict[str, Any]):
    """Buffer small state samples across a short recorder scheduling pause.

    Live control and images retain their latest-sample policies. Recording a
    100 Hz state stream with the sensor profile's depth of five loses history
    after only 50 ms, before the capture's existing freshness checks can run.
    This bounded history does not change reliability or freshness checks.
    """
    qos = subscription_qos(item)
    if item.get('type') not in {
        'sensor_msgs/msg/JointState', 'geometry_msgs/msg/PoseStamped',
        'geometry_msgs/msg/TwistStamped', 'geometry_msgs/msg/WrenchStamped',
        'nav_msgs/msg/Odometry',
    }:
        return qos
    from rclpy.qos import QoSProfile
    return QoSProfile(depth=max(64, qos.depth), history=qos.history,
                      reliability=qos.reliability, durability=qos.durability)


def warm_recording_subscription_qos(item: dict[str, Any]):
    """Persistent ordinary-X1 evidence readers must never back-pressure control.

    Keep the configured recording depth, but explicitly use BEST_EFFORT for
    this limited set of warm sensor streams. Snapshot/correction readers keep
    their configured policy through recording_subscription_qos.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    qos = recording_subscription_qos(item)
    return QoSProfile(depth=qos.depth, history=qos.history,
                      reliability=ReliabilityPolicy.BEST_EFFORT, durability=qos.durability)
