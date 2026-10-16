"""HC public feedback and selected-command boundary, owned by the middleware.

The driver and model consumers use /internal/robot only. There is exactly one
converter on each direction; headers and absent optional measurements survive.
"""
import json
import time

from .robot_api import ENDPOINTS, INTERNAL_COMMAND, INTERNAL_STATE, RobotApiProfile
from .robot_profiles import RobotProfileError


class RobotApiBoundary:
    def __init__(self, node, profile):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

        self.node = node
        self.api = profile if isinstance(profile, RobotApiProfile) else RobotApiProfile(profile)
        self.feedback = None
        self.feedback_received = float('-inf')
        self.last_command_stamp = {}
        self.collision = None
        if self.api.teleop['control'].get('collision_avoidance_enabled', True):
            from adapters.core.collision_safety import ModelCollisionGuard
            self.collision = ModelCollisionGuard(self.api)
        self.max_age = self.api.motion['feedback_max_age_ms'] / 1000.
        self.state_pub = node.create_publisher(JointState, ENDPOINTS['joint_states'], 10)
        self.command_pub = node.create_publisher(JointState, INTERNAL_COMMAND, 1)
        self.collision_pub = node.create_publisher(String, '/status/robot/collision', 1)
        self.info_pub = node.create_publisher(String, ENDPOINTS['info'], QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscriptions = [
            node.create_subscription(JointState, INTERNAL_STATE, self.state, qos_profile_sensor_data),
            node.create_subscription(JointState, ENDPOINTS['joint_commands'], self.command, 1)]
        self.info_pub.publish(String(data=json.dumps(self.api.describe(), allow_nan=False)))

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def state(self, message):
        try:
            public = self.api.convert_message(message)
        except RobotProfileError as exc:
            self.node.get_logger().warning(f'rejected model feedback: {exc}', throttle_duration_sec=5.)
            return
        self.feedback, self.feedback_received = message, time.monotonic()
        self.state_pub.publish(public)

    def command(self, message):
        try:
            model = self.api.convert_message(message, to_model=True)
            stamp = self.stamp(message)
            age = (self.node.get_clock().now().nanoseconds - stamp) * 1e-9
            if stamp <= 0 or not -0.05 <= age <= self.max_age:
                raise RobotProfileError('stale/future selected command')
            if any(stamp <= self.last_command_stamp.get(n, 0) for n in model.name):
                raise RobotProfileError('reordered selected command')
            if self.feedback is None or time.monotonic() - self.feedback_received > self.max_age:
                raise RobotProfileError('model feedback is missing/stale')
            feedback_age = (self.node.get_clock().now().nanoseconds - self.stamp(self.feedback)) * 1e-9
            if not -0.05 <= feedback_age <= self.max_age or not set(model.name) <= set(self.feedback.name):
                raise RobotProfileError('required model feedback is stale/incomplete')
            for n, q in zip(model.name, model.position):
                joint = self.api.joints[n]
                lo, hi = joint['model_lower'], joint['model_upper']
                if (lo is not None and q < lo-1e-9) or (hi is not None and q > hi+1e-9):
                    raise RobotProfileError(f'selected command exceeds limits: {n}')
            if self.collision:
                from std_msgs.msg import String
                allowed, collision, escape = self.collision.check(
                    dict(zip(self.feedback.name, self.feedback.position)), dict(zip(model.name, model.position)))
                self.collision_pub.publish(String(data=json.dumps(dict(
                    blocked=not allowed, collision=collision, escape_mode=escape))))
                if not allowed:
                    raise RobotProfileError(f'shared collision check refused target: {collision}')
            self.command_pub.publish(model)
            self.last_command_stamp.update(dict.fromkeys(model.name, stamp))
        except (RobotProfileError, ValueError) as exc:
            self.node.get_logger().warning(f'rejected public command: {exc}', throttle_duration_sec=5.)

    def close(self):
        if self.collision:
            self.collision.close()
