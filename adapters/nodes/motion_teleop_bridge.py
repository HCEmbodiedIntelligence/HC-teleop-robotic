#!/usr/bin/env python3
"""Bridge the X1 external-IK adapter to motion ServoP/ServoJ channels.

The state/cache implementation is deliberately independent of ROS.  Receiving a
motion result alone never grants authority to move an arm: a fresh enabled
teleoperation status, hardware heartbeat and forwarded target are all required.
"""
from __future__ import annotations

import json
import math
import time

SIDES = ("right", "left")
ARM_NAMES = {side: tuple(f"Joint{i}_{'R' if side == 'right' else 'L'}" for i in range(1, 8)) for side in SIDES}


def joint_values(names, positions):
    if not names or len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("invalid joint names/positions")
    values = dict(zip(names, map(float, positions)))
    if any(not name or not math.isfinite(value) for name, value in values.items()):
        raise ValueError("invalid joint value")
    return values


def valid_stamp(stamp, wall, max_age=0.3):
    return math.isfinite(stamp) and stamp > 0 and -0.2 <= wall - stamp <= max_age


class MotionCache:
    def __init__(self):
        self.feedback = {}
        self.feedback_time = float('-inf')
        self.status = {}
        self.status_time = float('-inf')
        self.hardware = False
        self.hardware_time = float('-inf')
        self.estopped = False
        self.targets = {}
        self.lease = {}
        self.cutoff = {side: 0.0 for side in SIDES}
        self.reset_pending = False
        self.reset_wall = 0.0
        self.guard_until = 0.0

    def clear(self, wall):
        self.targets.clear()
        self.lease.clear()
        self.cutoff = {side: wall for side in SIDES}

    def update_status(self, status, now, wall):
        if not isinstance(status, dict):
            raise ValueError("status must be an object")
        if not isinstance(status.get('arms', {}), dict) or any(
                not isinstance(arm, dict) for arm in status.get('arms', {}).values()):
            raise ValueError('invalid arm status')
        self.status, self.status_time = status, now
        for side in SIDES:
            if not self.allowed(side, now):
                self.targets.pop(side, None)
                self.lease.pop(side, None)
                self.cutoff[side] = wall

    def allowed(self, side, now, homing=False):
        status = self.status
        return (not self.estopped and now >= self.guard_until and not self.reset_pending
                and self.hardware and now - self.hardware_time <= 0.5
                and now - self.status_time <= 0.6 and status.get('enabled') is True
                and now - self.feedback_time <= 0.3
                and all(name in self.feedback for name in ARM_NAMES[side])
                and (status.get('mode') == 'homing' if homing else
                     status.get('mode') != 'homing' and
                     status.get('arms', {}).get(side, {}).get('active') is True))

    def feedback_received(self, names, positions, stamp, now, wall):
        values = joint_values(names, positions)
        if not all(name in values for arm in ARM_NAMES.values() for name in arm):
            raise ValueError('incomplete arm feedback')
        if not valid_stamp(stamp, wall):
            raise ValueError("stale feedback header")
        if self.reset_pending and stamp <= self.reset_wall:
            raise ValueError('feedback predates reset')
        self.feedback, self.feedback_time = values, now
        if self.reset_pending:
            self.reset_pending = False
            self.clear(wall)
            self.guard_until = now + 0.1
            return True
        return False

    def reset(self, wall):
        self.clear(wall)
        self.reset_pending = True
        self.reset_wall = wall

    def grant(self, side, now, wall, homing=False):
        if not self.allowed(side, now, homing):
            return False
        if side not in self.lease:
            self.cutoff[side] = wall
        self.lease[side] = (now, homing)
        return True

    def accept_result(self, names, positions, stamp, now, wall):
        values = joint_values(names, positions)
        if not valid_stamp(stamp, wall, 0.1):
            return
        for side in SIDES:
            lease = self.lease.get(side)
            required = ARM_NAMES[side]
            if (lease is not None and now - lease[0] <= 0.2
                    and self.allowed(side, now, lease[1]) and stamp > self.cutoff[side]
                    and all(name in values for name in required)):
                self.targets[side] = ({name: values[name] for name in required}, now)

    def output(self, now, wall):
        if now - self.feedback_time > 0.3 or self.reset_pending:
            self.clear(wall)
            return None
        result = dict(self.feedback)
        for side in SIDES:
            lease = self.lease.get(side)
            target = self.targets.get(side)
            if (lease and target and self.allowed(side, now, lease[1])
                    and now - lease[0] <= 0.2 and now - target[1] <= 0.1):
                result.update(target[0])
            elif (lease and (not self.allowed(side, now, lease[1]) or now - lease[0] > 0.2)):
                self.targets.pop(side, None)
                self.lease.pop(side, None)
                self.cutoff[side] = wall
            elif target:
                # A late/missing result requires measured hold, but must not
                # revoke a fresh target lease and advance its acceptance cutoff.
                # Otherwise every timer tick rejects the in-flight SDK result.
                self.targets.pop(side, None)
        return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseArray, PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String

    def stamp_seconds(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    class MotionTeleopBridge(Node):
        def __init__(self):
            super().__init__('motion_teleop_bridge')
            self.cache = MotionCache()
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
            self.pose_pubs = {side: self.create_publisher(PoseStamped, f'/control/motion/{side}_arm/servo_p', qos) for side in SIDES}
            self.joint_pubs = {side: self.create_publisher(JointState, f'/control/motion/{side}_arm/servo_j', qos) for side in SIDES}
            self.command_pub = self.create_publisher(JointState, '/control/solver/joint_targets', qos)
            self.state_pub = self.create_publisher(JointState, '/status/solver/joint_states', qos)
            self.ack_pub = self.create_publisher(Header, '/status/solver/reset_ack', qos)
            self.create_subscription(JointState, '/observation/joint_states', self.feedback, qos_profile_sensor_data)
            self.create_subscription(JointState, '/control/motion/joint_targets', self.result, qos)
            self.create_subscription(PoseArray, '/action/ee_poses', self.pose, qos)
            self.create_subscription(JointState, '/control/solver/target_joint_from_vr', self.home, qos)
            self.create_subscription(String, '/status/teleop', self.status, qos)
            self.create_subscription(Bool, '/status/hardware/ready', self.ready, qos)
            self.create_subscription(Bool, '/control/emergency_stop', self.stop, qos)
            self.create_subscription(Bool, '/control/teleop/enabled', self.enable, qos)
            self.create_subscription(Bool, '/control/solver/reset', self.reset, qos)
            self.create_timer(0.01, self.tick)
            self.get_logger().info('motion bridge ready: X1 ServoP/ServoJ, fresh measured hold at 100 Hz')

        def times(self):
            return time.monotonic(), self.get_clock().now().nanoseconds * 1e-9

        def feedback(self, msg):
            now, wall = self.times()
            try:
                ack = self.cache.feedback_received(msg.name, msg.position, stamp_seconds(msg), now, wall)
            except (ValueError, TypeError):
                return
            if ack:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                self.ack_pub.publish(header)

        def result(self, msg):
            now, wall = self.times()
            try:
                self.cache.accept_result(msg.name, msg.position, stamp_seconds(msg), now, wall)
            except (ValueError, TypeError):
                pass

        def status(self, msg):
            now, wall = self.times()
            try:
                self.cache.update_status(json.loads(msg.data), now, wall)
            except (ValueError, TypeError, AttributeError):
                self.cache.status = {}
                self.cache.clear(wall)

        def ready(self, msg):
            now, wall = self.times()
            self.cache.hardware, self.cache.hardware_time = bool(msg.data), now
            if not msg.data:
                self.cache.clear(wall)

        def stop(self, msg):
            if msg.data:
                self.cache.estopped = True
                self.cache.clear(self.times()[1])

        def enable(self, msg):
            self.cache.estopped = not bool(msg.data)
            self.cache.clear(self.times()[1])

        def reset(self, msg):
            if msg.data:
                self.cache.reset(self.times()[1])

        def pose(self, msg):
            now, wall = self.times()
            if msg.header.frame_id != 'zhi_Link' or len(msg.poses) != 2 or not valid_stamp(stamp_seconds(msg), wall, 0.2):
                return
            for side, pose in zip(SIDES, msg.poses):
                q = pose.orientation
                numbers = [pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w]
                norm = math.sqrt(sum(value * value for value in numbers[3:]))
                if not all(math.isfinite(value) for value in numbers) or norm < 1e-8:
                    continue
                if self.cache.grant(side, now, wall):
                    target = PoseStamped()
                    target.header, target.pose = msg.header, pose
                    target.pose.orientation.x /= norm
                    target.pose.orientation.y /= norm
                    target.pose.orientation.z /= norm
                    target.pose.orientation.w /= norm
                    self.pose_pubs[side].publish(target)

        def home(self, msg):
            now, wall = self.times()
            try:
                values = joint_values(msg.name, msg.position)
            except (ValueError, TypeError):
                return
            if not valid_stamp(stamp_seconds(msg), wall, 0.2):
                return
            for side in SIDES:
                names = ARM_NAMES[side]
                if all(name in values for name in names) and self.cache.grant(side, now, wall, homing=True):
                    target = JointState()
                    target.header = msg.header
                    target.name, target.position = list(names), [values[name] for name in names]
                    self.joint_pubs[side].publish(target)

        def tick(self):
            now, wall = self.times()
            values = self.cache.output(now, wall)
            if values is not None:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name, msg.position = list(values), list(values.values())
                self.command_pub.publish(msg)
                self.state_pub.publish(msg)

    rclpy.init()
    node = MotionTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
