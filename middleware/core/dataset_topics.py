"""Read-only dataset projections; never feed these topics back into control."""
from __future__ import annotations

import json
import math


class DatasetTopics:
    def __init__(self, node, config):
        from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String
        from rclpy.qos import qos_profile_sensor_data

        self.node = node
        self.publishers = {}
        self.subscriptions = []
        self.groups = config['joint_groups']

        def subscribe(cls, topic, callback):
            self.subscriptions.append(node.create_subscription(
                cls, topic, callback, qos_profile_sensor_data))

        for category, source in [('observation', '/observation/joint_states'),
                                 ('action', '/action/joint_states')]:
            for group in self.groups:
                self.publisher(f'/{category}/{group}/joint_states', JointState)
            subscribe(JointState, source,
                      lambda msg, c=category: self.joints(c, msg))
        for category, source in [('observation', '/observation/ee_poses'),
                                 ('action', '/action/ee_poses')]:
            for side in ('right', 'left'):
                self.publisher(f'/{category}/{side}_arm/ee_pose', PoseStamped)
            subscribe(PoseArray, source, lambda msg, c=category: self.poses(c, msg))
        self.publisher('/action/base/velocity', TwistStamped)
        subscribe(Float64MultiArray, '/action/base/command', self.base)
        self.publisher('/observation/headset/pose', PoseStamped)
        subscribe(String, '/vrdata', self.head)
        self.publisher('/observation/head/pose', PoseStamped)
        self.publisher('/action/head/pose', PoseStamped)
        subscribe(PoseArray, '/status/head/tracking_poses', self.robot_head)
        from .camera_metadata import CameraMetadata
        self.camera_metadata = CameraMetadata(node, config.get('camera_metadata', {}))
        self.publishers.update(self.camera_metadata.publishers)
        self.subscriptions.extend(self.camera_metadata.subscriptions)
        self.publisher('/observation/ee_wrenches', Float64MultiArray)
        subscribe(Float64MultiArray, '/io_teleop/state_ee_wrenches',
                  lambda msg: self.relay('/observation/ee_wrenches', msg))

    def publisher(self, topic, cls):
        self.publishers[topic] = self.node.create_publisher(cls, topic, 10)

    def relay(self, topic, msg):
        """Republish immutable metadata under the standard observation namespace."""
        self.publishers[topic].publish(msg)

    def joints(self, category, msg):
        from sensor_msgs.msg import JointState
        indices = {name: i for i, name in enumerate(msg.name)}
        for group, names in self.groups.items():
            # Do not fabricate absent joints or mix stale samples into a group.
            if not all(name in indices for name in names):
                continue
            out = JointState()
            out.header = msg.header
            out.name = list(names)
            for field in ('position', 'velocity', 'effort'):
                values = getattr(msg, field)
                if len(values) == len(msg.name):
                    setattr(out, field, [values[indices[name]] for name in names])
            self.publishers[f'/{category}/{group}/joint_states'].publish(out)

    def poses(self, category, msg):
        from geometry_msgs.msg import PoseStamped
        if len(msg.poses) < 2:
            return
        for index, side in enumerate(('right', 'left')):
            out = PoseStamped()
            out.header = msg.header
            out.pose = msg.poses[index]
            self.publishers[f'/{category}/{side}_arm/ee_pose'].publish(out)

    def base(self, msg):
        from geometry_msgs.msg import TwistStamped
        if len(msg.data) != 3 or not all(math.isfinite(v) for v in msg.data):
            return
        out = TwistStamped()
        # Source Float64MultiArray has no header; this is bridge receipt time.
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = 'base_command'
        out.twist.angular.z, out.twist.linear.x, out.twist.linear.y = msg.data
        self.publishers['/action/base/velocity'].publish(out)

    def robot_head(self, msg):
        """Split [actual, target] camera-head poses in the robot base frame."""
        from geometry_msgs.msg import PoseStamped
        if len(msg.poses) < 2:
            return
        for topic, pose in (
            ('/observation/head/pose', msg.poses[0]),
            ('/action/head/pose', msg.poses[1]),
        ):
            out = PoseStamped()
            out.header = msg.header
            out.pose = pose
            self.publishers[topic].publish(out)

    def head(self, msg):
        from geometry_msgs.msg import PoseStamped
        try:
            data = json.loads(msg.data)
            if not data['tracking']['head']:
                return
            pose = data['poses']['head']
            p, q = pose['position'], pose['quaternion']
            if len(p) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in [*p, *q]):
                return
            if sum(v*v for v in q) < 1e-12:
                return
            out = PoseStamped()
            # Preserve original VR coordinates. vr_timestamp remains in /vrdata.
            out.header.stamp = self.node.get_clock().now().to_msg()
            out.header.frame_id = 'vr_tracking'
            out.pose.position.x, out.pose.position.y, out.pose.position.z = p
            (out.pose.orientation.x, out.pose.orientation.y,
             out.pose.orientation.z, out.pose.orientation.w) = q
            self.publishers['/observation/headset/pose'].publish(out)
        except (ValueError, TypeError, KeyError):
            return


def load_joint_groups(config, config_dir):
    """Resolve split groups from the active robot profile, including profiles without waist."""
    from adapters.core.runtime_config import unified_profile
    from .robot_api import RobotApiProfile, selected_profile
    import yaml
    directory = selected_profile(config, config_dir)
    if unified_profile(directory):
        api = RobotApiProfile(directory)
        return {group: [api.joints[n]['public_name'] for n in names]
                for group, names in api.groups.items() if group != 'whole_body'}
    adapter = yaml.safe_load((directory / 'arm_teleop.yaml').read_text())
    groups = {f'{side}_arm': list(arm['joint_names']) for side, arm in adapter['arms'].items()}
    waist = adapter.get('body', {}).get('waist_joint_names', [])
    if waist:
        groups['waist'] = list(waist)
    return groups
