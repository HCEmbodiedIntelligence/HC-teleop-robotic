"""Read-only dataset projections; never feed these topics back into control."""
from __future__ import annotations

import json
import math


def _has_subscribers(pub) -> bool:
    """Return True unless a real ROS publisher reports zero active subscribers."""
    count = getattr(pub, 'get_subscription_count', None)
    return not callable(count) or count() > 0


class DatasetTopics:
    def __init__(self, node, config):
        from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        dataset_sub_qos = QoSProfile(
            depth=64,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.node = node
        self.publishers = {}
        self.subscriptions = []
        self.groups = config['joint_groups']

        try:
            from rclpy.node import Node as _RclpyNode
            use_raw = isinstance(node, _RclpyNode)
        except ImportError:
            use_raw = False
        if use_raw:
            from rclpy.serialization import deserialize_message

        def subscribe(cls, topic, callback, target_pubs=()):
            if use_raw and target_pubs:
                targets = tuple(target_pubs)
                def filtered(msg_or_raw, _cls=cls, _cb=callback, _targets=targets):
                    if not any(_has_subscribers(pub) for pub in _targets):
                        return
                    if isinstance(msg_or_raw, (bytes, bytearray, memoryview)):
                        msg_or_raw = deserialize_message(msg_or_raw, _cls)
                    _cb(msg_or_raw)
                self.subscriptions.append(node.create_subscription(
                    cls, topic, filtered, dataset_sub_qos, raw=True))
                return
            self.subscriptions.append(node.create_subscription(
                cls, topic, callback, dataset_sub_qos))

        for category, source in [('observation', '/observation/joint_states'),
                                 ('action', '/action/joint_states')]:
            pubs = [self.publisher(f'/{category}/{group}/joint_states', JointState)
                    for group in self.groups]
            subscribe(JointState, source,
                      lambda msg, c=category: self.joints(c, msg), pubs)
        for category, source in [('observation', '/observation/ee_poses'),
                                 ('action', '/action/ee_poses')]:
            pubs = [self.publisher(f'/{category}/{side}_arm/ee_pose', PoseStamped)
                    for side in ('right', 'left')]
            subscribe(PoseArray, source, lambda msg, c=category: self.poses(c, msg), pubs)
        subscribe(PoseArray, '/internal/recording/action/ee_poses',
                  lambda msg: self.poses('action', msg),
                  (self.publishers['/action/right_arm/ee_pose'],
                   self.publishers['/action/left_arm/ee_pose']))
        if config.get('x1_recording_fk'):
            category = 'internal/recording/observation'
            pubs = [self.publisher(f'/{category}/{side}_arm/ee_pose', PoseStamped)
                    for side in ('right', 'left')]
            subscribe(PoseArray, f'/{category}/ee_poses',
                      lambda msg: self.poses(category, msg), pubs)
        base_pub = self.publisher('/action/base/velocity', TwistStamped)
        subscribe(Float64MultiArray, '/action/base/command', self.base, (base_pub,))
        headset_pub = self.publisher('/observation/headset/pose', PoseStamped)
        subscribe(String, '/vrdata', self.head, (headset_pub,))
        head_pubs = (
            self.publisher('/observation/head/pose', PoseStamped),
            self.publisher('/action/head/pose', PoseStamped),
        )
        subscribe(PoseArray, '/status/head/tracking_poses', self.robot_head, head_pubs)
        from .camera_metadata import CameraMetadata
        self.camera_metadata = CameraMetadata(node, config.get('camera_metadata', {}))
        self.publishers.update(self.camera_metadata.publishers)
        self.subscriptions.extend(self.camera_metadata.subscriptions)

    def publisher(self, topic, cls):
        pub = self.node.create_publisher(cls, topic, 10)
        self.publishers[topic] = pub
        return pub

    def relay(self, topic, msg):
        """Republish immutable metadata under the standard observation namespace."""
        pub = self.publishers[topic]
        if _has_subscribers(pub):
            pub.publish(msg)

    def joints(self, category, msg):
        from sensor_msgs.msg import JointState
        indices = None
        name_count = len(msg.name)
        for group, names in self.groups.items():
            pub = self.publishers[f'/{category}/{group}/joint_states']
            if not _has_subscribers(pub):
                continue
            if indices is None:
                indices = {name: i for i, name in enumerate(msg.name)}
            # Do not fabricate absent joints or mix stale samples into a group.
            if not all(name in indices for name in names):
                continue
            names_list = list(names)
            kwargs = {'header': msg.header, 'name': names_list}
            for field in ('position', 'velocity', 'effort'):
                values = getattr(msg, field)
                if len(values) == name_count:
                    kwargs[field] = [values[indices[name]] for name in names_list]
            pub.publish(JointState(**kwargs))

    def poses(self, category, msg):
        from geometry_msgs.msg import PoseStamped
        if len(msg.poses) < 2:
            return
        for index, side in enumerate(('right', 'left')):
            pub = self.publishers[f'/{category}/{side}_arm/ee_pose']
            if _has_subscribers(pub):
                pub.publish(PoseStamped(header=msg.header, pose=msg.poses[index]))

    def base(self, msg):
        from geometry_msgs.msg import TwistStamped
        pub = self.publishers['/action/base/velocity']
        if not _has_subscribers(pub):
            return
        if len(msg.data) != 3 or not all(math.isfinite(v) for v in msg.data):
            return
        out = TwistStamped()
        # Source Float64MultiArray has no header; this is bridge receipt time.
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = 'base_command'
        out.twist.angular.z, out.twist.linear.x, out.twist.linear.y = msg.data
        pub.publish(out)

    def robot_head(self, msg):
        """Split [actual, target] camera-head poses in the robot base frame."""
        from geometry_msgs.msg import PoseStamped
        if len(msg.poses) < 2:
            return
        for topic, pose in (
            ('/observation/head/pose', msg.poses[0]),
            ('/action/head/pose', msg.poses[1]),
        ):
            pub = self.publishers[topic]
            if _has_subscribers(pub):
                pub.publish(PoseStamped(header=msg.header, pose=pose))

    def head(self, msg):
        from geometry_msgs.msg import PoseStamped
        pub = self.publishers['/observation/headset/pose']
        if not _has_subscribers(pub):
            return
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
            pub.publish(out)
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
