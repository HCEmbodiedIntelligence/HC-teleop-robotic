"""Restore camera calibration and robot-relative extrinsics observation topics."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from xml.etree import ElementTree


CALIBRATION_TOPIC = '/observation/cameras/head/calibration'
EXTRINSICS_TOPIC = '/observation/cameras/head/extrinsics'


def load_camera_metadata(config, config_dir):
    """Only profiles with an explicit camera frame opt into model extrinsics."""
    import yaml
    from adapters.core.runtime_config import unified_profile
    from .robot_api import RobotApiProfile, selected_profile

    directory = selected_profile(config, config_dir)
    teleop = yaml.safe_load((directory / 'arm_teleop.yaml').read_text())
    settings = dict(teleop.get('camera_metadata', {}))
    if not settings.get('enabled', False):
        return {}
    settings['urdf_path'] = str((directory / teleop['robot']['urdf_path']).resolve())
    settings['joint_mapping'] = {}
    if unified_profile(directory):
        api = RobotApiProfile(directory)
        settings['joint_mapping'] = {
            joint['public_name']: (name, joint['scale'], joint['offset'])
            for name, joint in api.joints.items()
        }
    return settings


class CameraKinematics:
    """Evaluate only the configured parent-to-camera chain from measured joints.

    This returns the URDF camera frame, not an inferred driver optical frame.
    Missing joints are never replaced with home positions or cached samples.
    """
    def __init__(self, urdf_path, parent, child):
        from adapters.core.arm_teleop_math import quaternion_from_axis_angle, quaternion_multiply

        model = ElementTree.fromstring(Path(urdf_path).read_bytes())
        by_child = {j.find('child').get('link'): j for j in model.findall('joint')}
        self.chain = []
        visited = set()
        while child != parent:
            if child in visited or child not in by_child:
                raise ValueError(f'camera frame is not a descendant of {parent}')
            visited.add(child)
            joint = by_child[child]
            kind = joint.get('type')
            if kind not in {'fixed', 'revolute', 'continuous', 'prismatic'} or joint.find('mimic') is not None:
                raise ValueError('unsupported joint in camera chain: ' + joint.get('name'))
            origin = joint.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis = joint.find('axis')
            axis = [float(v) for v in (axis.get('xyz', '1 0 0') if axis is not None else '1 0 0').split()]
            if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in (xyz, rpy, axis)):
                raise ValueError('invalid camera joint origin or axis')
            norm = math.sqrt(sum(v * v for v in axis))
            if norm == 0:
                raise ValueError('camera joint axis is zero')
            axis = [v / norm for v in axis]
            rotation = quaternion_multiply(
                quaternion_from_axis_angle([0, 0, 1], rpy[2]),
                quaternion_multiply(quaternion_from_axis_angle([0, 1, 0], rpy[1]),
                                    quaternion_from_axis_angle([1, 0, 0], rpy[0])))
            self.chain.append((joint.get('name'), kind, xyz, rotation, axis))
            child = joint.find('parent').get('link')
        self.chain.reverse()

    def transform(self, positions):
        import numpy as np
        from adapters.core.arm_teleop_math import (
            quaternion_from_axis_angle, quaternion_multiply, quaternion_to_matrix,
        )

        translation, rotation = np.zeros(3), np.array([0., 0., 0., 1.])
        for name, kind, xyz, origin_rotation, axis in self.chain:
            translation += quaternion_to_matrix(rotation) @ xyz
            rotation = quaternion_multiply(rotation, origin_rotation)
            if kind == 'fixed':
                continue
            position = positions.get(name)
            if position is None or not math.isfinite(position):
                return None
            if kind == 'prismatic':
                translation += quaternion_to_matrix(rotation) @ (np.asarray(axis) * position)
            else:
                rotation = quaternion_multiply(rotation, quaternion_from_axis_angle(axis, position))
        return translation.tolist(), rotation.tolist()


class CameraMetadata:
    def __init__(self, node, config):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from std_msgs.msg import String

        self.node, self.config = node, config
        self.subscriptions = []
        self.infos = {}
        self.joints = None
        self.legacy_received = {}
        self.publishers = {
            CALIBRATION_TOPIC: node.create_publisher(String, CALIBRATION_TOPIC, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            EXTRINSICS_TOPIC: node.create_publisher(String, EXTRINSICS_TOPIC, 10),
        }
        for suffix, target in [('calibration', CALIBRATION_TOPIC), ('extrinsics', EXTRINSICS_TOPIC)]:
            self.subscriptions.append(node.create_subscription(
                String, f'/io_teleop/camera_head/{suffix}',
                lambda msg, topic=target: self.relay(topic, msg), qos_profile_sensor_data))
        if not config.get('enabled', False):
            return

        from sensor_msgs.msg import CameraInfo, JointState
        from tf2_ros import Buffer, TransformListener

        self.max_age = float(config.get('max_age_sec', 1.0))
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('camera_metadata.max_age_sec must be positive and finite')
        self.kinematics = CameraKinematics(config['urdf_path'], config['common_frame'], config['camera_frame'])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        for stream, topics in config.get('camera_info_topics', {}).items():
            if stream not in {'rgb', 'depth'}:
                raise ValueError('camera_info_topics only supports rgb and depth')
            if not isinstance(topics, list) or any(not isinstance(t, str) or not t.startswith('/') for t in topics):
                raise ValueError('camera_info_topics must contain absolute topic lists')
            for topic in topics:
                self.subscriptions.append(node.create_subscription(
                    CameraInfo, topic, lambda msg, s=stream, t=topic: self.camera_info(s, t, msg),
                    qos_profile_sensor_data))
        self.subscriptions.append(node.create_subscription(
            JointState, '/observation/joint_states', self.joint_state, qos_profile_sensor_data))
        self.timer = node.create_timer(0.5, self.publish)

    def relay(self, topic, message):
        # Existing HC_X1 drivers remain authoritative while they are publishing.
        self.legacy_received[topic] = time.monotonic()
        self.publishers[topic].publish(message)

    def camera_info(self, stream, topic, message):
        self.infos.setdefault(stream, {})[topic] = (message, time.monotonic())

    def joint_state(self, message):
        self.joints = None
        if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                or not all(math.isfinite(v) for v in message.position)):
            return
        mapping = self.config.get('joint_mapping', {})
        positions = {}
        for name, value in zip(message.name, message.position):
            if mapping:
                if name not in mapping:
                    continue
                name, scale, offset = mapping[name]
                value = value * scale + offset
            positions[name] = value
        self.joints = (positions, message.header.stamp, time.monotonic())

    def _fresh(self, received, stamp, now):
        if not 0 <= now - received <= self.max_age:
            return False
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        # Unstamped driver messages use receipt freshness, preserving their zero stamp.
        if not stamp_ns:
            return True
        age = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return -0.1 <= age <= self.max_age

    def calibration(self, now):
        document = {'schema': 'hc_camera_calibration/v1', 'camera': self.config.get('camera', 'eye'),
                    'source': 'sensor_msgs/CameraInfo', 'resolution': {}}
        for stream, candidates in self.infos.items():
            # The configured list defines priority; do not blend two publishers
            # of the same stream just because their callbacks alternate.
            for topic in self.config['camera_info_topics'][stream]:
                if topic not in candidates:
                    continue
                info, received = candidates[topic]
                if not self._fresh(received, info.header.stamp, now):
                    continue
                k, d = list(info.k), list(info.d)
                if (len(k) != 9 or not all(math.isfinite(v) for v in k + d)
                        or k[0] <= 0 or k[4] <= 0 or info.width <= 0 or info.height <= 0):
                    continue
                value = {'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5],
                         'distortion_model': info.distortion_model, 'distortion_coefficients': d,
                         'frame_id': info.header.frame_id, 'camera_info_topic': topic,
                         'stamp': {'sec': info.header.stamp.sec, 'nanosec': info.header.stamp.nanosec}}
                if info.distortion_model == 'plumb_bob' and len(d) == 5:
                    value['distortion_order'] = ['k1', 'k2', 'p1', 'p2', 'k3']
                document[stream] = value
                document['resolution']['color' if stream == 'rgb' else stream] = {
                    'width': info.width, 'height': info.height}
                break
        return document if document['resolution'] else None

    def extrinsics(self, now):
        from rclpy.time import Time
        from tf2_ros import TransformException

        parent, child = self.config['common_frame'], self.config['camera_frame']
        result = None
        try:
            transform = self.tf_buffer.lookup_transform(parent, child, Time())
            stamp = transform.header.stamp
            if self._fresh(now, stamp, now):
                t, q = transform.transform.translation, transform.transform.rotation
                result = ([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                source = 'tf'
        except TransformException:
            pass
        if result is None and self.joints is not None:
            positions, stamp, received = self.joints
            if self._fresh(received, stamp, now):
                result = self.kinematics.transform(positions)
                source = 'urdf_joint_states'
        if result is None:
            return None
        translation, rotation = result
        if not all(math.isfinite(v) for v in translation + rotation):
            return None
        norm = math.sqrt(sum(v * v for v in rotation))
        if norm < 1e-9:
            return None
        return {'schema': 'hc_robot_camera_extrinsics/v1',
                'camera': self.config.get('camera', 'eye'), 'common_frame': parent,
                'source': source, 'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                'transforms': [{'parent_frame': parent, 'child_frame': child,
                                'translation_m': translation,
                                'rotation_xyzw': [v / norm for v in rotation]}]}

    def publish(self):
        from std_msgs.msg import String

        now = time.monotonic()
        for topic, build in [(CALIBRATION_TOPIC, self.calibration), (EXTRINSICS_TOPIC, self.extrinsics)]:
            if now - self.legacy_received.get(topic, -math.inf) <= self.max_age:
                continue
            document = build(now)
            if document is not None:
                self.publishers[topic].publish(String(data=json.dumps(document, allow_nan=False, separators=(',', ':'))))
