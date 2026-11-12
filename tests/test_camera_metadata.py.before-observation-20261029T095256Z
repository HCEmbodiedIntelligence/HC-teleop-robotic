import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Header, String

from middleware.core.camera_metadata import (
    CALIBRATION_TOPIC, EXTRINSICS_TOPIC, CameraKinematics, CameraMetadata, load_camera_metadata,
)


ROOT = Path(__file__).resolve().parents[1]
PROFILE_CONFIG = {'robot_profiles': {'root': str(ROOT / 'adapters/robots'), 'active': 'x1'}}


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self):
        self.qos = {}

    def create_publisher(self, cls, topic, qos):
        self.qos[topic] = qos
        return Publisher()

    def create_subscription(self, cls, topic, callback, qos, **kwargs):
        return topic, callback

    def create_timer(self, period, callback):
        assert period == 0.5
        return callback

    def destroy_subscription(self, subscription):
        return True

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))


@pytest.fixture
def metadata(monkeypatch):
    monkeypatch.setenv('HC_CONTROL_BACKEND', 'v23')
    monkeypatch.delenv('HC_ROBOT_NAME', raising=False)
    monkeypatch.delenv('HC_ROBOT_CONFIG_ROOT', raising=False)
    return CameraMetadata(Node(), load_camera_metadata(PROFILE_CONFIG, ROOT / 'middleware'))


def info(frame='color_optical', fx=400., stamp=100):
    return CameraInfo(header=Header(stamp=Time(sec=stamp), frame_id=frame),
                      width=640, height=480, distortion_model='plumb_bob', d=[0.] * 5,
                      k=[fx, 0., 320., 0., 401., 240., 0., 0., 1.])


def joints(stamp=100, names=None, positions=None):
    return JointState(header=Header(stamp=Time(sec=stamp)),
                      name=names if names is not None else ['head_1', 'head_2'],
                      position=positions if positions is not None else [0.1, -0.2])


def test_rgb_only_then_depth_preserves_calibration_and_source(metadata):
    rgb, depth = (metadata.config['camera_info_topics'][s][0] for s in ('rgb', 'depth'))
    metadata.camera_info('rgb', rgb, info())
    value = metadata.calibration(time.monotonic())
    assert value['schema'] == 'hc_camera_calibration/v1' and value['camera'] == 'eye'
    assert value['resolution'] == {'color': {'width': 640, 'height': 480}}
    assert value['rgb']['fx'] == 400. and value['rgb']['fy'] == 401.
    assert value['rgb']['distortion_order'] == ['k1', 'k2', 'p1', 'p2', 'k3']
    assert value['rgb']['frame_id'] == 'color_optical'
    assert value['rgb']['stamp'] == {'sec': 100, 'nanosec': 0}
    assert 'depth' not in value and 'rgb_depth_extrinsics' not in value
    metadata.camera_info('depth', depth, info('depth_optical', 390.))
    value = metadata.calibration(time.monotonic())
    assert value['depth']['fx'] == 390.
    assert value['depth']['camera_info_topic'] == depth


@pytest.mark.parametrize('fx', [0., float('nan'), float('inf')])
def test_bad_intrinsics_remove_previously_valid_stream(metadata, fx):
    topic = metadata.config['camera_info_topics']['rgb'][0]
    metadata.camera_info('rgb', topic, info())
    metadata.camera_info('rgb', topic, info(fx=fx))
    assert metadata.calibration(time.monotonic()) is None


def test_info_source_priority_and_stale_source_fallback(metadata):
    primary, fallback = metadata.config['camera_info_topics']['rgb']
    metadata.camera_info('rgb', primary, info(fx=400.))
    metadata.camera_info('rgb', fallback, info(fx=410.))
    assert metadata.calibration(time.monotonic())['rgb']['fx'] == 400.
    message, received = metadata.infos['rgb'][primary]
    metadata.infos['rgb'][primary] = (message, received - 2.)
    assert metadata.calibration(time.monotonic())['rgb']['fx'] == 410.
    assert metadata.calibration(time.monotonic() + 2.) is None


def test_no_sources_no_fake_metadata(metadata):
    metadata.publish()
    assert all(not p.messages for p in metadata.publishers.values())


def test_urdf_extrinsics_use_feedback_and_change_with_head(metadata):
    metadata.joint_state(joints())
    first = metadata.extrinsics(time.monotonic())
    assert first['schema'] == 'hc_robot_camera_extrinsics/v1'
    assert first['common_frame'] == 'zhi_Link' and first['source'] == 'urdf_joint_states'
    transform = first['transforms'][0]
    assert transform['parent_frame'] == 'zhi_Link' and transform['child_frame'] == 'camera_head'
    assert np.linalg.norm(transform['rotation_xyzw']) == pytest.approx(1.)
    metadata.joint_state(joints(positions=[0.3, -0.4]))
    second = metadata.extrinsics(time.monotonic())
    assert second['transforms'][0] != transform
    assert metadata.extrinsics(time.monotonic() + 2.) is None


@pytest.mark.parametrize('message', [
    joints(names=['head_1'], positions=[0.1]),
    joints(names=['head_1', 'head_1'], positions=[0.1, 0.2]),
    joints(positions=[0.1]),
    joints(positions=[float('nan'), 0.2]),
    joints(stamp=98), joints(stamp=102),
])
def test_partial_invalid_and_stale_feedback_never_reuses_previous_pose(metadata, message):
    metadata.joint_state(joints())
    metadata.joint_state(message)
    assert metadata.extrinsics(time.monotonic()) is None


def test_tf_is_preferred_and_preserves_transform_direction(metadata):
    transform = TransformStamped(header=Header(stamp=Time(sec=100), frame_id='zhi_Link'),
                                 child_frame_id='camera_head')
    transform.transform.translation.x = 1.2
    transform.transform.rotation.w = 1.
    metadata.tf_buffer.set_transform(transform, 'test')
    metadata.joint_state(joints())
    value = metadata.extrinsics(time.monotonic())
    assert value['source'] == 'tf'
    assert value['transforms'][0]['translation_m'] == [1.2, 0., 0.]
    assert value['stamp'] == {'sec': 100, 'nanosec': 0}


def test_stale_tf_falls_back_to_fresh_measured_joints(metadata):
    transform = TransformStamped(header=Header(stamp=Time(sec=98), frame_id='zhi_Link'),
                                 child_frame_id='camera_head')
    transform.transform.rotation.w = 1.
    metadata.tf_buffer.set_transform(transform, 'test')
    assert metadata.extrinsics(time.monotonic()) is None
    metadata.joint_state(joints())
    assert metadata.extrinsics(time.monotonic())['source'] == 'urdf_joint_states'


def test_legacy_messages_remain_authoritative_without_duplicate_fallback(metadata):
    metadata.joint_state(joints())
    legacy = String(data='{"schema":"hc_robot_camera_extrinsics/v1","legacy":true}')
    metadata.relay(EXTRINSICS_TOPIC, legacy)
    metadata.publish()
    assert metadata.publishers[EXTRINSICS_TOPIC].messages == [legacy]
    metadata.legacy_received[EXTRINSICS_TOPIC] -= 2.
    metadata.publish()
    values = metadata.publishers[EXTRINSICS_TOPIC].messages
    assert len(values) == 2
    assert json.loads(values[-1].data)['source'] == 'urdf_joint_states'


def test_unified_public_head_names_map_back_to_model(monkeypatch, metadata):
    monkeypatch.setenv('HC_CONTROL_BACKEND', 'motion')
    config = load_camera_metadata(PROFILE_CONFIG, ROOT / 'middleware')
    unified = CameraMetadata(Node(), config)
    unified.joint_state(joints(names=['head_joint_1', 'head_joint_2']))
    metadata.joint_state(joints())
    assert unified.extrinsics(time.monotonic()) == metadata.extrinsics(time.monotonic())


def test_x1_kinematics_matches_independent_pybullet_model(metadata):
    import pybullet as bullet
    client = bullet.connect(bullet.DIRECT)
    try:
        robot = bullet.loadURDF(metadata.config['urdf_path'], useFixedBase=True, physicsClientId=client)
        links = {}
        positions = {'head_1': 0.3, 'head_2': -0.4, 'zhi': 0.1, 'leg_1': 0.5, 'leg_2': 0.8}
        for index in range(bullet.getNumJoints(robot, physicsClientId=client)):
            joint = bullet.getJointInfo(robot, index, physicsClientId=client)
            links[joint[12].decode()] = index
            if joint[1].decode() in positions:
                bullet.resetJointState(robot, index, positions[joint[1].decode()], physicsClientId=client)
        parent = bullet.getLinkState(robot, links['zhi_Link'], computeForwardKinematics=True, physicsClientId=client)
        child = bullet.getLinkState(robot, links['camera_head'], computeForwardKinematics=True, physicsClientId=client)
        inverse = bullet.invertTransform(parent[4], parent[5])
        translation, rotation = bullet.multiplyTransforms(*inverse, child[4], child[5])
        actual_translation, actual_rotation = metadata.kinematics.transform(positions)
        np.testing.assert_allclose(actual_translation, translation, atol=2e-6)
        assert abs(np.dot(actual_rotation, rotation)) == pytest.approx(1., abs=2e-6)
    finally:
        bullet.disconnect(client)


def test_prismatic_and_rotated_origins_are_composed_in_parent_frame(tmp_path):
    path = tmp_path / 'camera.urdf'
    path.write_text('''<robot name="test"><link name="base"/><link name="slide"/><link name="camera"/>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="slide"/>
        <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/></joint>
      <joint name="mount" type="fixed"><parent link="slide"/><child link="camera"/>
        <origin xyz="0 1 0"/></joint></robot>''')
    chain = CameraKinematics(path, 'base', 'camera')
    assert chain.transform({}) is None
    position, rotation = chain.transform({'slide': 2.})
    np.testing.assert_allclose(position, [0., 2., 0.], atol=1e-12)
    np.testing.assert_allclose(rotation, [0., 0., math.sqrt(.5), math.sqrt(.5)], atol=1e-12)


def test_other_profiles_do_not_guess_camera_mounts(monkeypatch):
    monkeypatch.delenv('HC_ROBOT_NAME', raising=False)
    monkeypatch.delenv('HC_ROBOT_CONFIG_ROOT', raising=False)
    config = {'robot_profiles': {**PROFILE_CONFIG['robot_profiles'], 'active': 'openarmx'}}
    assert load_camera_metadata(config, ROOT / 'middleware') == {}
