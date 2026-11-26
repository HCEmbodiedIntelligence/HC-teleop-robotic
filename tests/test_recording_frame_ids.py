import json
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped
from mcap.writer import Writer
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState

from middleware.core.lerobot_export import prepare_mcap
from middleware.core.robot_api import RobotApiProfile
from middleware.core.x1_recording import X1_BODY_JOINTS

STATE = '/observation/joint_states'
ACTION = '/action/joint_states'
POSE = '/observation/left_arm/ee_pose'


def project(tmp, frame_ids, *, mode='auto', framed_topic=ACTION):
    tmp.mkdir()
    original = tmp / 'inputs.mcap'
    stage = tmp / 'stage'
    stage.mkdir()
    with original.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        info = {'profile_id': 'x1'}
        if mode == 'live':
            profile = Path(__file__).resolve().parents[1] / 'adapters/robots/x1'
            info['robot_api_info'] = json.dumps(RobotApiProfile(profile).describe_servo_j())
        writer.add_metadata('hc_teleop.session', info)
        joints = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        channels = {name: writer.register_channel(name, 'cdr', joints) for name in (STATE, ACTION)}
        if framed_topic == POSE:
            pose = writer.register_schema('geometry_msgs/msg/PoseStamped', 'ros2msg', b'')
            channels[POSE] = writer.register_channel(POSE, 'cdr', pose)
        for i, frame_id in enumerate(frame_ids):
            ns = 30_000_000_000 + i * 100_000_000
            axes = list(X1_BODY_JOINTS[:14 if i % 2 else 16])
            for topic, channel in channels.items():
                msg = PoseStamped() if topic == POSE else JointState(
                    name=axes, position=[float(i + j) for j in range(len(axes))])
                msg.header.frame_id = frame_id if topic == framed_topic else 'base_link'
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
                writer.add_message(channel, ns, serialize_message(msg), ns)
        writer.finish()
    manifest = prepare_mcap(original, stage, 10, 'test', 'x1',
                            training_schema='shared' if mode == 'shared' else 'auto')
    return json.loads(manifest.read_text())


@pytest.mark.parametrize('mode', ['auto', 'shared', 'live'])
@pytest.mark.parametrize('frame_ids', [('', 'base_link', ''), ('base_link', '', 'base_link')])
def test_named_joints_keep_first_declared_frame_and_identical_samples(tmp_path, frame_ids, mode):
    mixed = project(tmp_path / 'mixed', frame_ids, mode=mode)
    declared = project(tmp_path / 'declared', ('base_link',) * 3, mode=mode)
    assert mixed['source_frame_ids'][ACTION] == 'base_link'
    assert mixed['sync_audit']['joint_frame_id_variants'][ACTION] == ['', 'base_link']
    assert mixed['joint_names'] == declared['joint_names']
    assert mixed['episodes'] == declared['episodes']


def test_all_unspecified_joint_frames_remain_unspecified(tmp_path):
    result = project(tmp_path / 'unstated', ('', ''))
    assert result['source_frame_ids'][ACTION] == ''
    assert ACTION not in result['sync_audit'].get('joint_frame_id_variants', {})


@pytest.mark.parametrize('mode', ['auto', 'shared', 'live'])
def test_blank_between_conflicting_joint_frames_does_not_hide_conflict(tmp_path, mode):
    with pytest.raises(ValueError, match='Reference frame changed.*' + ACTION):
        project(tmp_path / 'conflict', ('base_link', '', 'map'), mode=mode)


def test_pose_empty_frame_remains_significant(tmp_path):
    with pytest.raises(ValueError, match='Reference frame changed.*' + POSE):
        project(tmp_path / 'pose', ('', 'base_link'), framed_topic=POSE)
