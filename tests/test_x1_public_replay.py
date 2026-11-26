import json
from pathlib import Path
from unittest.mock import Mock

import pytest
from mcap.writer import Writer
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

from middleware.core.robot_api import RobotApiProfile
from middleware.core.topic_player import TopicPlayer, _X1_MOTION_REVISION

ACTION = '/action/joint_states'
PUBLISHED = '/internal/recording/published_action'
STATE = '/observation/joint_states'


@pytest.fixture
def replay(tmp_path):
    api = RobotApiProfile(Path(__file__).resolve().parents[1] / 'adapters/robots/x1')
    active = {'robot_api_info': json.dumps(api.describe_servo_j())}
    sink = Mock()
    player = TopicPlayer(tmp_path, lambda: sink, active_profile_provider=lambda: 'x1',
                         active_api_metadata_provider=lambda: active)

    def write(streams, revision=_X1_MOTION_REVISION, mode='corrections_only', record_only=None, selected=None):
        with (tmp_path/'old.mcap').open('wb') as stream:
            writer = Writer(stream)
            writer.start(profile='ros2')
            writer.add_metadata('hc_teleop.session', dict(profile_id='x1', robot_api_schema='hc-robot-api/v1',
                                robot_api_revision=revision, capture_mode=mode))
            if record_only:
                writer.add_metadata('hc_teleop.record_only', {'topic': record_only})
            schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
            for i, (topic, names, positions) in enumerate(streams):
                channel = writer.register_channel(topic, 'cdr', schema)
                msg = JointState(name=names, position=positions, velocity=[.2]*len(names))
                writer.add_message(channel, 1000+i, serialize_message(msg), 1000+i)
            writer.finish()
        return player.play('old.mcap', mode='drive', selected_topics=selected)

    yield write, player, sink
    player.stop(require_reset=False)


def test_verified_published_target_precedes_training_label(replay):
    write, player, sink = replay
    write([(ACTION, ['left_joint_2'], [float('nan')]),
           (PUBLISHED, ['right_joint_4', 'left_joint_2'], [-.3, .1])])
    player._thread.join(3)
    assert player.status()['state'] == 'completed'
    topic, msg_type, data = sink.publish_raw.call_args.args
    assert sink.publish_raw.call_count == 1 and topic == ACTION
    msg = deserialize_message(data, JointState)
    assert msg.name == ['Joint4_R', 'Joint2_L'] and list(msg.position) == [-.3, .1]
    assert list(msg.velocity) == [.2, .2]


@pytest.mark.parametrize('topic,revision,mode,error', [
    (STATE, _X1_MOTION_REVISION, 'full_session', 'recorded final actions'),
    (ACTION, 'unknown', 'corrections_only', 'mapping revision'),
    (ACTION, _X1_MOTION_REVISION, '', 'recorded final actions'),
])
def test_unverified_drive_fails_before_arming(replay, topic, revision, mode, error):
    write, _, sink = replay
    with pytest.raises(ValueError, match=error):
        write([(topic, ['left_joint_1'], [0.])], revision, mode)
    sink.begin_replay.assert_not_called()


def test_unknown_historical_joint_reports_playback_error(replay):
    write, player, sink = replay
    write([(ACTION, ['left_gripper_joint_1'], [.3])])
    player._thread.join(3)
    assert player.status()['state'] == 'error'
    assert 'unverified' in player.status()['error']
    sink.publish_raw.assert_not_called()


@pytest.mark.parametrize('streams,expect_drive', [
    ([(STATE, ['Joint1_L'], [0.])], False),
    ([(STATE, ['Joint1_L'], [0.]), ('/action/left_hand/joint_states', ['L_ban'], [1.])], False),
    ([(ACTION, ['left_joint_1'], [0.])], False),
    ([(PUBLISHED, None, []), (ACTION, ['Joint1_L'], [.2])], False),
    ([(ACTION, ['Joint1_L'], [.2])], True),
    ([(PUBLISHED, ['Joint1_L'], [.2]), (ACTION, ['Joint1_L'], [float('nan')])], True),
])
def test_platform_selected_file_with_no_session_metadata(tmp_path, streams, expect_drive):
    with (tmp_path/'selected.mcap').open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        for i, (topic, names, positions) in enumerate(streams):
            channel = writer.register_channel(topic, 'cdr', schema)
            if names:
                data = serialize_message(JointState(name=names, position=positions))
                writer.add_message(channel, i+1, data, i+1)
        writer.finish()
    sink = Mock()
    player = TopicPlayer(tmp_path, lambda: sink, active_profile_provider=lambda: 'x1')
    try:
        if not expect_drive:
            with pytest.raises(ValueError, match='recorded final actions|historical mapping revision'):
                player.play('selected.mcap', mode='drive')
            sink.begin_replay.assert_not_called()
        else:
            player.play('selected.mcap', mode='drive')
            player._thread.join(3)
            sink.publish_raw.assert_called_once()
            topic, _, raw = sink.publish_raw.call_args.args
            assert topic == ACTION and list(deserialize_message(raw, JointState).position) == [.2]
    finally:
        player.stop(require_reset=False)


def test_explicit_public_drive_refuses_record_only_source(replay):
    write, _, sink = replay
    with pytest.raises(ValueError, match='recorded final actions'):
        write([(ACTION, ['left_joint_1'], [.1])], record_only=ACTION, selected=[ACTION])
    sink.begin_replay.assert_not_called()
