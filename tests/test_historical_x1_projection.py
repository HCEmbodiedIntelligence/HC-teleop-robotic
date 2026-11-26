import json

import pytest
from mcap.writer import Writer
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState

from middleware.core.historical_x1_projection import historical_body
from middleware.core.lerobot_export import prepare_mcap
from middleware.core.recording_action_labels import ACTION_TOPIC, FEEDBACK_TOPIC, PROVENANCE_TOPIC
from middleware.core.x1_recording import X1_BODY_JOINTS


def test_shared_rejects_unidentified_or_frozen_history():
    with pytest.raises(ValueError, match='explicitly identified'):
        historical_body('shared', None, True, {})
    with pytest.raises(ValueError, match='ordinary'):
        historical_body('shared', None, False, {'profile_id': 'x1'})
    assert historical_body('auto', None, False, {}) is None


def test_historical_labels_require_matched_publication_and_keep_default(tmp_path):
    body = X1_BODY_JOINTS
    kinds = ['feedback_hold', 'published_action', 'unverified', 'mismatched_stamp']
    source = tmp_path / 'history.mcap'
    start = 35_000_000_000
    with source.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        writer.add_metadata('hc_teleop.session', {'profile_id': 'x1'})
        joint = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        state = writer.register_channel(FEEDBACK_TOPIC, 'cdr', joint)
        action = writer.register_channel(ACTION_TOPIC, 'cdr', joint)
        hand = writer.register_channel('/action/right_hand/joint_states', 'cdr', joint)
        marker = writer.register_channel(PROVENANCE_TOPIC, 'json', 0)
        for i, kind in enumerate(kinds):
            stamp = start + i * 400_000_000
            def write(channel, names, positions):
                msg = JointState(name=names, position=positions)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                writer.add_message(channel, stamp, serialize_message(msg), stamp)
            write(state, list(reversed(body)), list(reversed([float(n) for n in range(17)])))
            write(action, ['R_ban', body[6]], [1.57, 4.+i])
            write(hand, ['R_ban'], [float(i % 2)])
            if kind != 'unverified':
                published = kind != 'feedback_hold'
                payload = dict(schema_version='hc-action-provenance/v1', source='published_action' if published else kind,
                               action_topic=ACTION_TOPIC, source_topic=ACTION_TOPIC if published else FEEDBACK_TOPIC,
                               source_stamp_ns=stamp + int(kind == 'mismatched_stamp'), action_log_time_ns=stamp,
                               publication_ns=stamp if published else None, sent_to_robot=published,
                               included_in_action_labels=True)
                writer.add_message(marker, stamp, json.dumps(payload).encode(), stamp)
        writer.finish()

    def prepare(mode):
        stage = tmp_path / mode
        stage.mkdir()
        return json.loads(prepare_mcap(source, stage, 10, 'x1', 'x1',
                                       record_hands='both', training_schema=mode).read_text())
    auto, shared = prepare('auto'), prepare('shared')
    assert 'live_source_keys' not in auto
    assert auto['episodes'][0][0]['values']['action'][6] == 4.
    assert shared['live_source_keys'] == ['action', 'observation.state']
    assert shared['joint_units']['action'] == ['rad']*17 + ['1','1']
    assert shared['joint_names'] == list(body) + ['left_grip','right_grip']
    assert shared['sync_audit']['historical_x1_projection']['raw_label_counts'] == {
        'published_action': 1, 'feedback_hold': 1, 'unverified': 2}
    source_ranges = shared['sync_audit']['historical_x1_projection']['raw_label_origin_ranges']
    assert [r['source'] for r in source_ranges] == ['feedback_hold', 'missing', 'published_action', 'missing',
                                                    'unverified', 'missing', 'unverified']
    for i in range(4):
        row = shared['episodes'][0][4*i]
        assert row['observation.timestamp_ns'] == [start + 400_000_000*i]
        assert row['values']['observation.state'][:17] == list(range(17))
        assert row['state_valid'] == [True]*17+[False,False]
        assert row['action_valid'][6] == (i == 1)
        assert row['action_valid'][-2:] == [False,True]
        assert row['action.source_time_ns'][6] == (-1 if i == 0 else start + 400_000_000)
        assert row['values']['action'][-1] == float(i % 2)


def test_shared_batch_keeps_default_dataset_and_selected_hands(tmp_path):
    from pathlib import Path
    from unittest.mock import patch
    from middleware.core.topic_recorder import TopicRecorder

    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path)
    (tmp_path / 'history.mcap').touch()
    existing = tmp_path / 'lerobot/history/meta'
    existing.mkdir(parents=True)
    (existing / 'info.json').write_text('{}')
    try:
        with patch('middleware.core.lerobot_export.ensure_export_available'), \
                patch('middleware.core.topic_recorder.read_recording_metadata', return_value={'profile_id':'x1'}), \
                patch('middleware.core.topic_recorder.subprocess.Popen') as spawn:
            result = recorder.convert_recordings_to_lerobot(['history.mcap'], 'shared', 'both')
        assert result['accepted'] == ['history.mcap']
        item = json.loads(Path(spawn.call_args.args[0][-1]).read_text())['items'][0]
        assert item['record_hands'] == 'both'
        assert item['training_schema'] == 'shared'
        assert Path(item['destination']).name == 'history.shared-both'
        assert (existing / 'info.json').read_text() == '{}'
    finally:
        recorder.shutdown()
