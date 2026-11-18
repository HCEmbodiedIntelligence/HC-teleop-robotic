"""Old JSON event bags and ROS String/CDR event bags retain identical meaning."""
import json

import pytest
from mcap.reader import make_reader
from mcap.writer import Writer
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from middleware.core.lerobot_export import prepare_mcap
from middleware.core.topic_recorder import _get_msg_def
from tools.diagnostics.audit_mcap_timing import (
    ACTION, ACTION_PROVENANCE, PUBLISHED_ACTION, audit,
)


def event_channel(writer, topic, encoding):
    if encoding == 'json':
        schema = writer.register_schema('teleop_event', 'jsonschema', b'{}')
    else:
        schema = writer.register_schema('std_msgs/msg/String', 'ros2msg', b'string data\n')
    return writer.register_channel(topic, encoding, schema)


def event_bytes(payload, encoding):
    text = json.dumps(payload, ensure_ascii=False)
    return text.encode() if encoding == 'json' else bytes(serialize_message(String(data=text)))


@pytest.mark.parametrize('encoding', ['json', 'cdr'])
@pytest.mark.parametrize('capture_mode', ['full_session', 'corrections_only'])
def test_export_preserves_interventions_and_initial_observation_for_both_event_formats(
        tmp_path, encoding, capture_mode):
    source = tmp_path / 'events.mcap'
    origin = 1_000_000_000
    interval = dict(key='human-one', start_ns=origin+50_000_000, end_ns=origin+150_000_000)
    event = dict(kind='session_close', capture_mode=capture_mode, start_ns=origin,
                 end_ns=origin+200_000_000, interventions=[interval])
    if capture_mode == 'corrections_only':
        event.update(kind='manual_close', key='human-one', quality={'first_action_ns': origin})
    initial = dict(role='initial_observation', topic='/observation/joint_states',
                   source_stamp_ns=origin-1_000_000)
    with source.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        joint_schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg',
                                               _get_msg_def('sensor_msgs/msg/JointState'))
        channels = {topic: writer.register_channel(topic, 'cdr', joint_schema)
                    for topic in ['/observation/joint_states', ACTION]}
        intervention = event_channel(writer, '/events/control/intervention', encoding)
        provenance = event_channel(writer, '/events/recording/sample_provenance', encoding)
        writer.add_message(provenance, origin-1_000_000, event_bytes(initial, encoding), origin-1_000_000)
        for index in range(3):
            stamp = origin+index*100_000_000
            message = JointState(name=['arm'], position=[index*.1])
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            raw = bytes(serialize_message(message))
            for channel in channels.values():
                writer.add_message(channel, stamp, raw, stamp)
        writer.add_message(intervention, event['end_ns'], event_bytes(event, encoding), event['end_ns'])
        writer.finish()
    staging = tmp_path / 'staging'
    staging.mkdir()
    manifest = json.loads(prepare_mcap(source, staging, 10, 'test', 'fake').read_text())
    if capture_mode == 'full_session':
        assert manifest['session'] == event
        assert 'correction' not in manifest
        rows = manifest['episodes'][0]
        assert [row['intervention'] for row in rows] == [False, True, False]
    else:
        assert manifest['correction'] == event
        assert manifest['initial_observations'] == {initial['topic']: initial}
        assert [row['grid_time'] for episode in manifest['episodes'] for row in episode] == [.001, .101]


@pytest.mark.parametrize('encoding', ['json', 'cdr'])
def test_timing_audit_accepts_both_action_provenance_formats(tmp_path, encoding):
    source = tmp_path / 'provenance.mcap'
    stamp = 1_000_000_000
    marker = dict(schema_version='hc-action-provenance/v1', source='published_action',
                  action_log_time_ns=stamp, publication_ns=stamp, sent_to_robot=True,
                  included_in_action_labels=True)
    with source.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        writer.add_metadata('hc_teleop.action_labels', {'json': '{}'})
        schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg',
                                        _get_msg_def('sensor_msgs/msg/JointState'))
        message = JointState(name=['arm'], position=[.1])
        message.header.stamp.sec = 1
        for topic in (ACTION, PUBLISHED_ACTION):
            channel = writer.register_channel(topic, 'cdr', schema)
            writer.add_message(channel, stamp, bytes(serialize_message(message)), stamp)
        channel = event_channel(writer, ACTION_PROVENANCE, encoding)
        writer.add_message(channel, stamp, event_bytes(marker, encoding), stamp)
        writer.finish()
    report = audit(source)
    assert report['label_provenance'] == dict(
        sources={'published_action': 1}, invalid=0, unmarked_action_frames=0,
        markers_without_action=0, publication_marker_mismatch=0)


@pytest.mark.parametrize('encoding', ['json', 'cdr'])
def test_video_bundle_keeps_event_schema_encoding_and_payload(tmp_path, encoding):
    from middleware.core.video_bundle import prepare
    source = tmp_path / 'source.mcap'
    payload = event_bytes({'kind': 'manual_close', 'reason': '人工接管'}, encoding)
    with source.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        channel = event_channel(writer, '/events/control/intervention', encoding)
        writer.add_message(channel, 1_000_000_000, payload, 999_000_000)
        writer.finish()
    bundle = prepare(source, tmp_path / 'bundle', dict(
        recording_id='events', started_at='2026-11-18T00:00:00Z', ended_at='2026-11-18T00:00:01Z'))
    with (bundle / 'raw.mcap').open('rb') as stream:
        rows = list(make_reader(stream, validate_crcs=True).iter_messages())
    assert len(rows) == 1
    schema, channel, message = rows[0]
    assert (channel.message_encoding, message.data, message.log_time, message.publish_time) == (
        encoding, payload, 1_000_000_000, 999_000_000)
    assert schema.name == ('teleop_event' if encoding == 'json' else 'std_msgs/msg/String')
    assert schema.encoding == ('jsonschema' if encoding == 'json' else 'ros2msg')
