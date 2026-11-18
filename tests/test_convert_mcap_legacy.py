"""Historical MCAP conversion preserves all evidence except event wire encoding."""
import hashlib
import json
import struct
from pathlib import Path

import pytest
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

from middleware.core.recording_cdr import position_label, read_string, write_string
from tools.diagnostics.convert_mcap_legacy import convert, verify


JSON_TEXT = '{ "source": "feedback_hold", "说明": "人工恢复", "exact_ns": 1794963234567890123 }'
JOINT_SCHEMA = b'std_msgs/Header header\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_fixture(path, *, event_encoding='json', invalid_json=False):
    with path.open('wb') as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start(profile='ros2', library='legacy-robot-recorder/custom')
        joints = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', JOINT_SCHEMA)
        unused = writer.register_schema('std_msgs/msg/Bool', 'ros2msg', b'bool data\n')
        writer.register_channel('/silent', 'cdr', unused, {'declared': 'yes'})
        # These three independent channels intentionally share a topic. Even the
        # two with identical schema+metadata must remain separate channels.
        channels = [writer.register_channel('/action/joint_states', 'cdr', joints, metadata)
                    for metadata in ({'publisher': 'one'}, {'publisher': 'two'}, {'publisher': 'one'})]
        if event_encoding == 'json':
            event_schema = writer.register_schema('teleop_event', 'jsonschema', b'{"type":"object"}')
        else:
            event_schema = writer.register_schema('std_msgs/msg/String', 'ros2msg', b'string data\n')
        events = writer.register_channel('/events/recording/action_provenance', event_encoding,
                                         event_schema, {'origin': 'recorder'})
        writer.add_metadata('hc_teleop.session', {'profile_id': 'x1', 'robot': 'niic'})
        writer.add_metadata('repeat', {'value': 'first'})
        writer.add_metadata('repeat', {'value': 'second'})
        writer.add_attachment(create_time=55, log_time=66, name='calibration.bin',
                              media_type='application/octet-stream', data=b'\0original\xff')
        prefix = b'\0\1\0\0'+struct.pack('<iII', 123, 456, 1)+b'\0'
        raw = position_label(prefix, ['Joint1_R'], [.125])
        for index, channel in enumerate(channels):
            # Deliberately non-monotonic log timestamps: conversion must retain
            # physical message ordering rather than re-sort by log_time.
            stamp = (300, 100, 200)[index]
            writer.add_message(channel, stamp, raw, stamp-7, sequence=101+index)
        text = '{broken JSON' if invalid_json else JSON_TEXT
        event_raw = text.encode('utf-8') if event_encoding == 'json' else write_string(text)
        writer.add_message(events, 250, event_raw, 247, sequence=999)
        writer.finish()


def snapshot(path):
    with path.open('rb') as stream:
        reader = make_reader(stream, validate_crcs=True)
        summary = reader.get_summary()
        header = reader.get_header()
        messages = list(reader.iter_messages(log_time_order=False))
        metadata = [(m.name, m.metadata) for m in reader.iter_metadata()]
        attachments = [(a.create_time, a.log_time, a.name, a.media_type, a.data)
                       for a in reader.iter_attachments()]
    return summary, header, messages, metadata, attachments


def test_conversion_uses_ros2_cdr_and_retains_all_original_evidence(tmp_path):
    source, destination = tmp_path/'mixed.mcap', tmp_path/'legacy.mcap'
    write_fixture(source)
    source_hash = digest(source)
    result = convert(source, destination)
    assert isinstance(result, dict)
    assert isinstance(verify(source, destination), dict)
    assert digest(source) == source_hash
    old, old_header, old_rows, old_metadata, old_attachments = snapshot(source)
    new, new_header, new_rows, new_metadata, new_attachments = snapshot(destination)
    assert (new_header.profile, new_header.library) == (old_header.profile, old_header.library)
    assert new_metadata == old_metadata
    assert new_attachments == old_attachments
    assert len(new.channels) == len(old.channels)
    assert all(schema.encoding == 'ros2msg' for schema in new.schemas.values())
    assert all(channel.message_encoding == 'cdr' for channel in new.channels.values())
    assert any(schema.name == 'std_msgs/msg/Bool' for schema in new.schemas.values())
    silent = next(c for c in new.channels.values() if c.topic == '/silent')
    assert silent.metadata == {'declared': 'yes'}
    assert new.statistics.channel_message_counts.get(silent.id, 0) == 0
    assert len(new_rows) == len(old_rows) == 4
    assert len({c.id for _, c, _ in new_rows if c.topic == '/action/joint_states'}) == 3
    for (old_schema, old_channel, before), (new_schema, new_channel, after) in zip(old_rows, new_rows):
        assert (after.log_time, after.publish_time, after.sequence) == (
            before.log_time, before.publish_time, before.sequence)
        assert (new_channel.topic, new_channel.metadata) == (old_channel.topic, old_channel.metadata)
        if old_channel.message_encoding == 'json':
            assert new_schema.name == 'std_msgs/msg/String'
            # Preserve the exact original JSON text, including whitespace,
            # Unicode and full-width integer values, inside a standard String.
            assert read_string(after.data).encode('utf-8') == before.data
            assert json.loads(read_string(after.data)) == json.loads(before.data)
        else:
            assert (new_schema.name, new_schema.data) == (old_schema.name, old_schema.data)
            assert after.data == before.data


def test_already_ros2_file_is_equivalent_after_copy_conversion(tmp_path):
    source, destination = tmp_path/'already.mcap', tmp_path/'copy.mcap'
    write_fixture(source, event_encoding='cdr')
    original = digest(source)
    assert isinstance(convert(source, destination), dict)
    verify(source, destination)
    assert digest(source) == original
    _, _, before, _, _ = snapshot(source)
    _, _, after, _, _ = snapshot(destination)
    assert [m.data for _, _, m in after] == [m.data for _, _, m in before]


@pytest.mark.parametrize('failure', ['invalid_json', 'truncated'])
def test_bad_source_is_not_replaced_and_final_destination_is_not_left(tmp_path, failure):
    source, destination = tmp_path/'bad.mcap', tmp_path/'converted.mcap'
    write_fixture(source, invalid_json=failure == 'invalid_json')
    if failure == 'truncated':
        source.write_bytes(source.read_bytes()[:-30])
    before = digest(source)
    with pytest.raises(Exception):
        convert(source, destination)
    assert digest(source) == before
    assert not destination.exists()


def test_existing_destination_is_never_overwritten(tmp_path):
    source, destination = tmp_path/'source.mcap', tmp_path/'existing.mcap'
    write_fixture(source)
    destination.write_bytes(b'pre-existing destination')
    before = digest(source)
    with pytest.raises(FileExistsError):
        convert(source, destination)
    assert destination.read_bytes() == b'pre-existing destination'
    assert digest(source) == before


def test_verify_rejects_changed_original_ros_payload_or_message_timestamp(tmp_path):
    source, wrong = tmp_path/'source.mcap', tmp_path/'wrong.mcap'
    write_fixture(source)
    # A second complete, checksum-valid MCAP cannot pass merely because both
    # have the same set of schema types or message count.
    with wrong.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        schema = writer.register_schema('std_msgs/msg/String', 'ros2msg', b'string data\n')
        channel = writer.register_channel('/action/joint_states', 'cdr', schema)
        writer.add_message(channel, 999, write_string('unrelated'), 998, sequence=1)
        writer.finish()
    with pytest.raises(Exception):
        verify(source, wrong)
