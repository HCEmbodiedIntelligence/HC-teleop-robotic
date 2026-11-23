"""Native ROS compatibility without importing ROS into the recording writer."""
import json
import random
import struct
import subprocess
import sys

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from middleware.core.recording.recording_cdr import read_joint_state, read_string, position_label, write_string, read_event


@pytest.mark.parametrize('count', [0, 1, 7, 14, 27])
@pytest.mark.parametrize('motion_fields', [False, True])
def test_native_joint_state_round_trip_all_layouts_and_alignment(count, motion_fields):
    rng = random.Random(17+count)
    for frame in ('', 'r', 'root_link', '原始坐标系'):
        message = JointState(name=['关节_'+str(i) for i in range(count)],
                             position=[rng.uniform(-3, 3) for _ in range(count)])
        message.header.stamp.sec = 1794883900
        message.header.stamp.nanosec = 123456789
        message.header.frame_id = frame
        if motion_fields:
            message.velocity = [rng.random() for _ in range(count)]
            message.effort = [rng.random() for _ in range(count)]
        raw = serialize_message(message)
        decoded = read_joint_state(raw)
        assert decoded.name == tuple(message.name)
        assert decoded.position == tuple(message.position)
        encoded = position_label(decoded.header_prefix, decoded.name, decoded.position)
        actual = deserialize_message(encoded, JointState)
        assert actual.header == message.header
        assert actual.name == message.name and actual.position == message.position
        assert not actual.velocity and not actual.effort


def test_big_endian_labels_are_understood_by_native_ros():
    frame = '右臂'.encode()+b'\0'
    prefix = b'\0\0\0\0'+struct.pack('>iII', 123, 456, len(frame))+frame
    raw = position_label(prefix, ['right', '另一个关节'], [.125, -.5])
    actual = deserialize_message(raw, JointState)
    assert actual.header.stamp.sec == 123 and actual.header.stamp.nanosec == 456
    assert actual.header.frame_id == '右臂'
    assert actual.name == ['right', '另一个关节'] and list(actual.position) == [.125, -.5]
    assert read_joint_state(raw).position == (.125, -.5)


def test_ros_string_and_malformed_payloads():
    for text in ('', '中文', json.dumps({'enabled': True})):
        assert read_string(serialize_message(String(data=text))) == text
    raw = serialize_message(JointState(name=['right'], position=[.1], velocity=[.2], effort=[.3]))
    for truncated in (raw[:3], raw[:11], raw[:-16], b'\0\7\0\0'+raw[4:]):
        with pytest.raises(ValueError):
            read_joint_state(truncated)
    with pytest.raises(ValueError):
        read_joint_state(raw+b'not padding')


def test_label_worker_never_initializes_ros_numpy_or_native_threadpools():
    message = JointState(name=['right'], position=[.1])
    message.header.stamp.sec = 123
    raw = serialize_message(message).hex()
    code = '''
import sys,json
from middleware.core.recording.recording_action_labels import RecordingActionLabels
labels=RecordingActionLabels(feedback_timeout_s=.25,status_timeout_s=.25)
labels.process(dict(topic='/action/joint_states',msg_type='sensor_msgs/msg/JointState',
    _raw=bytes.fromhex(sys.argv[1]),stamp_ns=123000000000))
assert labels.names==('right',)
assert 'numpy' not in sys.modules
assert 'rclpy' not in sys.modules
'''
    subprocess.run([sys.executable, '-c', code, raw], check=True, timeout=10)


@pytest.mark.parametrize('text', ['', 'ASCII', '中文与emoji🤖', 'x'*65537])
def test_written_string_is_standard_ros_cdr(text):
    raw = write_string(text)
    assert deserialize_message(raw, String).data == text
    assert read_string(raw) == text


@pytest.mark.parametrize('encoding', ['json', 'cdr'])
def test_event_reader_accepts_historical_json_and_ros_string(encoding):
    payload = {'source': 'feedback_hold', 'sent_to_robot': False, '说明': '恢复'}
    text = json.dumps(payload, ensure_ascii=False)
    raw = text.encode() if encoding == 'json' else write_string(text)
    assert read_event(raw, encoding, 'std_msgs/msg/String') == payload
    with pytest.raises(ValueError):
        read_event(raw, 'cdr', 'sensor_msgs/msg/JointState')


def test_event_codec_never_initializes_ros_or_numpy():
    code = """
import sys
from middleware.core.recording.recording_cdr import write_string, read_event
assert read_event(write_string('{"ok":true}'), 'cdr', 'std_msgs/msg/String') == {'ok': True}
assert 'numpy' not in sys.modules
assert 'rclpy' not in sys.modules
"""
    subprocess.run([sys.executable, '-c', code], check=True, timeout=10)
