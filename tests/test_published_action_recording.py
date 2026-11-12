"""Record successful action publications exactly once, with unchanged CDR."""
from collections import Counter
from types import SimpleNamespace
from unittest.mock import Mock
import time

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

from middleware.core.app import MiddlewareRuntime
from middleware.core.ros_bridge import RosBridge


def make_bridge(sink):
    bridge = RosBridge({}, lambda *args: None)
    bridge._legacy_enabled = True
    bridge._command_source = 'vr'
    bridge._command_output_enabled = True
    bridge._ready_locked = lambda now: True
    bridge._ros_now = time.time
    bridge._published_action_sink = sink
    bridge._boundary = SimpleNamespace(arms=('Joint1_R',),
        prepare=lambda msg: (msg, None, None), commit=lambda *a, **k: True,
        diagnostics=Counter(), node=Mock())
    return bridge


def message():
    msg = JointState(name=['Joint1_R'], position=[0.12])
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(time.time_ns(), 10**9)
    msg.header.frame_id = 'retained-frame'
    return msg


def test_exact_successful_publication_is_queued_once():
    runtime = MiddlewareRuntime.__new__(MiddlewareRuntime)
    runtime.config = {'ros': {'record_action_at_publication': True}}
    queued = []
    runtime.recorder = SimpleNamespace(is_recording=lambda: True,
        record=lambda event: queued.append(event) or True)
    bridge = make_bridge(runtime._record_published_action)
    msg = message()
    publish = Mock()
    assert bridge.forward_live_command('vr', msg, publish)
    assert len(queued) == 1
    assert queued[0]['topic'] == '/action/joint_states'
    assert queued[0]['_raw'] == serialize_message(msg)
    assert deserialize_message(queued[0]['_raw'], JointState) == msg
    publish.assert_called_once_with(msg)
    assert bridge._boundary.diagnostics['action_recorded_at_publication'] == 1


def test_disabled_recording_does_not_enqueue():
    runtime = MiddlewareRuntime.__new__(MiddlewareRuntime)
    runtime.config = {'ros': {'record_action_at_publication': True}}
    runtime.recorder = SimpleNamespace(is_recording=lambda: False, record=Mock())
    assert not runtime._record_published_action(JointState())
    runtime.recorder.record.assert_not_called()


def test_configured_throttling_keeps_the_original_dds_capture_path():
    runtime = MiddlewareRuntime.__new__(MiddlewareRuntime)
    runtime.config = {'ros': {'record_action_at_publication': False}}
    runtime.recorder = SimpleNamespace(is_recording=lambda: True, record=Mock())
    assert not runtime._record_published_action(JointState())
    runtime.recorder.record.assert_not_called()


def test_rejected_target_cannot_create_action_record():
    sink = Mock()
    bridge = make_bridge(sink)
    bridge._boundary.prepare = lambda msg: None
    publish = Mock()
    assert not bridge.forward_live_command('vr', message(), publish)
    publish.assert_not_called()
    sink.assert_not_called()


def test_failed_publication_cannot_create_action_record():
    sink = Mock()
    bridge = make_bridge(sink)
    publish = Mock(side_effect=RuntimeError('publication failed'))
    with pytest.raises(RuntimeError):
        bridge.forward_live_command('vr', message(), publish)
    sink.assert_not_called()


def test_recording_exception_does_not_change_successful_output():
    bridge = make_bridge(Mock(side_effect=RuntimeError('writer failed')))
    msg = message()
    publish = Mock()
    assert bridge.forward_live_command('vr', msg, publish)
    publish.assert_called_once_with(msg)
    assert bridge._boundary.diagnostics['action_recording_errors'] == 1


def test_real_mcap_contains_every_accepted_publication(tmp_path):
    from mcap.reader import make_reader
    from middleware.core.topic_recorder import TopicRecorder
    topic = dict(topic='/action/joint_states', type='sensor_msgs/msg/JointState')
    recorder = TopicRecorder({'directory': str(tmp_path), 'format': 'mcap'},
                             tmp_path, topics=[topic])
    runtime = MiddlewareRuntime.__new__(MiddlewareRuntime)
    runtime.config = {'ros': {'record_action_at_publication': True}}
    runtime.recorder = recorder
    bridge = make_bridge(runtime._record_published_action)
    publish = Mock()
    expected = []
    path = recorder.start()
    try:
        for index in range(64):
            msg = message()
            msg.position = [index * .001]
            assert bridge.forward_live_command('vr', msg, publish)
            expected.append(serialize_message(msg))
    finally:
        recorder.stop()
    with open(path, 'rb') as stream:
        actual = [msg.data for _, channel, msg in make_reader(stream).iter_messages()
                  if channel.topic == topic['topic']]
    assert [deserialize_message(b, JointState) for b in actual] == [deserialize_message(b, JointState) for b in expected]
    assert publish.call_count == len(expected)
    assert bridge._boundary.diagnostics['action_recorded_at_publication'] == 64
