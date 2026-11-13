"""Record successful action publications exactly once, preserving message fields."""
from collections import Counter
from types import SimpleNamespace
from unittest.mock import Mock
import time

import pytest
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

from middleware.core.app import MiddlewareRuntime
from middleware.core.publication_capture import PublicationCapture
from middleware.core.ros_bridge import RosBridge


def make_bridge(capture):
    bridge = RosBridge({}, lambda *args: None)
    bridge._legacy_enabled = True
    bridge._command_source = 'vr'
    bridge._command_output_enabled = True
    bridge._ready_locked = lambda now: True
    bridge._ros_now = time.time
    bridge._publication_capture = capture
    bridge._boundary = SimpleNamespace(arms=('Joint1_R',),
        prepare=lambda msg: (msg, None, None), commit=lambda *a, **k: True,
        diagnostics=Counter(), node=Mock())
    return bridge


def message():
    msg = JointState(name=['Joint1_R'], position=[0.12], velocity=[0.03], effort=[0.04])
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(time.time_ns(), 10**9)
    msg.header.frame_id = 'retained-frame'
    return msg


def test_exact_successful_publication_is_queued_once():
    queued = []
    capture = PublicationCapture(SimpleNamespace(
        record=lambda event: queued.append(event) or True), pre_roll_s=0)
    bridge = make_bridge(capture)
    msg = message()
    publish = Mock()
    capture.begin_generation()
    try:
        assert bridge.forward_live_command('vr', msg, publish)
        capture.end_generation()
    finally:
        capture.close()
    assert len(queued) == 1
    assert queued[0]['topic'] == '/action/joint_states'
    assert deserialize_message(queued[0]['_raw'], JointState) == msg
    publish.assert_called_once_with(msg)
    assert capture.status()['enqueued'] == 1
    assert capture.status()['failed'] == 0


def test_disabled_recording_does_not_enqueue():
    recorder = SimpleNamespace(record=Mock(return_value=True))
    capture = PublicationCapture(recorder, pre_roll_s=0)
    bridge = make_bridge(capture)
    try:
        # Capture is installed, but no recording generation is open.
        assert bridge.forward_live_command('vr', message(), Mock())
        capture.end_generation()
        recorder.record.assert_not_called()
        assert capture.status()['captured'] == 0
    finally:
        capture.close()


def test_configured_throttling_keeps_the_original_dds_capture_path():
    runtime = MiddlewareRuntime.__new__(MiddlewareRuntime)
    runtime.config = {'ros': {'legacy_control_profile': 'profile', 'subscriptions': [
        {'topic': '/action/joint_states', 'max_hz': 50}]}}
    runtime.recorder = SimpleNamespace(record=Mock())
    runtime.ros = make_bridge(None)
    runtime._configure_publication_capture(runtime.recorder)
    assert runtime.config['ros']['record_action_at_publication'] is False
    assert runtime.ros._publication_capture is None
    assert not hasattr(runtime.recorder, 'publication_capture')
    runtime.recorder.record.assert_not_called()


@pytest.mark.parametrize('rejection', ['prepare', 'commit'])
def test_rejected_target_cannot_create_action_record(rejection):
    recorder = SimpleNamespace(record=Mock(return_value=True))
    capture = PublicationCapture(recorder, pre_roll_s=0)
    bridge = make_bridge(capture)
    if rejection == 'prepare':
        bridge._boundary.prepare = lambda msg: None
    else:
        bridge._boundary.commit = lambda *args, **kwargs: False
    publish = Mock()
    capture.begin_generation()
    try:
        assert not bridge.forward_live_command('vr', message(), publish)
        capture.end_generation()
    finally:
        capture.close()
    publish.assert_not_called()
    recorder.record.assert_not_called()
    assert capture.status()['inflight'] == 0
    assert capture.status()['captured'] == 0


def test_failed_publication_cannot_create_action_record():
    recorder = SimpleNamespace(record=Mock(return_value=True))
    capture = PublicationCapture(recorder, pre_roll_s=0)
    bridge = make_bridge(capture)
    publish = Mock(side_effect=RuntimeError('publication failed'))
    capture.begin_generation()
    try:
        with pytest.raises(RuntimeError, match='publication failed'):
            bridge.forward_live_command('vr', message(), publish)
        capture.end_generation()
    finally:
        capture.close()
    recorder.record.assert_not_called()
    assert capture.status()['inflight'] == 0
    assert capture.status()['captured'] == 0


def test_recording_exception_does_not_change_successful_output():
    recorder = SimpleNamespace(record=Mock(side_effect=RuntimeError('writer failed')))
    capture = PublicationCapture(recorder, pre_roll_s=0)
    bridge = make_bridge(capture)
    msg = message()
    publish = Mock()
    capture.begin_generation()
    try:
        assert bridge.forward_live_command('vr', msg, publish)
        capture.end_generation()
    finally:
        capture.close()
    publish.assert_called_once_with(msg)
    assert capture.status()['failed'] == 1
    assert 'writer failed' in capture.status()['error']


def test_real_mcap_contains_every_accepted_publication(tmp_path):
    from mcap.reader import make_reader
    from middleware.core.topic_recorder import TopicRecorder
    topic = dict(topic='/action/joint_states', type='sensor_msgs/msg/JointState')
    recorder = TopicRecorder({'directory': str(tmp_path), 'format': 'mcap'},
                             tmp_path, topics=[topic])
    capture = PublicationCapture(recorder, pre_roll_s=0)
    recorder.publication_capture = capture
    bridge = make_bridge(capture)
    publish = Mock()
    expected = []
    path = recorder.start()
    try:
        for index in range(64):
            msg = message()
            msg.position = [index * .001]
            assert bridge.forward_live_command('vr', msg, publish)
            expected.append(msg)
    finally:
        recorder.stop()
        recorder.shutdown()
    with open(path, 'rb') as stream:
        actual = [msg.data for _, channel, msg in make_reader(stream).iter_messages()
                  if channel.topic == topic['topic']]
    assert [deserialize_message(b, JointState) for b in actual] == expected
    assert publish.call_count == len(expected)
    assert capture.status()['enqueued'] == 64
    assert capture.status()['dropped'] == 0
    assert capture.status()['failed'] == 0
