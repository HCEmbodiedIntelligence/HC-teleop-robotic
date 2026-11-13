"""Publication capture integration without ROS network or physical actuators."""
from collections import Counter
from types import SimpleNamespace
from unittest.mock import Mock
import threading
import time

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

from middleware.core.publication_capture import PublicationCapture
from middleware.core.ros_bridge import RosBridge
from middleware.core.topic_recorder import TopicRecorder


def message(value):
    msg = JointState(name=['Joint1_R'], position=[float(value)])
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(time.time_ns(), 10**9)
    msg.header.frame_id = 'actual-publication'
    return msg


def bridge_with(capture):
    bridge = RosBridge({}, lambda *args: None)
    bridge._legacy_enabled = True
    bridge._command_source = 'vr'
    bridge._command_output_enabled = True
    bridge._ready_locked = lambda now: True
    bridge._ros_now = time.time
    bridge._publication_capture = capture
    bridge._boundary = SimpleNamespace(arms=('Joint1_R',),
        prepare=lambda msg: (msg, None, None), commit=lambda *args, **kwargs: True,
        diagnostics=Counter(), node=Mock())
    return bridge


def test_actual_mcap_retains_all_64_successful_publications(tmp_path):
    from mcap.reader import make_reader
    recorder = TopicRecorder({'directory': str(tmp_path), 'format': 'mcap'}, tmp_path,
        topics=[{'topic': '/action/joint_states', 'type': 'sensor_msgs/msg/JointState'}])
    capture = PublicationCapture(recorder, pre_roll_s=0)
    recorder.publication_capture = capture
    bridge = bridge_with(capture)
    publish = Mock()
    path = recorder.start()
    expected = []
    try:
        for index in range(64):
            msg = message(index*.001)
            assert bridge.forward_live_command('vr', msg, publish)
            expected.append(msg)
        recorder.stop()
        with open(path, 'rb') as stream:
            actual = [deserialize_message(m.data, JointState)
                      for _, c, m in make_reader(stream).iter_messages()
                      if c.topic == '/action/joint_states']
        assert actual == expected
        assert publish.call_count == 64
        assert recorder.status()['publication_capture']['enqueued'] == 64
    finally:
        recorder.shutdown()


def test_blocked_capture_ipc_never_holds_send_or_revocation_lock():
    entered, release = threading.Event(), threading.Event()
    def record(event):
        entered.set()
        assert release.wait(3)
        return True
    capture = PublicationCapture(SimpleNamespace(record=record), pre_roll_s=0)
    bridge = bridge_with(capture)
    original_reserve, original_complete = capture.reserve, capture.complete
    def reserve():
        assert not bridge._lock._is_owned()
        return original_reserve()
    def complete(*args, **kwargs):
        assert not bridge._lock._is_owned()
        return original_complete(*args, **kwargs)
    capture.reserve, capture.complete = reserve, complete
    capture.begin_generation()
    try:
        assert bridge.forward_live_command('vr', message(1), Mock())
        assert entered.wait(1)
        assert bridge.forward_live_command('vr', message(2), Mock())
        with bridge._lock:
            bridge._command_output_enabled = False
        assert not bridge.forward_live_command('vr', message(3), Mock())
    finally:
        release.set()
        capture.end_generation()
        capture.close()
    assert capture.status()['enqueued'] == 2


def test_final_gate_rejection_releases_capture_reservation_without_record():
    recorder = SimpleNamespace(record=Mock(return_value=True))
    capture = PublicationCapture(recorder, pre_roll_s=0)
    bridge = bridge_with(capture)
    bridge._boundary.commit = lambda *args, **kwargs: False
    publish = Mock()
    capture.begin_generation()
    try:
        assert not bridge.forward_live_command('vr', message(1), publish)
        capture.end_generation()
        publish.assert_not_called()
        recorder.record.assert_not_called()
        assert capture.status()['inflight'] == 0
    finally:
        capture.close()


def test_public_profile_validation_does_not_clone_ros_messages():
    bridge = bridge_with(None)
    bridge._public_api = Mock()
    msg = message(1)
    publish = Mock()
    assert bridge.forward_live_command('vr', msg, publish)
    bridge._public_api.convert.assert_called_once_with(
        msg.name, msg.position, msg.velocity, msg.effort, to_model=True)
    bridge._public_api.convert_message.assert_not_called()
    bridge._public_api.convert.side_effect = ValueError('unknown joint')
    assert not bridge.forward_live_command('vr', message(2), publish)
    assert publish.call_count == 1


def test_stop_closes_normal_ingress_before_draining_reserved_actions(tmp_path):
    from mcap.reader import make_reader
    topics = [{'topic': topic, 'type': 'sensor_msgs/msg/JointState'}
              for topic in ['/action/joint_states', '/observation/joint_states']]
    recorder = TopicRecorder({'directory': str(tmp_path), 'format': 'mcap'}, tmp_path,
                             topics=topics)
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    original = recorder.record_publication
    def blocked(event):
        entered.set()
        assert release.wait(3)
        return original(event)
    recorder.record_publication = blocked
    capture = PublicationCapture(recorder, pre_roll_s=0)
    recorder.publication_capture = capture
    bridge = bridge_with(capture)
    path = recorder.start()
    event = dict(kind='ros_message', topic='/observation/joint_states',
                 msg_type='sensor_msgs/msg/JointState', _raw=serialize_message(message(0)),
                 stamp_ns=time.time_ns())
    errors = []
    try:
        assert bridge.forward_live_command('vr', message(1), Mock())
        assert entered.wait(1)
        assert recorder.record(event)
        def stop():
            try:
                recorder.stop()
            except Exception as exc:
                errors.append(exc)
            stopped.set()
        thread = threading.Thread(target=stop)
        thread.start()
        deadline = time.monotonic()+1
        while recorder.is_recording() and time.monotonic() < deadline:
            time.sleep(.001)
        assert not recorder.is_recording()
        assert not stopped.is_set()  # The reserved action is still draining.
        assert not recorder.record(event)  # No artificial video/state tail.
        release.set()
        assert stopped.wait(2)
        thread.join(1)
        assert errors == []
        with open(path, 'rb') as stream:
            actual = [channel.topic for _, channel, _ in make_reader(stream).iter_messages()]
        assert actual.count('/action/joint_states') == 1
        assert actual.count('/observation/joint_states') == 1
    finally:
        release.set()
        recorder.shutdown()
