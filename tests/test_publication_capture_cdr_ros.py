"""Isolated ROS domain: successful native bytes send equals recording evidence."""
import time
import math
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

from middleware.core.recording.publication_capture import PublicationCapture
from middleware.core.recording.topic_recorder import TopicRecorder


def test_successful_native_ros_cdr_publish_and_capture_have_identical_original_payload(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=231)
    node = rclpy.create_node('capture_immutable_cdr_evidence', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    actual, recorded, sent = [], [], []
    topic = '/test/recording/immutable_cdr_evidence'
    qos = QoSProfile(depth=16)
    publisher = node.create_publisher(JointState, topic, qos)
    subscription = node.create_subscription(JointState, topic, actual.append, qos, raw=True)
    capture = PublicationCapture(SimpleNamespace(record=lambda event: recorded.append(event) or True), pre_roll_s=0)
    originals = []
    try:
        deadline = time.monotonic()+4
        while publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert publisher.get_subscription_count() == 1
        for index in range(4):
            message = JointState(name=['Joint1_L', 'Joint1_R', 'Joint7_R'],
                                 position=[math.nan, index*.01, -index*.02],
                                 velocity=[.0, .1, -.2], effort=[.0, .3, -.4])
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
            message.header.frame_id = 'unchanged-authority-marker#reserved-permit'
            originals.append(message)
            sent.append(serialize_message(message))
        capture.begin_generation()
        # Both native Publisher.publish(bytes) and raw capture must bypass a
        # second Python serialize_message, preserving precisely the sent CDR.
        with patch('rclpy.serialization.serialize_message', side_effect=AssertionError('second serialization')):
            for message, payload in zip(originals, sent):
                # Same native publisher and QoS: compare the established ROS
                # message API with its serialized-bytes API on the wire.
                publisher.publish(message)
                ticket = capture.reserve()
                publisher.publish(payload)
                capture.complete(ticket, serialized_cdr=payload,
                                 publication_ns=time.time_ns(), publication_monotonic=time.monotonic())
            capture.end_generation()
            deadline = time.monotonic()+3
            while len(actual) < len(sent)*2 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
        assert len(actual) == len(sent)*2
        # The established message API may emit different insignificant CDR
        # alignment padding. The serialized API must preserve every byte of
        # its supplied CDR; both paths must decode to the same original fields.
        assert actual[1::2] == sent
        assert [event['_raw'] for event in recorded] == sent
        assert all(event['_raw'] is payload for event, payload in zip(recorded, sent))
        for index, payload in enumerate(actual):
            decoded, original = deserialize_message(payload, JointState), originals[index//2]
            assert decoded.header == original.header
            assert decoded.name == original.name
            assert decoded.velocity == original.velocity and decoded.effort == original.effort
            assert all(math.isnan(left) and math.isnan(right) or left == right
                       for left, right in zip(decoded.position, original.position))
        assert capture.status()['enqueued'] == len(sent)
        assert capture.status()['failed'] == capture.status()['dropped'] == 0
    finally:
        capture.close()
        node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


def test_successful_raw_cdr_is_the_exact_mcap_payload_without_restamping(tmp_path):
    from mcap.reader import make_reader
    recorder = TopicRecorder({'directory':str(tmp_path)}, tmp_path,
        topics=[{'topic':'/action/joint_states','type':'sensor_msgs/msg/JointState'}])
    capture = PublicationCapture(recorder, pre_roll_s=0)
    recorder.publication_capture = capture
    try:
        path = recorder.start('immutable-cdr', {'profile_id':'x1'})
        message = JointState(name=['Joint1_R'], position=[.25], velocity=[.5], effort=[.75])
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns()-15_000_000, 1_000_000_000)
        message.header.frame_id = 'original-marker#permit'
        payload = serialize_message(message)
        publication_ns = time.time_ns()
        # Mutating the old message object after serialization cannot alter the
        # sent immutable CDR or the corresponding recording evidence.
        message.header.frame_id = 'later-mutated-message'
        message.position[0] = 99.
        with patch('rclpy.serialization.serialize_message', side_effect=AssertionError('second serialization')):
            capture.complete(capture.reserve(), serialized_cdr=payload, publication_ns=publication_ns)
            result = recorder.stop()
        with open(path, 'rb') as stream:
            samples = list(make_reader(stream).iter_messages())
        assert len(samples) == 1
        stored = samples[0][2]
        assert stored.data == payload
        assert stored.log_time == stored.publish_time == publication_ns
        restored = deserialize_message(stored.data, JointState)
        assert restored.header.frame_id == 'original-marker#permit'
        assert restored.position[0] == .25
        assert result['publication_capture']['captured'] == result['publication_capture']['enqueued'] == 1
        assert result['publication_capture']['failed'] == result['publication_capture']['dropped'] == 0
    finally:
        recorder.shutdown()
