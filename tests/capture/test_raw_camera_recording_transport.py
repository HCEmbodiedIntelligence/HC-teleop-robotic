"""Exercise the actual raw recorder with serialized full-size ROS images."""
import queue
import threading
import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Image
from humanoid_camera.transport import capture_qos
from middleware.core import ros_recording_executor as recorder


@pytest.mark.parametrize('offered', [ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT])
def test_raw_recorder_preserves_serialized_camera_image_and_reports_qos(monkeypatch, offered):
    # The worker runs in a thread here; do not lower the pytest process priority.
    monkeypatch.setattr(recorder, '_lower_recording_priority', lambda: None)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=226)
    source = rclpy.create_node('raw_camera_test_source', context=context)
    topic = '/raw_camera_test_' + uuid.uuid4().hex
    publisher = source.create_publisher(Image, topic, capture_qos(10, offered))
    events, statuses = queue.Queue(maxsize=100), queue.Queue(maxsize=1)
    accepting, stop, reset = threading.Event(), threading.Event(), threading.Event()
    accepting.set()
    worker = threading.Thread(target=recorder._raw_recording_process, args=(
        {'domain_id': 226, 'node_name': 'raw_camera_test'},
        [{'topic': topic, 'type': 'sensor_msgs/msg/Image'}],
        events, accepting, None, stop, reset, statuses,
    ))
    worker.start()
    try:
        status = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                status = statuses.get(timeout=.05)
            except queue.Empty:
                continue
            assert status['state'] != 'error', status
            qos = status.get('receiver_qos', {}).get(topic, {})
            if (qos.get('requested_reliability') == offered.name
                    and qos.get('offered_reliability') == [offered.name]
                    and publisher.get_subscription_count() == 1):
                break
        else:
            pytest.fail(f'raw recorder did not discover publisher: {status}')
        message = Image(width=640, height=480, encoding='rgb8', step=1920,
                        data=bytes([20, 40, 80]) * (640 * 480))
        message.header.stamp.sec = 1700000000
        message.header.stamp.nanosec = 123456789
        message.header.frame_id = 'unchanged_camera_optical_frame'
        # Compare the exact CDR buffer sent. A second serialization may contain
        # different alignment padding despite representing identical fields.
        serialized = serialize_message(message)
        publisher.publish(serialized)
        event = events.get(timeout=3)
        assert event['_raw'] == serialized
        assert deserialize_message(event['_raw'], Image) == message
        assert event['topic'] == topic and event['msg_type'] == 'sensor_msgs/msg/Image'
    finally:
        stop.set()
        worker.join(timeout=5)
        source.destroy_node()
        context.try_shutdown()
    assert not worker.is_alive()
