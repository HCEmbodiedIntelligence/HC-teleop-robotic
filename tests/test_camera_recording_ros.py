"""Isolated DDS test: no hardware nodes or motion commands."""
import json
import time

import pytest


def test_each_segment_records_prior_static_tf_and_camera_calibration(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from mcap.reader import make_reader
    from middleware.core.ros_qos import subscription_qos
    from middleware.core.ros_recording_executor import RosRecordingExecutor
    from middleware.core.topic_recorder import TopicRecorder

    context = Context()
    rclpy.init(context=context, domain_id=218)
    node = rclpy.create_node('camera_recording_test', context=context)
    static = node.create_publisher(TFMessage, '/tf_static', subscription_qos({'topic': '/tf_static'}))
    camera = node.create_publisher(CameraInfo, '/test_camera/color/camera_info', 10)
    transform = TransformStamped()
    transform.header.frame_id = 'test_mount'
    transform.child_frame_id = 'test_color_optical'
    transform.transform.rotation.w = 1.
    transform.transform.translation.x = .12
    static.publish(TFMessage(transforms=[transform]))  # Published ONCE, before either recording.
    info = CameraInfo(width=640, height=480)
    info.header.frame_id = transform.child_frame_id
    info.k = [400., 0., 320., 0., 400., 240., 0., 0., 1.]
    subscriptions = [dict(topic='/tf_static', type='tf2_msgs/msg/TFMessage', outputs=['record']),
                     dict(topic='/test_camera/color/camera_info', type='sensor_msgs/msg/CameraInfo', outputs=['record'])]
    try:
        for index in range(2):
            recorder = TopicRecorder({'directory': str(tmp_path), 'format': 'mcap'}, tmp_path)
            path = recorder.start('segment_' + str(index))
            executor = RosRecordingExecutor({'domain_id': 218, 'subscriptions': subscriptions}, recorder)
            try:
                executor.start()
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    info.header.stamp = node.get_clock().now().to_msg()
                    camera.publish(info)
                    time.sleep(.05)
                    status = executor.status()
                    assert not status.get('error'), status
                    if status.get('recorded', 0) >= 3:
                        break
                else:
                    pytest.fail('Recorder did not receive static TF and CameraInfo')
            finally:
                executor.stop()
                recorder.stop()
            with open(path, 'rb') as stream:
                messages = list(make_reader(stream).iter_messages())
            by_topic = {}
            for _, channel, message in messages:
                by_topic.setdefault(channel.topic, []).append(message)
            tf = deserialize_message(by_topic['/tf_static'][0].data, TFMessage)
            assert tf.transforms[0].transform.translation.x == .12
            assert '/test_camera/color/camera_info' in by_topic
            documents = by_topic['/test_camera/calibration']
            assert len(documents) == 1
            value = json.loads(deserialize_message(documents[0].data, String).data)
            assert value['rgb']['fx'] == 400
            assert value['rgb']['frame_id'] == 'test_color_optical'
    finally:
        node.destroy_node()
        context.shutdown()
