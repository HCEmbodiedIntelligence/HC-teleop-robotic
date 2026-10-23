"""Exercise metadata publication, retained calibration and expiry over real DDS."""
import json
import time
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String

from middleware.core.camera_metadata import CALIBRATION_TOPIC, EXTRINSICS_TOPIC, load_camera_metadata
from middleware.core.dataset_topics import DatasetTopics


def test_camera_metadata_dds_round_trip_and_stale_feedback(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('HC_CONTROL_BACKEND', 'motion')
    monkeypatch.delenv('HC_ROBOT_NAME', raising=False)
    monkeypatch.delenv('HC_ROBOT_CONFIG_ROOT', raising=False)
    root = Path(__file__).resolve().parents[1]
    config = load_camera_metadata({'robot_profiles': {
        'root': str(root / 'adapters/robots'), 'active': 'x1'}}, root / 'middleware')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=229)
    executor = SingleThreadedExecutor(context=context)
    bridge = rclpy.create_node('metadata_test_bridge', context=context)
    driver = rclpy.create_node('metadata_test_driver', context=context)
    observer = rclpy.create_node('metadata_test_observer', context=context)
    for node in (bridge, driver, observer):
        executor.add_node(node)
    projections = None
    try:
        projections = DatasetTopics(bridge, {'joint_groups': {}, 'camera_metadata': config})
        component = projections.camera_metadata
        sources = {
            stream: driver.create_publisher(CameraInfo, topics[0], qos_profile_sensor_data)
            for stream, topics in config['camera_info_topics'].items()
        }
        state = driver.create_publisher(JointState, '/observation/joint_states', qos_profile_sensor_data)
        received = {CALIBRATION_TOPIC: [], EXTRINSICS_TOPIC: []}
        subscribers = [observer.create_subscription(
            String, topic, lambda msg, t=topic: received[t].append(json.loads(msg.data)), 10)
            for topic in received]

        def send_sources():
            for stream, publisher in sources.items():
                info = CameraInfo(width=640, height=480, distortion_model='plumb_bob', d=[0.] * 5,
                                  k=[400., 0., 320., 0., 401., 240., 0., 0., 1.])
                info.header.stamp = driver.get_clock().now().to_msg()
                info.header.frame_id = stream + '_optical'
                publisher.publish(info)
            message = JointState(name=['head_joint_1', 'head_joint_2'], position=[0.2, -0.3])
            message.header.stamp = driver.get_clock().now().to_msg()
            state.publish(message)

        timer = driver.create_timer(.05, send_sources)

        def spin_until(predicate, timeout):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.03)
            assert predicate()

        spin_until(lambda: all(len(v) >= 2 for v in received.values()), 6.)
        calibration = received[CALIBRATION_TOPIC][-1]
        assert calibration['rgb']['fx'] == calibration['depth']['fx'] == 400.
        extrinsics = received[EXTRINSICS_TOPIC][-1]
        assert extrinsics['source'] == 'urdf_joint_states'
        assert extrinsics['transforms'][0]['child_frame'] == 'camera_head'
        assert extrinsics['stamp']['sec'] > 0

        # With the output timer stopped, a late reader still gets calibration.
        component.timer.cancel()
        late = []
        subscribers.append(observer.create_subscription(
            String, CALIBRATION_TOPIC, lambda msg: late.append(json.loads(msg.data)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        spin_until(lambda: bool(late), 3.)
        assert late[-1] == calibration

        timer.cancel()
        component.timer.reset()
        stop = time.monotonic() + 1.7
        spin_until(lambda: time.monotonic() >= stop, 2.)
        count = len(received[EXTRINSICS_TOPIC])
        stop = time.monotonic() + .8
        spin_until(lambda: time.monotonic() >= stop, 1.)
        assert len(received[EXTRINSICS_TOPIC]) == count
    finally:
        if projections is not None:
            projections.camera_metadata.tf_listener.unregister()
        executor.shutdown()
        for node in (observer, driver, bridge):
            node.destroy_node()
        context.shutdown()
