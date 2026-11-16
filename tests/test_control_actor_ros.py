"""Only localhost domain 231: real DDS is driven by one actor thread."""
import threading
import time
from types import SimpleNamespace

import pytest
pytest.importorskip('rclpy')
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from middleware.core.ros_bridge import RosBridge


def test_real_ros_actor_single_sender_latest_targets_and_stop(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '231')
    final_topic = '/test/control_actor/final'
    target_topic = '/test/control_actor/targets'
    ready_topic = '/test/control_actor/ready'
    enable_topic = '/test/control_actor/enabled'
    bridge = RosBridge({'domain_id':231, '_control_actor_mode':True,
        'node_name':'test_control_actor_core',
        'subscriptions':[{'topic':'/test/ignored_camera','type':'sensor_msgs/msg/CompressedImage'}],
        'dataset_topics':{'enabled':True},
        'command_mux':{'vr_topic':target_topic, 'output_topic':final_topic,
                       'hardware_ready_topic':ready_topic, 'hardware_enable_topic':enable_topic}},
        lambda *_: None, camera_topics=['/test/ignored_camera'])
    sent_threads = []
    original = bridge._publisher
    def tracked_publisher(node, publishers, topic, msg_type, kind):
        pub = original(node, publishers, topic, msg_type, kind)
        if topic == final_topic:
            return SimpleNamespace(publish=lambda target: (sent_threads.append(threading.get_ident()), pub.publish(target)))
        return pub
    bridge._publisher = tracked_publisher
    context = Context()
    rclpy.init(args=[], context=context, domain_id=231)
    node = rclpy.create_node('test_control_actor_fixture', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    ready = node.create_publisher(Bool, ready_topic, 10)
    source = node.create_publisher(JointState, target_topic, qos_profile_sensor_data)
    actual = []
    subscription = node.create_subscription(JointState, final_topic, actual.append, 10)
    core_nodes, destroyed = [], []
    original_create = rclpy.create_node
    def create_node(name, **kwargs):
        created = original_create(name, **kwargs)
        if name.startswith('test_control_actor_core'):
            core_nodes.append((created, kwargs))
            original_destroy = created.destroy_node
            def destroy():
                destroyed.append(id(created))
                return original_destroy()
            created.destroy_node = destroy
        return created
    monkeypatch.setattr(rclpy, 'create_node', create_node)
    try:
        bridge.start()
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            ready.publish(Bool(data=True))
            executor.spin_once(timeout_sec=.005)
            if bridge.status()['state'] == 'error':
                pytest.fail(str(bridge.status()))
            if source.get_subscription_count() and bridge.status()['command_mux']['hardware_ready']:
                break
        assert source.get_subscription_count() == 1
        assert bridge._actor_thread_ident is not None
        assert len(core_nodes) == 1
        assert core_nodes[0][1]['enable_rosout'] is False
        assert core_nodes[0][1]['start_parameter_services'] is False
        assert not tuple(core_nodes[0][0].services)
        assert bridge._contention_thread is None
        assert bridge.status()['topic_health'] == {}
        assert not bridge.status().get('dataset_process')
        for index in range(20):
            ready.publish(Bool(data=True))
            target = JointState(name=['test_joint'], position=[index*.001])
            target.header.stamp.sec,target.header.stamp.nanosec = divmod(time.time_ns(),1_000_000_000)
            source.publish(target)
            executor.spin_once(timeout_sec=.005)
        deadline = time.monotonic()+1.
        while not actual and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert actual
        assert set(sent_threads) == {bridge._actor_thread_ident}
        # All targets retained their source stamps, rather than restamping an
        # accumulated queue during a drain/catch-up period.
        assert all(target.header.stamp.sec > 0 for target in actual)
        bridge.emergency_stop('/test/control_actor/estop', 'loopback test')
        count = len(sent_threads)
        for _ in range(10):
            ready.publish(Bool(data=True))
            target.header.stamp.sec,target.header.stamp.nanosec = divmod(time.time_ns(),1_000_000_000)
            source.publish(target)
            executor.spin_once(timeout_sec=.005)
        assert len(sent_threads) == count
        assert not bridge.status()['command_mux']['output_enabled']
        assert not bridge._actor_targets
    finally:
        bridge.stop()
        assert bridge.status()['state'] != 'error', bridge.status()
        assert bridge._thread is None or not bridge._thread.is_alive()
        assert destroyed == [id(core_nodes[0][0])]
        node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
