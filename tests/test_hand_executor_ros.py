"""Isolated localhost ROS test: slow hand work must not overwrite arm input."""
import os
from pathlib import Path
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('HC_RUN_LEGACY_ROS_TESTS') != '1',
                                reason='explicit isolated ROS test only')


def test_slow_hands_do_not_starve_arm_reader(monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert not os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    import yaml
    from middleware.core.ros_bridge import RosBridge
    from middleware.core.legacy_control import LegacyTopicControl

    shared_hands = os.environ.get('HC_TEST_SHARED_HANDS') == '1'
    if shared_hands:
        original_init = LegacyTopicControl.__init__
        def init_on_shared_executor(self, *args, **kwargs):
            kwargs['hand_node'] = None
            original_init(self, *args, **kwargs)
        monkeypatch.setattr(LegacyTopicControl, '__init__', init_on_shared_executor)

    root = Path(__file__).resolve().parents[1]
    profile = root/'adapters/robots/x1'
    cfg = yaml.safe_load((profile/'arm_teleop.yaml').read_text())
    arms = [n for side in ('right', 'left') for n in cfg['arms'][side]['joint_names']]
    q = [float(cfg['robot']['initial_joints'][n]) for n in arms]
    rclpy.init()
    node = rclpy.create_node('isolated_hand_test_peers')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
    arm = node.create_publisher(JointState, '/control/vr/joint_states', qos)
    state = node.create_publisher(JointState, '/observation/joint_states', qos)
    ready = node.create_publisher(Bool, '/status/hardware/ready', qos)
    fingers = {side: node.create_publisher(JointState, f'/control/vr/{side}_hand/joint_states', qos)
               for side in ('right', 'left')}
    received, sent = set(), set()
    node.create_subscription(JointState, '/action/joint_states',
        lambda m: received.add(m.header.stamp.sec*10**9+m.header.stamp.nanosec), qos)
    bridge = RosBridge(dict(domain_id=231, node_name='isolated_hand_test',
        enabled=True, legacy_control_profile=str(profile),
        command_mux=dict(enabled=True, default_source='vr'),
        intervention=dict(enabled=False, automatic_recording=False),
        subscriptions=[]), lambda *a: None)
    timer_active = [False]
    def message(names, values, stamp):
        m = JointState(name=names, position=values)
        m.header.stamp = stamp
        return m
    def tick():
        stamp = node.get_clock().now().to_msg()
        state.publish(message(arms, q, stamp))
        ready.publish(Bool(data=True))
        if not timer_active[0]:
            return
        sent.add(stamp.sec*10**9+stamp.nanosec)
        arm.publish(message(arms, q, stamp))
        for side in fingers:
            c = cfg['grippers'][side]
            fingers[side].publish(message(c['finger_joint_names'], c['finger_open'], stamp))
    node.create_timer(.01, tick)
    def spin_for(seconds):
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.002)
    try:
        bridge.start()
        spin_for(2)
        assert bridge._boundary is not None, bridge.status()
        original = bridge._boundary.prepare
        def slow_hand(m):
            if not set(m.name).intersection(arms):
                time.sleep(.008)
            return original(m)
        bridge._boundary.prepare = slow_hand
        bridge.set_control_source('vr')
        assert bridge.set_command_output_enabled(True)
        timer_active[0] = True
        spin_for(3)
        timer_active[0] = False
        spin_for(.15)
        ratio = len(sent & received)/len(sent)
        print('HAND_EXECUTOR_RESULT', dict(shared_hands=shared_hands, sent=len(sent), received=len(sent & received), ratio=ratio), flush=True)
        if not shared_hands:
            assert ratio >= .95
        subscriptions = {topic: [p.node_name for p in node.get_subscriptions_info_by_topic(topic)
                                  if p.node_name.startswith('isolated_hand_test_')]
                         for topic in ['/control/vr/joint_states', '/control/vr/right_hand/joint_states']}
        assert subscriptions['/control/vr/joint_states'] == ['isolated_hand_test_commands']
        assert subscriptions['/control/vr/right_hand/joint_states'] == [
            'isolated_hand_test_commands' if shared_hands else 'isolated_hand_test_hands']
    finally:
        bridge.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
