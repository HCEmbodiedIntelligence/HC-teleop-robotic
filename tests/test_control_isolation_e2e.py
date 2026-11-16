"""Spawned control owner against localhost DDS, with real publication recording.

These tests never discover the robot domain. They exercise the process boundary
and immutable CDR capture while the parent observer and writer ingress stall.
Collision, freshness and motion limits use the shipped X1 implementation.
"""
from pathlib import Path
import queue
import threading
import time

import pytest

pytest.importorskip('rclpy')
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import yaml

from middleware.core.control_process import CoreProcessClient, RemotePublicationCapture
from middleware.core.topic_recorder import TopicRecorder


ROOT = Path(__file__).resolve().parents[1]


def test_shipped_x1_uses_control_process_without_a_temporary_environment_flag(monkeypatch):
    from middleware.core.config import ConfigStore, isolated_control_process_enabled
    monkeypatch.delenv('HC_ISOLATED_CONTROL_PROCESS', raising=False)
    settings = ConfigStore(ROOT/'middleware/config.yaml').load()['ros']
    assert isolated_control_process_enabled(settings) is True


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    assert predicate(), 'condition did not become true'


def joint(values, stamp):
    message = JointState(name=list(values), position=list(values.values()),
                         velocity=[0.]*len(values))
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    return message


class LoopbackRobot:
    def __init__(self):
        profile = ROOT/'adapters/robots/x1'
        teleop = yaml.safe_load((profile/'arm_teleop.yaml').read_text())
        self.values = {n:float(v) for n,v in teleop['robot']['initial_joints'].items()}
        self.arms = {n:self.values[n] for side in ('right', 'left')
                     for n in teleop['arms'][side]['joint_names']}
        self.hands = {side:dict(zip(teleop['grippers'][side]['finger_joint_names'],
                                   teleop['grippers'][side]['finger_open']))
                      for side in ('right', 'left')}
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=231)
        self.node = rclpy.create_node('test_isolated_robot_fixture', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        def publisher(kind, topic):
            return self.node.create_publisher(kind, topic, qos_profile_sensor_data)
        self.ready = self.node.create_publisher(Bool, '/status/hardware/ready', 10)
        self.feedback = publisher(JointState, '/observation/joint_states')
        self.hand_feedback = publisher(JointState, '/observation/hands/joint_states')
        self.source = publisher(JointState, '/control/vr/joint_states')
        self.hand_sources = {side:publisher(JointState, '/control/vr/'+side+'_hand/joint_states')
                             for side in self.hands}
        self.actual = []
        self.subscription = self.node.create_subscription(
            JointState, '/action/joint_states',
            lambda raw:self.actual.append((time.monotonic_ns(), bytes(raw))), 1000, raw=True)
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, name='loopback-robot', daemon=True)
        self.thread.start()

    def run(self):
        next_publish = 0.
        while not self.stopped.is_set():
            now = time.monotonic()
            if now >= next_publish:
                next_publish = now+.01
                stamp = time.time_ns()
                self.ready.publish(Bool(data=True))
                self.feedback.publish(joint(self.values, stamp))
                hand_values = {n:v for values in self.hands.values() for n,v in values.items()}
                self.hand_feedback.publish(joint(hand_values, stamp))
                self.source.publish(joint(self.arms, stamp))
                for side, values in self.hands.items():
                    self.hand_sources[side].publish(joint(values, stamp))
            self.executor.spin_once(timeout_sec=.001)

    def close(self):
        self.stopped.set()
        self.thread.join(2.)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()


@pytest.fixture
def loopback(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '231')
    robot = LoopbackRobot()
    try:
        yield robot
    finally:
        robot.close()


def config():
    return {'domain_id':231, 'node_name':'test_isolated_control_owner',
            'legacy_control_profile':str(ROOT/'adapters/robots/x1'),
            'intervention':{'automatic_recording':False},
            'dataset_topics':{'enabled':False}, 'subscriptions':[]}


def test_observer_and_recording_stall_do_not_block_actual_control(loopback, tmp_path):
    from mcap.reader import make_reader
    blocked, release = threading.Event(), threading.Event()
    def observer(*_):
        blocked.set()
        release.wait(5.)
    client = CoreProcessClient(config(), on_event=observer)
    recorder = TopicRecorder({'directory':'recordings'}, tmp_path,
                            topics=[{'topic':'/action/joint_states', 'type':'sensor_msgs/msg/JointState'}])
    client.capture_spec = {'resources':recorder.ipc_resources(), 'finish':recorder._finish_event}
    capture = RemotePublicationCapture(client, lambda:True)
    recorder.publication_capture = capture
    try:
        client.start()
        assert client.process.pid != __import__('os').getpid()
        wait_until(lambda:client.snapshot().get('control_ready'))
        client.call('set_command_output_enabled', True)
        wait_until(lambda:len(loopback.actual) >= 5)
        recorder.start('isolated.mcap', {'profile_id':'x1'})
        # Block the real parent observer thread, then fill its bounded mailbox.
        # This is independent of the IPC reply/cache thread and control actor.
        client.channels['observers'].put_nowait(({'kind':'test'}, []))
        assert blocked.wait(1.)
        for _ in range(512):
            try:
                client.channels['observers'].put_nowait(({'kind':'test'}, []))
            except queue.Full:
                break
        # Hold the actual recorder ingress lock: its child capture worker waits
        # here, while the actuator sender must continue with unchanged targets.
        with recorder._ingress_lock:
            before = len(loopback.actual)
            started = time.monotonic()
            time.sleep(.8)
            assert len(loopback.actual)-before >= 40
            assert time.monotonic()-started < 1.5
            client.call('control_ready', timeout=.5)
        wait_until(lambda:capture.status()['enqueued'] >= 40)
        result = recorder.stop()
        assert not result['error'], result
        assert result['accepted'] == result['messages']
        assert result['dropped'] == 0
        assert capture.status()['dropped'] == capture.status()['failed'] == 0
        with recorder.path.open('rb') as stream:
            recorded = list(make_reader(stream).iter_messages())
        assert len(recorded) >= 40
        actual = {raw for _,raw in loopback.actual}
        assert all(record.data in actual for _,_,record in recorded)
        assert len({record.data for _,_,record in recorded}) == len(recorded)
        # This regression detects a parent stall coupled into the sender. The
        # strict 40 ms acceptance bound is measured separately on real hardware.
        stamps = [stamp for stamp,_ in loopback.actual[before:]]
        assert max(b-a for a,b in zip(stamps, stamps[1:])) < 300_000_000
        original_fence = client.authorization_fence()
        client.revoke_intent()
        with pytest.raises(ValueError, match='authorization revoked'):
            client.call('resume_output', '/control/emergency_stop', expected_fence=original_fence)
        wait_until(lambda:not client.snapshot()['command_mux']['output_enabled'])
        time.sleep(.05)
        stopped_count = len(loopback.actual)
        time.sleep(.12)
        assert len(loopback.actual) == stopped_count
    finally:
        release.set()
        recorder.stop()
        capture.close()
        client.stop()
        recorder.shutdown()


def test_dead_core_stops_and_marks_capture_incomplete_without_auto_recovery(loopback, tmp_path):
    fallbacks = []
    client = CoreProcessClient(config(), fallback_stop=fallbacks.append)
    recorder = TopicRecorder({'directory':'recordings'}, tmp_path,
                            topics=[{'topic':'/action/joint_states', 'type':'sensor_msgs/msg/JointState'}])
    client.capture_spec = {'resources':recorder.ipc_resources(), 'finish':recorder._finish_event}
    capture = RemotePublicationCapture(client, lambda:True)
    recorder.publication_capture = capture
    try:
        client.start()
        wait_until(lambda:client.snapshot().get('control_ready'))
        client.call('set_command_output_enabled', True)
        recorder.start('core-died.mcap', {'profile_id':'x1'})
        wait_until(lambda:capture.status()['enqueued'] >= 8)
        old_fence = client.authorization_fence()
        client.process.terminate()
        client.process.join(2.)
        wait_until(lambda:client.snapshot()['state'] == 'error')
        assert not client.snapshot()['command_mux']['output_enabled']
        assert len(fallbacks) == 1
        with pytest.raises(RuntimeError):
            client.call('resume_output', '/control/emergency_stop', expected_fence=old_fence)
        time.sleep(.08)
        stopped_count = len(loopback.actual)
        time.sleep(.15)
        assert len(loopback.actual) == stopped_count
        result = recorder.stop()
        assert result['error'], 'core loss must not produce a successful recording audit'
        assert 'incomplete' in capture.status()['error']
        assert not recorder.is_recording()
    finally:
        recorder.stop()
        capture.close()
        client.stop()
        recorder.shutdown()
