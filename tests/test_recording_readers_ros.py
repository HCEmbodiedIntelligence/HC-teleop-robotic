"""Localhost-only evidence: stalled image consumer cannot queue state history."""
import os
import signal
import struct
import threading
import time
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
from mcap.reader import make_reader
import rclpy
from rclpy.context import Context
from sensor_msgs.msg import JointState, CompressedImage

from middleware.core.ros_recording_executor import RosRecordingExecutor
from middleware.core.topic_recorder import TopicRecorder


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic()+timeout
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.01)


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '231' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
                    reason='requires explicitly reserved localhost ROS domain 231')
def test_stalled_image_reader_does_not_drop_small_headers_or_truncate_fixed_window(tmp_path):
    context = Context()
    rclpy.init(context=context, domain_id=231)
    node = rclpy.create_node('test_independent_recording_sources', context=context)
    small = ['/observation/joint_states', '/control/joint_cmd', '/control/solver/joint_targets']
    images = ['/observation/cameras/head/'+kind+'/compressed' for kind in ('color', 'depth')]
    items = [dict(topic=topic, type='sensor_msgs/msg/JointState', outputs=['record']) for topic in small]
    items += [dict(topic=topic, type='sensor_msgs/msg/CompressedImage', outputs=['record']) for topic in images]
    publishers = {topic: node.create_publisher(JointState if topic in small else CompressedImage, topic, 10)
                  for topic in small+images}
    recorder = TopicRecorder({'directory':'captures'}, tmp_path, topics=items)
    config = dict(enabled=True, domain_id=231, legacy_control_profile='x1',
                  isolated_control_process=True, subscriptions=items)
    receiver = RosRecordingExecutor(config, recorder)
    stopped = threading.Event()
    published = []
    def produce():
        sequence, deadline = 0, time.monotonic()
        payload = b'raw-recording-payload'*8000
        while not stopped.is_set():
            ns = time.time_ns()
            state = JointState(name=['test_joint'], position=[sequence*.00001])
            state.header.stamp.sec, state.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            for topic in small:
                publishers[topic].publish(state)
            published.append(ns)
            if sequence % 3 == 0:
                image = CompressedImage(header=state.header, format='jpeg', data=payload)
                for topic in images:
                    publishers[topic].publish(image)
            sequence += 1
            deadline += .01
            stopped.wait(max(0., deadline-time.monotonic()))
    producer = threading.Thread(target=produce, daemon=True)
    paused = None
    try:
        receiver.prepare_recording()
        producer.start()
        wait_until(lambda: all(pub.get_subscription_count() for pub in publishers.values()))
        wait_until(lambda: all(x.get('received',0) > 10 for x in receiver.status()['readers']))
        path = Path(recorder.start('reader-isolation.mcap', {'profile_id':'x1'}))
        status = receiver.status()
        assert {worker['group'] for worker in status['readers']} == {'small','images'}
        assert status['priority'] == 'normal'
        image_pid = next(worker['pid'] for worker in status['readers'] if worker['group'] == 'images')
        small_pid = next(worker['pid'] for worker in status['readers'] if worker['group'] == 'small')
        assert image_pid != small_pid
        time.sleep(.4)
        os.kill(image_pid, signal.SIGSTOP)
        paused = image_pid
        time.sleep(.35)
        # Reader status runs at 2 Hz; the MCAP equality assertion below proves
        # every state packet, even while the entire image process was stopped.
        os.kill(image_pid, signal.SIGCONT)
        paused = None
        time.sleep(.65)
        result = recorder.stop()
        assert not result['error'], result
        assert result['accepted'] == result['messages']
        assert result['dropped'] == 0
        window = result['capture_window']
        assert window['late_arrival_budget_ns'] == 250_000_000
        assert 250_000_000 <= window['raw_ingress_closed_ns']-window['stop_requested_ns'] < 350_000_000
        source = {stamp for stamp in published if window['window_start_ns'] <= stamp <= window['stop_requested_ns']}
        rows = {topic:[] for topic in small}
        with path.open('rb') as stream:
            for _schema, channel, message in make_reader(stream).iter_messages():
                if channel.topic in rows:
                    sec,nano = struct.unpack_from('<iI', message.data, 4)
                    rows[channel.topic].append(sec*1_000_000_000+nano)
        for topic, stamps in rows.items():
            assert len(stamps) >= 100, topic
            assert set(stamps) == source, (topic, len(source-set(stamps)), len(set(stamps)-source))
            assert len(stamps) == len(set(stamps))
            assert window['stop_requested_ns']-max(stamps) < 40_000_000
            assert min(stamps)-window['window_start_ns'] < 40_000_000
        receiver.stop()
        assert all(worker['process'] is None for worker in receiver._workers)
    finally:
        if paused is not None:
            os.kill(paused, signal.SIGCONT)
        stopped.set()
        if producer.is_alive():
            producer.join(2.)
        recorder.stop()
        receiver.stop()
        recorder.shutdown()
        node.destroy_node()
        context.try_shutdown()
