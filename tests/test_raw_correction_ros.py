"""Real normalized ROS inputs, prearmed raw writers, short windows and sealing."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, JointState
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message

from middleware.core.control_ownership import ControlToken
from tests.correction_recording_support import service as RecordingService
from tests.correction_recording_support import config as recording_config


def test_raw_short_corrections_preserve_images_actions_and_unbound_download(tmp_path):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    context = Context()
    rclpy.init(context=context, domain_id=218)
    node = rclpy.create_node('correction_raw_fake_inputs', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publishers = {topic: node.create_publisher(kind, topic, 10) for topic, kind in (
        ('/test/raw/state', JointState), ('/test/raw/rgb', Image), ('/test/raw/depth', Image))}
    cfg = recording_config(tmp_path)
    cfg.update(format='mcap_mp4', corrections=dict(prepared_slots=4, late_message_sec=.05))
    recorder = RecordingService(cfg, tmp_path, dict(enabled=True, domain_id=218, subscriptions=[
        dict(topic=topic, type='sensor_msgs/msg/'+('JointState' if topic.endswith('state') else 'Image'), outputs=['record'])
        for topic in publishers]))
    current = None
    sequence = 0

    def feed():
        nonlocal sequence
        stamp = node.get_clock().now().to_msg()
        state = JointState(name=['arm'], position=[.125])
        state.header.stamp = stamp
        publishers['/test/raw/state'].publish(state)
        for topic, encoding, step, data in (
            ('/test/raw/rgb', 'rgb8', 48, bytes([64, 128, 192]*256)),
            ('/test/raw/depth', '16UC1', 32, bytes([232, 3]*256))):
            image = Image(height=16, width=16, encoding=encoding, step=step, data=data)
            image.header.stamp = stamp
            publishers[topic].publish(image)
        if current:
            sequence += 1
            recorder.correction_action(dict(asdict(current), stamp_ns=time.time_ns(), command_seq=sequence,
                action_stage='sent', action_boundary='hc_final', intervention=True, names=['arm'], positions=[.25]))

    timer = node.create_timer(.01, feed)
    def pump(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.002)
    def until(predicate, timeout=15):
        end = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < end, recorder.corrections.status() if recorder.corrections else recorder.status()
            pump(.01)
    try:
        recorder.prepare_corrections({'profile_id': 'fake-raw'})
        until(lambda: recorder.corrections.status()['ready'])
        pid = recorder.corrections.process.pid
        for index in range(6):
            current = ControlToken('controller', 'session', index*2+1, 'vr', 'manual'+str(index))
            sequence = 0
            recorder.correction_open(current, time.time_ns())
            pump((.1, .2, .3)[index % 3])
            recorder.correction_close(current, time.time_ns(), sequence, 'normal release')
            current = None
            assert recorder.corrections.process.pid == pid
        until(lambda: len(recorder.corrections.jobs) == 6 and
              all(job['capture_state'] == 'source_complete' for job in recorder.corrections.jobs.values()))
        until(lambda: not recorder.pending, 30)
        for job in recorder.records():
            assert job['state'] == 'ready', job
            source = Path(job['path'])
            bundle = recorder.directory/'bundles'/source.stem
            manifest = json.loads((bundle/'manifest.json').read_text())
            assert list((bundle/'videos').glob('*.mp4'))
            assert job['quality']['missing_actions'] == 0
            correction = json.loads((bundle/'correction.json').read_text())
            assert correction['intervention'] is True and correction['outcome'] == 'unknown'
            assert any(item['path'] == 'correction.json' for item in manifest['committed'])
            with (bundle/'raw.mcap').open('rb') as stream:
                messages = list(make_reader(stream).iter_messages())
            depths = [deserialize_message(msg.data, Image) for _, channel, msg in messages
                      if channel.topic == '/test/raw/depth']
            assert depths and all(bytes(image.data) == bytes([232, 3]*256) for image in depths)
            evidence = [json.loads(msg.data) for _, channel, msg in messages
                        if channel.topic == '/events/control/action_evidence']
            assert evidence and all(event['intervention_id'] == job['metadata']['intervention_id'] for event in evidence)
            from middleware.core.recording_datasets import archive_dataset
            archive = archive_dataset(bundle)
            archive.unlink()
        recorder.stop_corrections()
        assert len(recorder.jobs) == 6  # Empty warmed slots never become recordings.
    finally:
        if current:
            recorder.correction_close(current, time.time_ns(), sequence, 'test shutdown')
        recorder.shutdown()
        node.destroy_timer(timer)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
