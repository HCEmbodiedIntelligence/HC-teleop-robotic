"""Real Motion/driver/final HC gate; simulated tracked device and fake plant only."""
import json
import os
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor
import pytest

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, String
from mcap.reader import make_reader

from tests.robot_api_fake_rig import FakeRig
from tests.correction_recording_support import config as recording_config
from tests.correction_recording_support import service as RecordingService


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('capture_mode', ['corrections_only', 'full_session'])
def test_real_motion_switches_short_corrections_and_rejects_old_generations(tmp_path, capture_mode):
    report = Path(os.environ.get('HC_INTERVENTION_REPORT', str(tmp_path/'rig')))
    with FakeRig(ROOT/'tests/fixtures/robot_api_six_dof', report) as rig:
        control = rig.bridge.ownership
        peer = 'fake-tracked-teleop'
        ack_pub = rig.node.create_publisher(String, '/status/control/ack', 100)
        input_pub = rig.node.create_publisher(String, '/events/teleop/clutch', 100)
        owned = rig.node.create_publisher(JointState, '/internal/control/joint_targets', 1)
        camera = rig.node.create_publisher(Image, '/test/color/image', 1)
        group = rig.api.describe()['groups']['left_arm']
        program = rig.node.create_publisher(JointState, group['endpoints']['servo_j'], 1)
        phase, key, gripped, sequence = '', '', False, 0
        enabled = True

        def acknowledge():
            ack_pub.publish(String(data=json.dumps(dict(participant='teleop', instance=peer,
                stamp_ns=rig.node.get_clock().now().nanoseconds, token=key, phase=phase, success=True))))

        def permit(message):
            nonlocal phase, key
            if message.frame_id.startswith('hc1:'):
                identity, stage, _, target = message.frame_id.split('#')
                if target == peer:
                    key, phase = identity, stage
                    acknowledge()

        rig.node.create_subscription(Header, '/control/motion/permit', permit, 1)

        def feed():
            nonlocal sequence
            if not enabled:
                return
            sequence += 1
            stamp = rig.node.get_clock().now().to_msg()
            now = stamp.sec*10**9+stamp.nanosec
            input_pub.publish(String(data=json.dumps(dict(device_instance=peer, sequence=sequence,
                stamp_ns=now, valid=True, gripped=gripped))))
            acknowledge()
            image = Image(height=4, width=4, encoding='rgb8', step=12, data=bytes([64, 128, 192]*16))
            image.header.stamp = stamp
            camera.publish(image)
            msg = JointState(name=group['joint_names'], position=[.01]*len(group['joint_names']))
            msg.header.stamp = stamp
            program.publish(msg)  # The unmodified external program never stops.
            if gripped and control.state == 'MANUAL':
                msg.header.frame_id = control.token.key()+'#'
                msg.position = [.02]*len(msg.name)
                owned.publish(msg)

        timer = rig.node.create_timer(.01, feed)
        cfg = recording_config(report/'capture'/str(time.time_ns()))
        cfg['corrections'] = dict(prepared_slots=4, late_message_sec=.05)
        ros = dict(enabled=True, domain_id=rig.domain, subscriptions=[
            dict(topic='/observation/joint_states', type='sensor_msgs/msg/JointState', outputs=['record']),
            dict(topic='/action/joint_states', type='sensor_msgs/msg/JointState', outputs=['record']),
            dict(topic='/test/color/image', type='sensor_msgs/msg/Image', outputs=['record'])])
        recorder = RecordingService(cfg, report, ros)
        control.open_segment = recorder.correction_open
        control.close_segment = recorder.correction_close
        rig.bridge._action_sink = recorder.correction_action
        try:
            control.enable()
            with ThreadPoolExecutor(max_workers=1) as preparing:
                future = preparing.submit(recorder.prepare_corrections, {'profile_id': rig.api.profile_id, 'simulation': True}, capture_mode=capture_mode)
                rig.until(future.done, 15)
                future.result()
            rig.until(lambda: recorder.corrections.status()['ready'] and control.input_armed and len(control.peers) == 2, 15)
            assert control.state == 'HOLD', control.snapshot()
            rig.until(rig.bridge.control_ready)
            control.start_auto(recording_ready=True, control_ready=rig.bridge.control_ready())
            rig.until(lambda: control.state == 'AUTO')
            rig.pump(.15)
            count = int(os.environ.get('HC_INTERVENTION_CYCLES', '100'))
            identities = set()
            for index in range(count):
                gripped = True
                rig.until(lambda: control.state == 'MANUAL')
                old = control.token
                identities.add(old.intervention_id)
                rig.pump((.1, .2, .3)[index % 3])
                assert control.command_seq > 0
                gripped = False
                rig.until(lambda: control.state == 'AUTO')
                # A computation from the previous epoch arrives after return.
                old_target = JointState(name=group['joint_names'], position=[.4]*len(group['joint_names']))
                old_target.header.stamp = rig.node.get_clock().now().to_msg()
                old_target.header.frame_id = old.key()+'#'
                owned.publish(old_target)
                rig.pump(.025)
                assert all(abs(rig.positions[name]) < .05 for name in rig.api.groups['left_arm'])
            assert len(identities) == count
            control.hold()
            gripped = True
            rig.until(lambda: control.state == 'MANUAL')
            rig.pump(.1)
            gripped = False
            rig.until(lambda: control.state == 'HOLD')
            assert not rig.bridge._command_output_enabled
            control.disable()
            if capture_mode == 'full_session':
                with ThreadPoolExecutor(max_workers=1) as closing:
                    future = closing.submit(recorder.stop_corrections)
                    rig.until(future.done, 20)
                    future.result()
                jobs = recorder.records()
                assert len(jobs) == 1
                job = jobs[0]
                assert job['capture_state'] == 'source_complete'
                session = json.loads(Path(job['path']).with_suffix('.session.json').read_text())
                intervals = session['interventions']
                assert len(intervals) == count+1
                assert not Path(job['path']).with_suffix('.correction.json').exists()
                with open(job['path'], 'rb') as stream:
                    messages = list(make_reader(stream).iter_messages())
                actions = [msg.log_time for _, channel, msg in messages if channel.topic == '/action/joint_states']
                assert any(not any(i['start_ns'] <= stamp < i['end_ns'] for i in intervals) for stamp in actions)
                assert any(any(i['start_ns'] <= stamp < i['end_ns'] for i in intervals) for stamp in actions)
                observations = [msg.log_time for _, channel, msg in messages if channel.topic == '/observation/joint_states']
                assert min(observations) < intervals[0]['start_ns']
                assert max(observations) >= intervals[-1]['end_ns']
                events = [json.loads(msg.data) for _, channel, msg in messages
                          if channel.message_encoding == 'json' and channel.topic == '/events/control/intervention']
                assert sum(event['kind'] == 'manual_open' for event in events) == count+1
                assert sum(event['kind'] == 'session_close' for event in events) == 1
                return
            # Keep DDS spinning while background files complete, independent
            # of control return. No camera/hardware service is started here.
            rig.until(lambda: len(recorder.corrections.jobs) == count+1 and
                      all(job['capture_state'] == 'source_complete' for job in recorder.corrections.jobs.values()), 20)
            assert all(job['quality']['action_received'] > 0 for job in recorder.corrections.jobs.values())
            for job in recorder.corrections.jobs.values():
                with open(job['path'], 'rb') as stream:
                    events = [json.loads(msg.data) for _, channel, msg in make_reader(stream).iter_messages()
                              if channel.topic == '/events/control/action_evidence']
                assert events and all(event['source'] == 'vr' and event['intervention'] for event in events)
                assert len({event['intervention_id'] for event in events}) == 1
            metrics = list(control.metrics)
            def distribution(field):
                values = sorted(item[field]/1e6 for item in metrics if field in item)
                return dict(count=len(values), p50_ms=values[len(values)//2],
                            p99_ms=values[min(len(values)-1, int(len(values)*.99))], max_ms=max(values)) if values else {}
            result = dict(cycles=count, rig='real Motion + HC + driver, fake teleop receiver/plant',
                release_to_permission=distribution('release_to_open_ns'),
                permission_to_first_final_program_target=distribution('open_to_first_final_command_ns'),
                physical_hardware_response='not measured; fake plant only')
            (report/'latency.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(result))
        finally:
            control.disable()
            recorder.shutdown()
            enabled = False
            rig.node.destroy_timer(timer)
