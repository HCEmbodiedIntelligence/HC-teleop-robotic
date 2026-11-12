"""Existing PyBullet simulator + actual VR/Motion/HC/capture, on both robot URDFs.

The existing simulator applies joint positions with resetJointState. This is a
kinematic/recording integration test, not a motor, contact or load-dynamics test.
All ROS messages are existing standard types. Nothing connects to hardware.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import itertools
import math
import os
from pathlib import Path
import sys
import time

import pytest
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from mcap.reader import make_reader

from tests.correction_recording_support import service as RecordingService
from tests.robot_api_fake_rig import FakeRig
from tests.correction_recording_support import config as recording_config


ROOT = Path(__file__).resolve().parents[1]
CAMERA = '/test/urdf_sim/color/image'


def run_simulator(profile, report):
    """Run the normal simulator in its own process, with a rendered test camera."""
    import numpy as np
    import pybullet as p
    from PIL import Image as PillowImage
    sys.path.insert(0, str(ROOT/'simulation'))
    from simulation.general_sim_robot_control_node_ros2 import SimRobotController
    from middleware.core.robot_api import RobotApiProfile

    api = RobotApiProfile(profile)
    report.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=['--ros-args', '-r', '/io_teleop/hardware_ready:=/status/hardware/ready',
        '-r', '/io_teleop/target_ee_poses:=/action/ee_poses'])
    simulator = SimRobotController(str(profile/'arm_teleop.yaml'), init_debug=False, headless=True)
    camera = simulator.create_publisher(Image, CAMERA, 1)
    phase = 'initial'
    saved = set()

    def ownership(message):
        nonlocal phase
        phase = json.loads(message.data)['state'].lower()

    simulator.create_subscription(String, '/status/control/ownership', ownership, 10)
    with simulator.bullet_lock:
        boxes = [p.getAABB(simulator.robot_id, index, physicsClientId=simulator.physics_clent_id)
                 for index in range(-1, p.getNumJoints(simulator.robot_id, physicsClientId=simulator.physics_clent_id))]
        lower = np.min([box[0] for box in boxes], axis=0)
        upper = np.max([box[1] for box in boxes], axis=0)
    center = ((lower+upper)/2).tolist()
    distance = float(np.max(upper-lower))*1.8
    view = p.computeViewMatrixFromYawPitchRoll(center, distance, 130, -20, 0, 2)
    projection = p.computeProjectionMatrixFOV(50, 4/3, .02, max(10., distance*3))
    (report/'model.json').write_text(json.dumps(dict(profile=api.profile_id,
        urdf=str(api.urdf_path), urdf_sha256=hashlib.sha256(api.urdf_path.read_bytes()).hexdigest(),
        simulator='simulation/general_sim_robot_control_node_ros2.py',
        command_application='resetJointState; kinematic simulation',
        camera=dict(width=320, height=240, requested_fps=15, renderer='PyBullet TinyRenderer'),
        joint_count=len(simulator.joint_name2id_dict)), indent=2))

    def render():
        with simulator.bullet_lock:
            pixels = p.getCameraImage(320, 240, view, projection, shadow=0,
                renderer=p.ER_TINY_RENDERER, physicsClientId=simulator.physics_clent_id)[2]
        rgb = np.asarray(pixels, dtype=np.uint8).reshape(240, 320, 4)[:, :, :3].copy()
        message = Image(height=240, width=320, encoding='rgb8', step=960, data=rgb.tobytes())
        message.header.stamp = simulator.get_clock().now().to_msg()
        message.header.frame_id = 'urdf_sim_camera'
        camera.publish(message)
        if phase in {'initial', 'auto', 'manual'} and phase not in saved:
            PillowImage.fromarray(rgb).save(report/(phase+'.png'))
            saved.add(phase)

    simulator.create_timer(1/15, render)
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.close_simulation()
        simulator.destroy_node()
        rclpy.try_shutdown()


class UrdfSimRig(FakeRig):
    domains = itertools.cycle(range(220, 228))

    def _start_driver(self):
        def feedback(message):
            self.positions.update(zip(message.name, message.position))
        self.node.create_subscription(JointState, '/internal/robot/joint_states', feedback, 10)
        self.spawn('pybullet', [sys.executable, '-m', 'tests.test_intervention_urdf_sim',
            '--plant', '--profile', str(self.api.directory), '--report', str(self.report)])

    def _feedback(self):
        # Only the actual simulator publishes measured URDF joint feedback.
        pass


@pytest.mark.parametrize('profile_name', ['x1', 'openarmx'])
def test_urdf_simulation_interventions_and_correction_capture(profile_name, tmp_path):
    report = Path(os.environ.get('HC_URDF_SIM_REPORT', str(tmp_path)))/profile_name
    with UrdfSimRig(ROOT/'adapters/robots'/profile_name, report, with_teleop=True) as rig:
        control = rig.bridge.ownership
        vr = rig.node.create_publisher(String, '/vrdata', 1)
        old_targets = rig.node.create_publisher(JointState, '/internal/control/joint_targets', 1)
        groups = rig.api.describe()['groups']
        programs = {side: rig.node.create_publisher(JointState, groups[side+'_arm']['endpoints']['servo_j'], 1)
                    for side in ('left', 'right')}
        homes = {side: [rig.positions[name] for name in rig.api.groups[side+'_arm']] for side in programs}
        frame = dict(protocol_version=3, sequence=0, vr_timestamp=0.,
            tracking=dict(head=True, left=True, right=True), poses={
                'head': dict(position=[0., 1.5, 0.], quaternion=[0., 0., 0., 1.]),
                'left': dict(position=[-.2, 1.3, -.4], quaternion=[0., 0., 0., 1.]),
                'right': dict(position=[.2, 1.3, -.4], quaternion=[0., 0., 0., 1.])},
            inputs=dict(left=dict(grip=0., trigger=.5), right=dict(grip=0., trigger=.5)))
        program_poison = False
        human_actions = []

        def feed():
            frame.update(sequence=frame['sequence']+1, vr_timestamp=time.monotonic(), hc_received_ns=time.time_ns())
            vr.publish(String(data=json.dumps(frame)))
            for side, publisher in programs.items():
                target = homes[side].copy()
                target[0] += .3 if program_poison else .01*math.sin(time.monotonic())
                message = JointState(name=groups[side+'_arm']['joint_names'], position=target)
                message.header.stamp = rig.node.get_clock().now().to_msg()
                publisher.publish(message)

        timer = rig.node.create_timer(.01, feed)
        cfg = recording_config(report/'capture'/str(time.time_ns()))
        cfg['max_pending_jobs'] = 8
        cfg['corrections'] = dict(prepared_slots=4, late_message_sec=.05)
        recorder = RecordingService(cfg, report, dict(enabled=True, domain_id=rig.domain, subscriptions=[
            dict(topic='/observation/joint_states', type='sensor_msgs/msg/JointState', outputs=['record']),
            dict(topic=CAMERA, type='sensor_msgs/msg/Image', outputs=['record'])]))
        control.open_segment = recorder.correction_open
        control.close_segment = recorder.correction_close
        def sent(event):
            human_actions.append(event)
            recorder.correction_action(event)
        rig.bridge._action_sink = sent
        intervals, motion, gripper_direction = [], [], {}
        try:
            control.enable()
            with ThreadPoolExecutor(max_workers=1) as preparing:
                future = preparing.submit(recorder.prepare_corrections, dict(profile_id=profile_name, simulation=True))
                rig.until(future.done, 20)
                future.result()
            rig.until(lambda: recorder.corrections.status()['ready'] and control.input_armed and len(control.peers) == 2, 20)
            rig.until(rig.bridge.control_ready)
            control.start_auto(recording_ready=True, control_ready=rig.bridge.control_ready())
            rig.until(lambda: control.state == 'AUTO')
            rig.pump(.3)
            durations = [.6]+[.1, .2, .3]*4
            for index, duration in enumerate(durations):
                frame['inputs']['right']['grip'] = 1.
                rig.until(lambda: control.state == 'MANUAL')
                token = control.token
                before = rig.positions.copy()
                program_poison = True
                frame['poses']['left']['position'][2] -= .004
                frame['poses']['right']['position'][2] -= .003
                if profile_name == 'openarmx':
                    for side, _, initial, _, opened, closed in rig.teleop.intervention.grippers.values():
                        if index == 0:
                            gripper_direction[side] = .04 if initial > (opened+closed)/2 else -.04
                        frame['inputs'][side]['trigger'] += gripper_direction[side] * (-1 if index % 2 else 1)
                rig.pump(duration)
                (report/'last-cycle.json').write_text(json.dumps(dict(cycle=index,
                    control=control.snapshot(), bridge=rig.bridge.status()['command_mux'],
                    teleop=dict(granted=rig.teleop.intervention.granted,
                        phase=rig.teleop.intervention.phase,
                        token=rig.teleop.intervention.token.key(),
                        valid=rig.teleop.intervention.valid,
                        feedback_age=time.monotonic()-rig.teleop.joint_state_stamp),
                    pose_bridge=dict(key=rig.vr.ownership_key, phase=rig.vr.ownership_phase,
                        authorized=rig.vr.authorized(), cutoff=rig.vr.cutoff,
                        permit_age=time.monotonic()-rig.vr.permit_time)), indent=2))
                assert control.state == 'MANUAL', control.snapshot()
                assert control.command_seq > 0 and not control.recording_error, control.snapshot()
                for side in programs:
                    names = rig.api.groups[side+'_arm']
                    moved = max(abs(rig.positions[name]-before[name]) for name in names)
                    motion.append(dict(side=side, cycle=index, max_joint_delta=moved))
                    # The initial sustained window verifies visible model
                    # motion. Short windows verify both final command streams
                    # and capture boundaries without assuming zero render /
                    # feedback latency in this kinematic simulator.
                    if index == 0:
                        assert moved > 1e-5, (side, index, moved)
                    assert any(event['intervention_id'] == token.intervention_id and
                               groups[side+'_arm']['joint_names'][0] in event['names'] for event in human_actions)
                    assert abs(rig.positions[names[0]]-homes[side][0]) < .1, 'program goal leaked into manual control'
                allowed = {name for side in programs for name in rig.api.groups[side+'_arm']}
                if profile_name == 'openarmx':
                    references = rig.teleop.intervention.grippers
                    assert len(references) == 2
                    for name, reference in references.items():
                        allowed.add(name)
                        if index == 0:
                            assert abs(rig.positions[name]-reference[2]) > 1e-5, 'gripper did not follow relative trigger'
                assert all(abs(rig.positions[name]-value) < 1e-6 for name, value in before.items()
                           if name not in allowed), 'passive body or unsupported gripper moved'
                intervals.append(duration)
                program_poison = False
                frame['inputs']['right']['grip'] = 0.
                rig.until(lambda: control.state == 'AUTO')
                stale = JointState(name=groups['left_arm']['joint_names'], position=[value+.15 for value in homes['left']])
                stale.header.stamp = rig.node.get_clock().now().to_msg()
                stale.header.frame_id = token.key()+'#'
                old_targets.publish(stale)
                rig.pump(.04)
                assert abs(rig.positions[rig.api.groups['left_arm'][0]]-homes['left'][0]) < .1

            control.hold()
            frame['inputs']['right']['grip'] = 1.
            rig.until(lambda: control.state == 'MANUAL')
            rig.pump(.15)
            frame['inputs']['right']['grip'] = 0.
            rig.until(lambda: control.state == 'HOLD')
            assert not rig.bridge._command_output_enabled
            frame['inputs']['right']['grip'] = 1.
            rig.until(lambda: control.state == 'MANUAL')
            rig.pump(.15)
            frame['tracking']['right'] = False
            frame['inputs']['right']['grip'] = 0.
            rig.until(lambda: control.state == 'FAULT')
            frame['tracking']['right'] = True
            rig.pump(.2)
            assert control.state == 'FAULT'
            rig.bridge.recover_intervention()
            rig.until(lambda: control.state == 'HOLD')
            assert not rig.bridge._command_output_enabled
            control.disable()
            rig.until(lambda: len(recorder.corrections.jobs) == len(durations)+2 and
                      all(job['capture_state'] == 'source_complete' for job in recorder.corrections.jobs.values()), 20)

            files = []
            for job in recorder.corrections.jobs.values():
                with open(job['path'], 'rb') as stream:
                    entries = list(make_reader(stream).iter_messages())
                evidence = [json.loads(message.data) for _, channel, message in entries
                            if channel.topic == '/events/control/action_evidence']
                frames = [deserialize_message(message.data, Image) for _, channel, message in entries if channel.topic == CAMERA]
                assert evidence and all(event['source'] == 'vr' and event['intervention'] for event in evidence)
                assert len({event['intervention_id'] for event in evidence}) == 1
                assert frames and all(image.width == 320 and image.height == 240 for image in frames)
                assert job['quality']['missing_actions'] == 0 and not job['quality']['missing_topics']
                files.append(dict(path=job['path'], frames=len(frames), actions=len(evidence), quality=job['quality']))
            assert (report/'manual.png').is_file()
            metrics = list(control.metrics)
            def summary(field):
                values = [value[field]/1e6 for value in metrics if field in value]
                return dict(count=len(values), min_ms=min(values), max_ms=max(values), mean_ms=sum(values)/len(values))
            result = dict(model=json.loads((report/'model.json').read_text()), ros_domain=rig.domain,
                normal_cycles=len(intervals), manual_durations_sec=intervals, correction_segments=len(files),
                short_cycles=sum(duration <= .3 for duration in intervals),
                final_human_commands=len(human_actions), motion=motion, recordings=files,
                release_to_permission=summary('release_to_open_ns'),
                permission_to_first_final_target=summary('open_to_first_final_command_ns'),
                checks=dict(both_arms_moved=True, program_rejected_during_manual=True,
                    grippers_moved=profile_name == 'openarmx', passive_axes_held=True,
                    stale_generation_rejected=True, paused_release_stays_hold=True,
                    tracking_loss_latches_fault=True, recovery_stays_hold=True,
                    real_rendered_observations_saved=True, only_human_actions_saved=True),
                physical_hardware_response='not measured; existing kinematic URDF simulator')
            (report/'result.json').write_text(json.dumps(result, indent=2))
        finally:
            control.disable()
            recorder.shutdown()
            rig.node.destroy_timer(timer)


def test_pose_router_handles_missed_revoke_without_retagging_old_pose():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from geometry_msgs.msg import Pose, PoseArray
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    from adapters.nodes.unified_motion_teleop import UnifiedMotionTeleopBridge as Bridge
    from middleware.core.control_ownership import ControlToken

    old = ControlToken('controller', 'session', 4, 'motion', '').key()
    current = ControlToken('controller', 'session', 5, 'vr', 'segment').key()
    router = SimpleNamespace(times=lambda: (time.monotonic(), 100.2), stamp=Bridge.stamp,
        permit_stamp=100., permit=old+'#grant#motion#teleop', ownership_key=old,
        ownership_phase='grant', intervention=True, cancel_home=Mock(), cache=Mock(),
        sides=('right', 'left'), base_frame='base', pose_pubs={'right': Mock(), 'left': Mock()})
    router.authorized = lambda: Bridge.authorized(router)
    # The preceding revoke was overwritten before this subscriber ran.
    header = Header(stamp=Time(sec=100, nanosec=100_000_000),
        frame_id=current+'#grant#motion#teleop')
    Bridge.permit_received(router, header)
    assert router.authorized()
    for key in (old, current):
        pose = PoseArray(header=Header(stamp=Time(sec=100, nanosec=150_000_000),
            frame_id=key+'#base'), poses=[Pose(), Pose()])
        Bridge.owned_pose(router, pose)
    for publisher in router.pose_pubs.values():
        publisher.publish.assert_called_once()
        assert publisher.publish.call_args.args[0].header.frame_id == current+'#base'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--plant', action='store_true', required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    run_simulator(args.profile, args.report)
