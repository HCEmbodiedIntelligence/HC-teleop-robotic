"""Old-topic ownership, measured feedback and no-recording regressions."""
import asyncio
import copy
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
import pytest

pytest.importorskip('sensor_msgs.msg')
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from middleware.core.control_ownership import ControlOwnership
from middleware.core.intervention_api import register_intervention_routes
from middleware.core.legacy_control import LegacyTopicControl
from middleware.core.ros_bridge import RosBridge
from adapters.core.v23_intervention import V23Intervention

ROOT = Path(__file__).resolve().parents[1]


def test_shipped_profile_loads_legacy_mode_and_recording_off():
    from middleware.core.config import ConfigStore, validate_config, ConfigError
    settings = ConfigStore(ROOT/'middleware/config.yaml').load()['ros']['intervention']
    assert settings['legacy_topics'] is True
    assert settings['automatic_recording'] is False
    for field, value in [('automatic_recording', 'false'), ('max_arm_velocity_rad_s', float('nan'))]:
        with pytest.raises(ConfigError):
            validate_config({'ros': {'intervention': {field: value}}})


def message(values, stamp=None):
    result = JointState(name=list(values), position=list(values.values()))
    ns = time.time_ns() if stamp is None else stamp
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    return result


class Rig:
    def __init__(self):
        self.wire = []
        node = Mock()
        node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(nanoseconds=time.time_ns())
        node.create_publisher.side_effect = lambda kind, topic, qos: SimpleNamespace(
            publish=lambda msg: self.wire.append((topic, copy.deepcopy(msg))))
        self.bridge = RosBridge({'legacy_control_profile': str(ROOT/'adapters/robots/x1'),
            'intervention': {'automatic_recording': False}}, lambda *a: None)
        self.bridge._command_callback_group = self.bridge._telemetry_callback_group = None
        with patch('adapters.core.collision_safety.ModelCollisionGuard') as guard:
            guard.return_value.check.return_value = (True, None, False)
            self.boundary = LegacyTopicControl(node, self.bridge, node.create_publisher(JointState, '/action/joint_states', 10))
            self.boundary.collision = guard.return_value
        self.bridge._boundary = self.boundary
        self.control = self.bridge.ownership
        self.control.open_segment = Mock(side_effect=AssertionError('recording must remain off'))
        for peer in self.control.required_peers:
            self.control.peer(peer, peer)
        self.control.enable()
        self.control.input(device_instance='vr', sequence=1, valid=True, gripped=False, stamp_ns=time.time_ns())
        self.feed()
        self.control.start_auto(recording_ready=False, control_ready=self.bridge.control_ready())
        self.advance()
        self.advance()

    def feed(self):
        b = self.boundary
        positions = {n: float(b.teleop['robot']['initial_joints'][n]) for n in b.feedback_order}
        msg = message(positions)
        msg.velocity = [0.]*len(msg.name)
        b.state(msg)
        for side, names in b.hands.items():
            b.hand_state(message(dict(zip(names, b.teleop['grippers'][side]['finger_open']))))
        self.bridge.receive_hardware_ready(True)

    def advance(self):
        for peer in self.control.required_peers:
            self.control.acknowledge(self.control.token, self.control.phase, peer, peer)
        self.control.advance(control_ready=True, manual_ready=True)
        if self.control.state in ('AUTO','MANUAL'):
            self.bridge._command_source = self.control.token.source
            self.bridge._source_cutoff = self.control.cutoff_ns*1e-9
            self.bridge._command_output_enabled = True

    def trigger(self, sequence, value, *, grip=False, valid=True):
        return self.bridge.receive_intervention_input(dict(
            device_instance='vr', sequence=sequence, valid=valid,
            gripped=grip, right_gripper_trigger=value, stamp_ns=time.time_ns()))


@pytest.mark.parametrize('actor', [False, True])
def test_right_trigger_interrupts_only_the_right_hand(actor):
    rig = Rig()
    owner, boundary = rig.control, rig.boundary
    if actor:
        rig.bridge._control_actor_mode = True
        rig.bridge._actor_thread_ident = threading.get_ident()
    original_token, output_epoch = owner.token, rig.bridge._output_epoch
    right = dict(zip(boundary.hands['right'], boundary.right_gripper['finger_closed']))
    left = {name: boundary.positions[name] for name in boundary.hands['left']}
    arm = boundary.arms[0]

    rig.trigger(2, 1.)
    assert owner.right_gripper_source == 'motion'  # Requires a fresh release.
    rig.trigger(3, 0.)
    rig.trigger(4, .8)
    assert owner.right_gripper_source == 'vr'
    rig.trigger(5, .8)
    rig.bridge._drain_actor_targets()
    right_topic = '/action/right_hand/joint_states'
    assert [topic for topic, _ in rig.wire] == [right_topic]
    _, command = rig.wire[-1]
    assert command.name[-1] == 'R_ban'
    assert command.position[-1] == 1.
    assert max(abs(value-boundary.positions[name])
               for name, value in zip(command.name[:-1], command.position)) <= .035+1e-9

    boundary.program(message(right), 'right')
    boundary.program(message(left), 'left')
    rig.bridge._drain_actor_targets()
    boundary.program(message({arm: boundary.positions[arm], 'R_ban': 1.5, 'L_ban': .2}), 'arms')
    rig.bridge._drain_actor_targets()
    assert [topic for topic, _ in rig.wire] == [
        right_topic, '/action/left_hand/joint_states', '/action/joint_states']
    _, command = rig.wire[-1]
    labels = dict(zip(command.name, command.position))
    assert labels['R_ban'] == boundary.right_gripper_position < .1
    assert labels['L_ban'] == .2
    rig.trigger(6, .08)
    assert owner.right_gripper_source == 'vr'  # Hysteresis.
    rig.bridge._drain_actor_targets()
    _, previous = rig.wire[-1]

    rig.trigger(7, 0.)
    assert owner.right_gripper_source == 'motion'
    boundary.program(message(right), 'right')
    rig.bridge._drain_actor_targets()
    assert rig.wire[-1][0] == right_topic
    assert max(abs(a-b) for a,b in zip(previous.position, rig.wire[-1][1].position)) <= .175+1e-9
    assert owner.state == 'AUTO' and owner.token == original_token
    assert rig.bridge._output_epoch == output_epoch


def test_right_gripper_cutoff_rejects_inflight_model_frame_after_handoff():
    rig = Rig()
    boundary = rig.boundary
    right = {name: boundary.positions[name] for name in boundary.hands['right']}
    command = message(right, time.time_ns()+20_000_000)
    prepare = boundary.prepare

    def switch_hand_while_preparing(target):
        rig.trigger(2, 0.)
        rig.trigger(3, .8)
        rig.trigger(4, 0.)
        return prepare(target)

    with patch.object(boundary, 'prepare', side_effect=switch_hand_while_preparing):
        boundary.program(command, 'right')
    assert rig.control.state == 'AUTO' and rig.control.right_gripper_source == 'motion'
    assert not rig.wire
    boundary.program(message(right), 'right')
    assert [topic for topic, _ in rig.wire] == ['/action/right_hand/joint_states']


def test_actor_discards_right_hand_queued_before_two_handoffs():
    rig = Rig()
    bridge, boundary = rig.bridge, rig.boundary
    bridge._control_actor_mode = True
    bridge._actor_thread_ident = threading.get_ident()
    right = {name: boundary.positions[name] for name in boundary.hands['right']}
    boundary.program(message(right, time.time_ns()+20_000_000), 'right')
    assert ('motion', 'right_hand') in bridge._actor_targets
    rig.trigger(2, 0.)
    rig.trigger(3, .8)
    rig.trigger(4, 0.)
    bridge._drain_actor_targets()
    assert not rig.wire
    boundary.program(message(right), 'right')
    bridge._drain_actor_targets()
    assert [topic for topic, _ in rig.wire] == ['/action/right_hand/joint_states']


def test_right_gripper_input_loss_waits_for_release_and_full_grip_still_works():
    rig = Rig()
    owner, boundary = rig.control, rig.boundary
    right = {name: boundary.positions[name] for name in boundary.hands['right']}
    arm = boundary.arms[0]
    rig.trigger(2, 0.)
    rig.trigger(3, .8)
    rig.trigger(4, .8, valid=False)
    assert owner.state == 'AUTO' and owner.right_gripper_source == 'hold'
    rig.trigger(5, .8)
    boundary.program(message(right), 'right')
    boundary.program(message({arm: boundary.positions[arm]}), 'arms')
    assert [topic for topic, _ in rig.wire] == ['/action/joint_states']
    rig.trigger(6, 0.)
    rig.trigger(7, .8)
    rig.trigger(8, .8, grip=True)
    assert owner.state == 'TAKEOVER' and owner.right_gripper_token is None
    rig.advance()
    rig.advance()
    assert owner.state == 'MANUAL'
    boundary.receive_owned(message(right), owner.token)
    assert rig.wire[-1][0] == '/action/right_hand/joint_states'
    rig.trigger(9, .8)
    assert owner.state == 'RETURNING'
    rig.advance()
    rig.advance()
    assert owner.state == 'AUTO' and owner.right_gripper_source == 'vr'
    rig.trigger(10, .8)
    assert rig.wire[-1][0] == '/action/right_hand/joint_states'


def test_grip_return_keeps_a_fixed_arm_target_until_the_first_safe_model_arm_send():
    rig = Rig()
    owner, bridge, boundary = rig.control, rig.bridge, rig.boundary
    clock = [owner.clock()]
    owner.clock = lambda: clock[0]
    rig.trigger(2, 0., grip=True)
    rig.advance()
    rig.advance()
    rig.trigger(3, 0.)
    assert owner.state == 'RETURNING' and not bridge._command_output_enabled
    bridge._tick_return_hold()
    tag = 'hc-dagger/return-hold'
    assert rig.wire[-1][1].header.frame_id == tag
    anchor = dict(bridge._return_hold.positions)
    rig.advance()
    rig.advance()
    assert owner.state == 'AUTO'
    for sequence in range(4, 42):
        clock[0] += 20_000_000
        rig.trigger(sequence, 0.)
        for peer in owner.required_peers:
            owner.peer(peer, peer)
        bridge._tick_return_hold()
        assert rig.wire[-1][1].header.frame_id == tag
        names, values = rig.wire[-1][1].name, rig.wire[-1][1].position
        assert {n: q for n, q in zip(names, values) if n in anchor} == anchor

    right = {name: boundary.positions[name] for name in boundary.hands['right']}
    boundary.program(message(right), 'right')
    assert bridge._return_hold is not None
    assert 'open_to_first_final_command_ns' not in owner.metrics[-1]
    arm = boundary.arms[0]
    boundary.program(message({arm: boundary.positions[arm]+.4}), 'arms')
    assert 'target too far' in boundary.last_rejection
    assert bridge._return_hold is not None

    boundary.positions[arm] += .025
    boundary.program(message({arm: boundary.positions[arm]+.020}), 'arms')
    assert bridge._return_hold is None
    assert 'open_to_first_final_command_ns' in owner.metrics[-1]
    _, first = rig.wire[-1]
    assert first.position[first.name.index(arm)] == pytest.approx(anchor[arm]+.0025)
    count = len(rig.wire)
    clock[0] += 30_000_000
    bridge._tick_return_hold()
    assert len(rig.wire) == count


@pytest.mark.parametrize('case', ['pause', 'regrip', 'timeout', 'input_lost', 'tracking', 'feedback_stale'])
def test_grip_return_hold_closes_on_stop_timeout_or_loss(case):
    rig = Rig()
    owner, bridge, boundary = rig.control, rig.bridge, rig.boundary
    clock = [owner.clock()]
    owner.clock = lambda: clock[0]
    rig.trigger(2, 0., grip=True)
    rig.advance()
    rig.advance()
    rig.trigger(3, 0.)
    bridge._tick_return_hold()
    count = len(rig.wire)
    clock[0] += 20_000_000
    if case == 'pause':
        owner.hold()
    elif case == 'regrip':
        rig.trigger(4, 0., grip=True)
    elif case == 'timeout':
        rig.advance()
        rig.advance()
        clock[0] = bridge._return_hold.deadline_ns
    elif case == 'input_lost':
        owner.input_lost('headset lost')
    elif case == 'tracking':
        boundary.positions[boundary.arms[0]] += .31
    else:
        boundary.sample_ns[boundary.arms[0]] -= 300_000_000
    bridge._tick_return_hold()
    assert len(rig.wire) == count
    assert bridge._return_hold is None
    if case in {'timeout', 'feedback_stale', 'tracking'}:
        assert owner.state == 'FAULT'


@pytest.mark.parametrize('held_trigger', [0., .5])
def test_grip_handback_seeds_right_hand_after_first_auto_arm_frame(held_trigger):
    rig = Rig()
    owner, boundary = rig.control, rig.boundary
    names = boundary.hands['right']
    closed = dict(zip(names, boundary.right_gripper['finger_closed']))
    opened = dict(zip(names, boundary.right_gripper['finger_open']))
    rig.trigger(2, 0.)
    rig.trigger(3, .8, grip=True)
    rig.advance()
    rig.advance()
    boundary.receive_owned(message(closed), owner.token)
    topic, last_manual = rig.wire[-1]
    assert topic == '/action/right_hand/joint_states'

    rig.trigger(4, held_trigger)
    rig.advance()
    rig.advance()
    arm = boundary.arms[0]
    boundary.program(message({arm: boundary.positions[arm]}), 'arms')
    assert rig.wire[-1][0] == '/action/joint_states'
    for name in names:
        boundary.received[name] = time.monotonic()-1
    if held_trigger:
        rig.trigger(5, held_trigger)
    else:
        boundary.program(message(opened), 'right')
    topic, first_auto = rig.wire[-1]
    assert topic == '/action/right_hand/joint_states'
    step = .01 * boundary.teleop['grippers']['max_velocity']
    assert max(abs(a-b) for a,b in zip(last_manual.position, first_auto.position)) <= step+1e-9
    assert any(abs(a-b) >= step-1e-9 for a,b in zip(last_manual.position, first_auto.position))


@pytest.mark.parametrize('case', ['valid', 'missing_feedback', 'revoked_during_prepare'])
def test_replay_legacy_arm_command(case):
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
    from rosidl_runtime_py.set_message import set_message_fields
    from rosidl_runtime_py.utilities import get_message

    rig = Rig()
    bridge, boundary = rig.bridge, rig.boundary
    node = boundary.node
    node.get_clock.return_value.now.side_effect = lambda: Time(nanoseconds=time.time_ns())
    rig.control.disable()
    rig.feed()
    bridge.set_command_output_enabled(True)
    assert bridge.begin_replay()
    arm = boundary.arms[0]
    target = message({arm: boundary.positions[arm]}, stamp=1_000_000_000)
    target.velocity = [1.0]
    topic = '/action/joint_states'
    assert bridge.publish_raw(topic, 'sensor_msgs/msg/JointState', serialize_message(target))
    if case == 'missing_feedback':
        boundary.positions.pop(arm)
    elif case == 'revoked_during_prepare':
        def stop_replay(*_):
            bridge.end_replay()
            return True, None, False
        boundary.collision.check.side_effect = stop_replay
    rig.wire.clear()
    before = time.time_ns()
    bridge._drain_commands(node, {}, get_message, set_message_fields)
    published = [msg for name, msg in rig.wire if name == topic]
    assert not bridge.status()['error']
    if case == 'valid':
        sent, = published
        assert boundary.stamp(sent) >= before
        assert sent.name == target.name and sent.position == target.position
        assert not sent.velocity  # The actual prepared target, not the raw MCAP payload.
    else:
        assert not published
        assert arm not in boundary.last_command_stamp


def test_boundary_logs_clips_rejections_and_never_admits_off_model_targets():
    r=Rig(); b=r.boundary
    b.program(message({n:b.positions[n]+.1 for n in b.arms}), 'arms')
    assert b.diagnostics['arm_velocity_clipped'] > 0
    assert b.last_clip['limit_rad_s'] == .25
    b.program(message({n:b.positions[n]+.4 for n in b.arms}), 'arms')
    assert 'target too far' in b.last_rejection
    assert 'measured=' in b.last_rejection and b.last_rejection_ns > 0
    side='right'
    b.program(message(dict(zip(b.hands[side],b.teleop['grippers'][side]['finger_open']))),side)
    assert b.last_rejection == ''
    assert 'target too far' in b.recent_rejection
    r.control.disable()
    count=len(r.wire)
    b.program(message({n:b.positions[n] for n in b.arms}), 'arms')
    assert len(r.wire)==count
    assert b.diagnostics['program_blocked_authority_arms']==1


def test_own_arms_and_hands_share_stamp_and_are_all_revoked_without_recording():
    r=Rig(); b=r.boundary
    ns=time.time_ns()
    arms={n:b.positions[n]+.01 for n in b.arms}
    b.program(message(arms,ns), 'arms')
    for side,names in b.hands.items():
        b.program(message({n:b.positions[n] for n in names},ns),side)
    assert {t for t,m in r.wire if t.startswith('/action/')} == {
        '/action/joint_states','/action/right_hand/joint_states','/action/left_hand/joint_states'}
    selected=[m for t,m in r.wire if t=='/action/joint_states'][-1]
    assert max(abs(q-b.positions[n]) for n,q in zip(selected.name,selected.position) if n in b.arms) <= .00251
    r.control.input(device_instance='vr', sequence=2, valid=True, gripped=True, stamp_ns=time.time_ns())
    r.advance();r.advance()
    assert r.control.state=='MANUAL'
    count=len(r.wire)
    b.program(message(arms),'arms')
    assert len(r.wire)==count
    r.control.open_segment.assert_not_called()
    assert not r.control.segment_open


def test_observation_is_not_relayed_and_supplied_waist_is_preserved():
    r=Rig();b=r.boundary
    assert b.feedback_fresh()
    assert not [topic for topic, _ in r.wire if topic.startswith('/hc_teleop/')]
    assert not hasattr(b, 'camera_pubs')
    b.program(message({n:b.positions[n] for n in reversed(b.feedback_order)}),'arms')
    target=[m for t,m in r.wire if t=='/action/joint_states'][-1]
    assert tuple(target.name)==b.arm_fields
    values = dict(zip(target.name, target.position))
    assert {n:values[n] for n in b.waist} == {n:b.positions[n] for n in b.waist}
    assert all(math.isnan(values[n]) for n in ('head_1', 'head_2', 'R_ban', 'L_ban'))


def test_gate_rechecks_revoke_during_collision_work_and_rejects_old_generation():
    r=Rig();b=r.boundary
    def takeover(*args):
        r.control.input(device_instance='vr', sequence=2, valid=True,
                        gripped=True, stamp_ns=time.time_ns())
        return True, None, False
    b.collision.check.side_effect=takeover
    before=len(r.wire)
    b.program(message({n:b.positions[n] for n in b.arms}), 'arms')
    assert len(r.wire)==before
    assert r.control.state=='TAKEOVER'
    old=r.control.token
    r.control.hold()
    b.receive_owned(message({n:b.positions[n] for n in b.arms}),old)
    assert len(r.wire)==before


@pytest.mark.parametrize('failure',['stale','nan','unknown','jump','collision'])
def test_invalid_or_discontinuous_program_never_reaches_output(failure):
    r=Rig();b=r.boundary
    values={n:b.positions[n] for n in b.arms}
    ns=time.time_ns()
    if failure=='stale':ns-=1_000_000_000
    if failure=='nan':values[b.arms[0]]=float('nan')
    if failure=='unknown':values['unexpected_axis']=0.
    if failure=='jump':values[b.arms[0]]+=.4
    if failure=='collision':b.collision.check.return_value=(False,('arm','body'),False)
    before=len(r.wire)
    b.program(message(values,ns),'arms')
    assert len(r.wire)==before


def test_hands_use_open_loop_targets_and_keep_command_rate_limit():
    r=Rig();b=r.boundary
    for name in b.hand_names:
        b.positions.pop(name, None)
        b.received.pop(name, None)
    names=b.hands['right']
    closed=dict(zip(names,b.teleop['grippers']['right']['finger_closed']))
    b.program(message(closed),'right')
    sent=[m for t,m in r.wire if t=='/action/right_hand/joint_states']
    assert len(sent)==1
    assert list(sent[0].position)==list(closed.values())
    assert not b.status()['hands_ready']['right']
    opened=dict(zip(names,b.teleop['grippers']['right']['finger_open']))
    b.program(message(opened),'right')
    sent=[m for t,m in r.wire if t=='/action/right_hand/joint_states']
    assert len(sent)==2
    assert max(abs(a-z) for a,z in zip(sent[0].position,sent[1].position)) <= b.teleop['grippers']['max_velocity']*.05+1e-9


@pytest.mark.parametrize('failure',['stale','nan','limit','unknown','hold','arm_feedback'])
def test_open_loop_hands_preserve_command_and_arm_safety_checks(failure):
    r=Rig();b=r.boundary
    values={n:b.positions[n] for n in b.hands['right']}
    ns=time.time_ns()
    if failure=='stale':ns-=1_000_000_000
    if failure=='nan':values[b.hands['right'][0]]=float('nan')
    if failure=='limit':values[b.hands['right'][0]]=100.
    if failure=='unknown':values['unknown_finger']=0.
    if failure=='hold':r.control.hold()
    channel='right'
    if failure=='arm_feedback':
        values={n:b.positions[n] for n in b.arms}
        b.received[b.arms[0]]=time.monotonic()-1
        channel='arms'
    before=len(r.wire)
    b.program(message(values,ns),channel)
    assert len(r.wire)==before


def test_open_loop_hand_commit_rechecks_revocation():
    r=Rig();b=r.boundary
    prepared=b.prepare(message({n:b.positions[n] for n in b.hands['right']}))
    assert prepared is not None
    r.control.hold()
    assert not b.commit(prepared)


def test_v23_revoke_ack_waits_for_measured_reset_and_old_pose_is_rejected():
    node=Mock()
    node.controller.free_joint_names=['axis']
    node.get_clock.return_value.now.side_effect=lambda: SimpleNamespace(nanoseconds=time.time_ns())
    solver=V23Intervention(node)
    control=ControlOwnership(recording_enabled=False, required_peers=('solver','teleop'))
    for peer in control.required_peers:control.peer(peer,peer)
    control.enable()
    control.start_auto(recording_ready=False, control_ready=True)
    token=control.token
    assert token.source=='motion' and not control.input_valid
    def permit(phase):
        msg=Header(frame_id='#'.join((token.key(),phase,solver.instance,'teleop')))
        msg.stamp=message({'axis':0.}).header.stamp
        solver.permit(msg)
    permit('revoke')
    assert not solver.ready
    node.controller.reset_to_feedback.assert_not_called()
    solver.feedback(message({'axis':0.}))
    assert solver.ready
    node.controller.reset_to_feedback.assert_called_once()
    permit('grant')
    assert not solver.ready
    assert not solver.authorized()
    solver.feedback(message({'axis':0.}))
    assert solver.ready
    assert not solver.authorized()  # Program grant never enables VR IK.


class NoRecorderApiTest(unittest.IsolatedAsyncioTestCase):
    async def test_session_and_return_without_recorder_or_capture(self):
        control=ControlOwnership(recording_enabled=False,required_peers=('solver','teleop'))
        for peer in control.required_peers:control.peer(peer,peer)
        runtime=SimpleNamespace(ros=SimpleNamespace(ownership=control,control_ready=lambda:True),
            recorder=None, player=None, session_profile_metadata={})
        runtime.ros.recover_intervention=lambda:None
        app=web.Application();register_intervention_routes(app,runtime)
        async with TestClient(TestServer(app)) as client:
            for action in ('enable','return','disable'):
                assert not control.input_valid
                state=control.snapshot()
                response=await client.post('/api/control/intervention/'+action,json=dict(
                    request_id=action,session_id=state['session_id'],expected_epoch=state['epoch']))
                assert response.status==202,await response.text()
                payload=await response.json()
                assert payload['capture'] is None
                assert payload['ownership']['recording_enabled'] is False


def test_feedback_timing_identifies_stale_axis_without_publishing():
    rig=Rig(); boundary=rig.boundary
    name=boundary.feedback_order[0]
    boundary.received[name]-=1.
    boundary.sample_ns[name]-=1_000_000_000
    output_count=len(rig.wire)
    status=boundary.status()
    assert not status['feedback_ready']
    assert status['feedback_timing_ms'][name]['receive_age_ms']>=1000
    assert status['feedback_timing_ms'][name]['source_age_ms']>=1000
    missing=boundary.feedback_order[-1]
    del boundary.positions[missing]
    assert boundary.status()['feedback_timing_ms'][missing]=={'missing':True}
    assert len(rig.wire)==output_count


def test_input_callback_error_and_feedback_failure_are_retained():
    rig=Rig(); bridge=rig.bridge
    node=rig.boundary.node
    node.get_clock.return_value.now.side_effect=lambda: SimpleNamespace(
        nanoseconds=time.time_ns(),to_msg=lambda:Header().stamp)
    callbacks={}
    def subscribe(_kind,topic,callback,*_args,**_kwargs):
        callbacks[topic]=callback
        return Mock()
    node.create_subscription.side_effect=subscribe
    bridge._ownership_callback_group=None
    bridge._setup_ownership_ros(node,{},Mock(),Mock(),[])
    callback=callbacks['/events/teleop/clutch']
    callback(SimpleNamespace(data='{}'))
    assert bridge.status()['last_input_callback_error']['error'].startswith('KeyError:')
    name=rig.boundary.feedback_order[0]
    rig.boundary.sample_ns[name]-=1_000_000_000
    callback(SimpleNamespace(data=json.dumps(dict(
        device_instance='vr',sequence=2,valid=True,gripped=False,
        stamp_ns=time.time_ns()))))
    assert bridge.ownership.state=='FAULT'
    failure=bridge.status()['last_feedback_failure']
    assert failure['feedback_timing_ms'][name]['source_age_ms']>=1000
    rig.feed()
    assert bridge.status()['last_feedback_failure']==failure


def test_callback_timing_preserves_return_and_exception():
    bridge=Rig().bridge
    with patch('middleware.core.ros_bridge.time.monotonic_ns', side_effect=[0,25_000_000]):
        assert bridge._timed_callback('probe',lambda value:value+1)(4)==5
    snapshot=bridge.status()
    assert snapshot['callback_timings']['probe']==dict(count=1,max_ms=25.,last_ms=25.)
    assert snapshot['slow_callbacks'][-1]['name']=='probe'
    def fail():raise ValueError('unchanged exception')
    with pytest.raises(ValueError,match='unchanged exception'):
        bridge._timed_callback('probe',fail)()
    assert snapshot['callback_timings']['probe']['count']==1
    assert bridge.status()['callback_timings']['probe']['count']==2


def test_input_delivery_timing_identifies_age_at_callback_entry():
    rig=Rig();bridge=rig.bridge
    source=time.time_ns()
    assert bridge.receive_intervention_input(dict(device_instance='vr',sequence=2,
        valid=True,gripped=False,stamp_ns=source),callback_entry_ns=source+5_000_000)
    diagnostic=bridge.ownership.snapshot()['input_diagnostics']
    assert diagnostic['callback_entry_source_ns']==source
    assert diagnostic['callback_entry_age_ms']==5.
    assert diagnostic['callback_lock_wait_ms']>=0
    assert diagnostic['callback_input_work_ms']>=0


def test_lock_owner_diagnostic_does_not_acquire_or_replace_control_lock():
    import threading
    bridge=Rig().bridge
    lock=bridge._lock
    assert bridge._sample_control_contention() is None
    bridge.ownership.input_diagnostics.update(last_age_ms=500.,last_callback_ns=time.time_ns())
    held=threading.Event(); release=threading.Event()
    def hold_lock():
        with lock:
            held.set();release.wait(2.)
    owner=threading.Thread(target=hold_lock,name='diagnostic-test-owner')
    owner.start();assert held.wait(1.)
    started=time.monotonic()
    try:
        sample=bridge._sample_control_contention()
        assert time.monotonic()-started<.5
        assert sample['owner_thread']=='diagnostic-test-owner'
        assert sample['owner_stack'] and sample['input_age_ms']==500.
    finally:
        release.set();owner.join(timeout=1.)
    assert bridge._lock is lock
    assert bridge._sample_control_contention()['owner_thread'] is None


def test_measured_feedback_uses_control_node_and_commands_stay_on_command_node():
    rig = Rig()
    command_node = rig.boundary.node
    command_node.create_subscription.reset_mock()
    feedback_node = Mock()
    rig.bridge._ownership_callback_group = object()
    with patch('adapters.core.collision_safety.ModelCollisionGuard'):
        boundary = LegacyTopicControl(command_node, rig.bridge, Mock(),
                                      feedback_node=feedback_node)
    feedback_topics = {call.args[1] for call in feedback_node.create_subscription.call_args_list}
    command_topics = {call.args[1] for call in command_node.create_subscription.call_args_list}
    assert feedback_topics == {'/observation/joint_states', '/observation/hands/joint_states'}
    assert '/hc_teleop/joint_cmd' in command_topics
    assert not feedback_topics & command_topics
    for call in feedback_node.create_subscription.call_args_list:
        assert call.kwargs['callback_group'] is rig.bridge._ownership_callback_group
    assert boundary.bridge._lock is rig.bridge.ownership.lock


def test_grip_revokes_while_command_preparation_is_blocked():
    import threading
    rig = Rig()
    entered, release = threading.Event(), threading.Event()
    def collision_check(*args):
        entered.set()
        assert release.wait(2.)
        return True, None, False
    rig.boundary.collision.check.side_effect = collision_check
    outputs = len(rig.wire)
    failures = []
    def prepare_command():
        try:
            rig.boundary.program(message({n:rig.boundary.positions[n]
                                          for n in rig.boundary.arms}), 'arms')
        except BaseException as exc:
            failures.append(exc)
    worker = threading.Thread(target=prepare_command)
    worker.start()
    try:
        assert entered.wait(1.)
        assert rig.bridge.receive_intervention_input(dict(device_instance='vr', sequence=2,
            valid=True, gripped=True, stamp_ns=time.time_ns()))
        assert rig.control.state == 'TAKEOVER'
        assert worker.is_alive()
    finally:
        release.set()
        worker.join(timeout=2.)
    assert not worker.is_alive() and not failures
    assert len(rig.wire) == outputs


def test_standard_gripper_fields_survive_action_boundary():
    r = Rig(); b = r.boundary
    r.control.disable()
    r.feed()
    values = {n: b.positions[n] for n in b.arms}
    values.update({n: 0.5 for n in b.sim_joints})
    prepared = b.prepare(message(values))
    assert prepared is not None, b.last_rejection
    assert prepared[0].name == list(values)
    assert list(prepared[0].position) == list(values.values())
    assert b.prepare(message({**values, 'unexpected_joint': 0.0})) is None
    assert b.prepare(message({n: 0.5 for n in b.sim_joints})) is None


def test_normal_vr_reaches_action_topic_without_dagger_session():
    r = Rig(); b = r.boundary
    r.control.disable()
    r.bridge._command_source = 'vr'
    r.bridge.set_command_output_enabled(True)
    r.feed()
    before = len(r.wire)
    values = {n: b.positions[n] for n in b.arms}
    values.update({n: 0.5 for n in b.sim_joints})
    assert r.bridge.forward_live_command('vr', message(values), b.output.publish)
    assert len(r.wire) == before + 1
    topic, target = r.wire[-1]
    assert topic == '/action/joint_states'
    assert set(target.name) == set(b.arms) | b.sim_joints
    r.bridge.set_command_output_enabled(False)
    assert not r.bridge.forward_live_command('vr', message(values), b.output.publish)
    assert len(r.wire) == before + 1


def test_empty_feedback_requirement_does_not_read_ros_clock():
    b = Rig().boundary
    clock = b.node.get_clock.return_value.now
    clock.reset_mock()
    assert b.fresh(set())
    assert clock.call_count == 0


def test_each_boundary_phase_reads_one_fresh_ros_clock():
    r = Rig(); b = r.boundary
    r.control.disable()
    b.collision = None
    r.feed()
    msg = message({n: b.positions[n] for n in b.arms})
    clock = b.node.get_clock.return_value.now
    clock.reset_mock()
    prepared = b.prepare(msg)
    assert prepared is not None, b.last_rejection
    assert clock.call_count == 1
    clock.reset_mock()
    assert b.commit(prepared)
    assert clock.call_count == 1


def test_final_commit_uses_new_clock_and_rejects_elapsed_command():
    r = Rig(); b = r.boundary
    r.control.disable()
    b.collision = None
    r.feed()
    msg = message({n: b.positions[n] for n in b.arms})
    prepared = b.prepare(msg)
    assert prepared is not None
    b.node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(
        nanoseconds=b.stamp(msg)+300_000_000)
    assert not b.commit(prepared)
    assert b.last_rejection == 'commit freshness or authority changed'


def test_grip_takeover_keeps_a_fixed_arm_target_until_the_first_manual_arm_send():
    rig = Rig()
    owner, bridge, boundary = rig.control, rig.bridge, rig.boundary
    clock = [owner.clock()]
    owner.clock = lambda: clock[0]
    evidences = []
    bridge._action_sink = evidences.append

    rig.trigger(2, 0., grip=True)
    assert owner.state == 'TAKEOVER' and not bridge._command_output_enabled
    assert bridge._takeover_hold is not None
    anchor = dict(bridge._takeover_hold.positions)
    bridge._tick_takeover_hold()
    tag = 'hc-dagger/takeover-hold'
    assert rig.wire[-1][1].header.frame_id == tag
    assert any(e['kind'] == 'grip_takeover_hold_open' for e in owner.events)
    assert not evidences

    rig.advance()
    assert owner.state == 'TAKEOVER' and owner.phase == 'grant'
    clock[0] += 20_000_000
    rig.trigger(3, 0., grip=True)
    for peer in owner.required_peers:
        owner.peer(peer, peer)
    bridge._tick_takeover_hold()
    assert rig.wire[-1][1].header.frame_id == tag

    rig.advance()
    assert owner.state == 'MANUAL'
    for sequence in range(4, 15):
        clock[0] += 20_000_000
        rig.trigger(sequence, 0., grip=True)
        for peer in owner.required_peers:
            owner.peer(peer, peer)
        bridge._tick_takeover_hold()
        assert rig.wire[-1][1].header.frame_id == tag
        names, values = rig.wire[-1][1].name, rig.wire[-1][1].position
        assert {n: q for n, q in zip(names, values) if n in anchor} == anchor

    right = {name: boundary.positions[name] for name in boundary.hands['right']}
    boundary.receive_owned(message(right), owner.token)
    assert bridge._takeover_hold is not None
    assert len(evidences) == 1 and evidences[0]['command_seq'] == 1

    arm = boundary.arms[0]
    boundary.receive_owned(message({arm: boundary.positions[arm] + .02}), owner.token)
    assert bridge._takeover_hold is None
    assert any(e['kind'] == 'grip_takeover_hold_closed' for e in owner.events)
    assert len(evidences) == 2 and evidences[-1]['command_seq'] == 2
    count = len(rig.wire)
    clock[0] += 30_000_000
    bridge._tick_takeover_hold()
    assert len(rig.wire) == count


@pytest.mark.parametrize(
    'case',
    ['release', 'pause', 'input_lost', 'tracking', 'feedback_stale', 'manual_deadline'],
)
def test_grip_takeover_hold_closes_on_release_pause_loss_or_timeout(case):
    rig = Rig()
    owner, bridge, boundary = rig.control, rig.bridge, rig.boundary
    clock = [owner.clock()]
    owner.clock = lambda: clock[0]

    rig.trigger(2, 0., grip=True)
    bridge._tick_takeover_hold()
    count = len(rig.wire)
    clock[0] += 20_000_000
    if case == 'release':
        rig.trigger(3, 0., grip=False)
    elif case == 'pause':
        owner.hold()
    elif case == 'input_lost':
        owner.input_lost('headset lost')
    elif case == 'tracking':
        boundary.positions[boundary.arms[0]] += .31
    elif case == 'feedback_stale':
        boundary.sample_ns[boundary.arms[0]] -= 300_000_000
    else:
        rig.advance()
        rig.advance()
        assert owner.state == 'MANUAL'
        clock[0] = bridge._takeover_hold.deadline_ns

    bridge._tick_takeover_hold()
    assert len(rig.wire) == count
    assert bridge._takeover_hold is None
    if case == 'release':
        assert owner.state == 'RETURNING' and bridge._return_hold is not None
        bridge._tick_return_hold()
        assert len(rig.wire) == count + 1
        assert rig.wire[-1][1].header.frame_id == 'hc-dagger/return-hold'
    elif case == 'manual_deadline':
        assert owner.state == 'MANUAL'
        assert any(e['kind'] == 'grip_takeover_hold_closed' for e in owner.events)
    elif case in {'tracking', 'feedback_stale', 'input_lost'}:
        assert owner.state == 'FAULT'
