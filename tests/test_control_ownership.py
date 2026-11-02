"""Control facts, transaction races and final-boundary isolation without ROS."""
import threading
from pathlib import Path
from types import SimpleNamespace as Obj
from unittest.mock import Mock

import pytest

from middleware.core.control_ownership import ControlOwnership, OwnershipConflict
from middleware.core.app import MiddlewareRuntime
from middleware.core.ros_bridge import RosBridge


class Rig:
    def __init__(self):
        self.ns = 1_000_000_000
        self.seq = 0
        self.effects = []
        self.control = ControlOwnership(clock=lambda: self.ns, ros_clock=lambda: self.ns,
            revoke=lambda: self.effects.append('revoke'),
            permit=lambda *args: self.effects.append(('permit', args[1], args[0])),
            open_segment=lambda *args: self.effects.append(('open', args)),
            close_segment=lambda *args: self.effects.append(('close', args)))
        for peer in ('motion', 'teleop'):
            self.control.peer(peer, peer+'-instance')
        self.control.enable()
        self.input(False)

    def input(self, gripped, valid=True):
        self.seq += 1
        self.ns += 1_000_000
        self.control.input(device_instance='vr-test', sequence=self.seq,
                           stamp_ns=self.ns, valid=valid, gripped=gripped)

    def ack(self):
        c = self.control
        for peer in ('motion', 'teleop'):
            c.acknowledge(c.token, c.phase, peer, peer+'-instance')
        c.advance(control_ready=True, manual_ready=True)

    def auto(self):
        self.control.start_auto(recording_ready=True, control_ready=True)
        self.ack()
        self.ack()
        assert self.control.state == 'AUTO'


def test_model_only_then_dagger_toggle_preserves_auto_and_fences_manual():
    rig = Rig()
    c = rig.control
    c.disable()
    c.start_model(control_ready=True)
    rig.ack(); rig.ack()
    assert c.state == 'AUTO' and not c.dagger_enabled
    token = c.token
    rig.input(False); rig.input(True)
    assert c.token == token and not c.takeover()
    c.set_dagger(True)
    assert c.token == token and c.allows(token)
    rig.input(True)
    assert c.state == 'AUTO'  # held before enabling cannot seize control
    rig.input(False); rig.input(True)
    rig.ack(); rig.ack()
    assert c.state == 'MANUAL'
    manual = c.token
    c.set_dagger(False)
    assert c.state == 'RETURNING' and not c.allows(manual)
    rig.ack(); rig.ack()
    assert c.state == 'AUTO' and not c.dagger_enabled
    token = c.token
    c.set_dagger(True); c.set_dagger(False)
    assert c.token == token and c.allows(token)


def test_dagger_disable_preserves_fault_and_model_ignores_controller_buttons():
    rig = Rig(); c = rig.control
    c.disable(); c.start_model(control_ready=True)
    rig.ack(); rig.ack()
    rig.ns += 1
    c.input(device_instance='vr-test', sequence=100, valid=True, gripped=True,
            pause_requested=True, resume_requested=True, stamp_ns=rig.ns)
    assert c.state == 'AUTO'
    c.set_dagger(True)
    c.fault('physical stop', estop=True)
    c.set_dagger(False)
    assert c.state == 'ESTOP'
    with pytest.raises(OwnershipConflict):
        c.start_model(control_ready=True)


def test_manual_input_gap_revokes_and_reports_elapsed_time():
    rig=Rig();rig.auto();rig.input(True);rig.ack();rig.ack()
    control=rig.control;token=control.token
    assert control.state=='MANUAL'
    rig.ns+=325_000_000
    control.watchdog()
    assert control.state=='FAULT' and not control.allows(token)
    events=[e for e in control.events if e['kind']=='manual_input_timeout']
    assert len(events)==1 and events[0]['input_gap_ms']==325
    assert events[0]['limit_ms']==300
    assert events[0]['last_accepted_stamp_ns']==rig.ns-325_000_000


def test_short_repeated_interventions_and_ordered_capture_boundaries():
    rig = Rig()
    rig.auto()
    c = rig.control
    identities = set()
    for _ in range(100):
        previous = c.token
        rig.input(True)
        assert rig.effects[-2] == 'revoke'
        assert not c.allows(previous)
        rig.ack()
        assert rig.effects[-2][0] == 'open'  # before grant is sent
        rig.ack()
        assert c.state == 'MANUAL'
        identities.add(c.token.intervention_id)
        manual = c.token
        for _ in range(10):
            rig.ns += 10_000_000
            evidence = c.sent(manual, Obj(name=['axis'], position=[.1]), rig.ns)
            assert evidence['intervention'] and evidence['action_boundary'] == 'hc_final'
        rig.input(False)
        assert c.state == 'RETURNING' and not c.allows(manual)
        close = next(e for e in reversed(rig.effects) if isinstance(e, tuple) and e[0] == 'close')
        assert close[1][2:] == (10, 'normal release')
        rig.ack()
        rig.ack()
        assert c.state == 'AUTO'
    assert len(identities) == 100
    assert len(c.metrics) == 100
    assert len([e for e in rig.effects if isinstance(e, tuple) and e[0] == 'close']) == 100


def test_release_during_takeover_does_not_open_empty_episode():
    rig = Rig()
    rig.auto()
    rig.input(True)
    stale = rig.control.token
    rig.input(False)
    assert rig.control.state == 'RETURNING'
    assert not rig.control.acknowledge(stale, 'revoke', 'motion', 'motion-instance')
    rig.ack()
    rig.ack()
    assert rig.control.state == 'AUTO'
    assert not any(isinstance(e, tuple) and e[0] == 'open' for e in rig.effects)


def test_regrip_while_returning_starts_a_new_identity():
    rig = Rig()
    rig.auto()
    rig.input(True)
    rig.ack()
    rig.ack()
    first = rig.control.token
    rig.input(False)
    returning = rig.control.token
    rig.input(True)
    rig.ack()
    rig.ack()
    assert rig.control.state == 'MANUAL'
    assert rig.control.token.intervention_id != first.intervention_id
    assert not rig.control.acknowledge(returning, 'grant', 'motion', 'motion-instance')
    assert not rig.control.allows(first)


@pytest.mark.parametrize('interruption', ['hold', 'fault', 'estop', 'invalid', 'timeout', 'restart'])
def test_release_never_clears_pause_or_fault(interruption):
    rig = Rig()
    rig.auto()
    rig.input(True)
    rig.ack()
    rig.ack()
    c = rig.control
    if interruption == 'hold':
        c.hold()
    elif interruption == 'estop':
        c.fault('estop', estop=True)
    elif interruption == 'invalid':
        rig.input(False, valid=False)
    elif interruption == 'timeout':
        rig.ns += 400_000_000
        c.watchdog()
    elif interruption == 'restart':
        c.peer('motion', 'new-instance')
    else:
        c.fault('feedback lost')
    frozen = c.state
    rig.input(False)
    rig.ack()
    assert c.state == frozen
    assert c.state in {'HOLD', 'FAULT', 'ESTOP'}


def test_hold_takeover_returns_to_hold_and_capture_failure_does_not_block_release():
    rig = Rig()
    def unavailable(*args):
        raise RuntimeError('disk failed')
    rig.control.open_segment = unavailable
    rig.input(True)
    rig.ack()
    rig.ack()
    assert rig.control.state == 'MANUAL'
    assert rig.control.recording_error == 'disk failed'
    rig.input(False)
    assert rig.control.state == 'HOLD'


def test_recording_failure_does_not_delay_auto_return():
    rig = Rig()
    rig.auto()
    rig.control.close_segment = lambda *args: (_ for _ in ()).throw(RuntimeError('writer failed'))
    rig.input(True)
    rig.ack()
    rig.ack()
    rig.input(False)
    rig.ack()
    rig.ack()
    assert rig.control.state == 'AUTO'
    assert rig.control.recording_error == 'writer failed'


def test_pause_wins_over_same_frame_grip_and_requires_fresh_release():
    rig = Rig()
    rig.auto()
    c = rig.control
    rig.seq += 1
    rig.ns += 1_000_000
    c.input(device_instance='vr-test', sequence=rig.seq, stamp_ns=rig.ns,
            valid=True, gripped=True, pause_requested=True, resume_requested=True)
    paused_epoch = c.epoch
    assert c.state == 'HOLD' and not c.input_armed and not c.resume_auto
    rig.input(True)
    assert c.state == 'HOLD' and c.epoch == paused_epoch
    rig.input(False)
    rig.input(True)
    rig.ack()
    rig.ack()
    assert c.state == 'MANUAL'
    rig.input(False)
    assert c.state == 'HOLD'


def test_duplicate_or_stale_pause_and_resume_do_not_change_authority():
    rig = Rig()
    rig.auto()
    c = rig.control
    sequence, stamp = rig.seq, rig.ns
    for extra in ({'pause_requested': True}, {'resume_requested': True}):
        assert not c.input(device_instance='vr-test', sequence=sequence, stamp_ns=stamp,
                           valid=True, gripped=False, **extra)
        assert not c.input(device_instance='vr-test', sequence=sequence+10,
                           stamp_ns=stamp-1_000_000_000, valid=True, gripped=False, **extra)
        assert c.state == 'AUTO'


def test_a_in_auto_does_not_suppress_manual_grip():
    rig = Rig()
    rig.auto()
    rig.ns += 1_000_000
    rig.control.input(device_instance='vr-test', sequence=rig.seq+1, stamp_ns=rig.ns,
                      valid=True, gripped=True, resume_requested=True)
    assert rig.control.state == 'TAKEOVER'


def test_snapshot_freshness_does_not_change_control_version():
    rig = Rig()
    first = rig.control.snapshot()
    rig.ns += 50_000_000
    second = rig.control.snapshot()
    assert second['published_stamp_ns'] > first['published_stamp_ns']
    assert second['sequence'] == first['sequence']
    assert second['epoch'] == first['epoch']


def test_new_input_instance_cannot_recover_its_own_fault_in_same_sample():
    rig = Rig()
    rig.auto()
    rig.input(True)
    rig.ack()
    rig.ack()
    rig.ns += 1_000_000
    accepted = rig.control.input(device_instance='restarted-vr', sequence=0,
        stamp_ns=rig.ns, valid=True, gripped=False, resume_requested=True)
    assert not accepted
    assert rig.control.state == 'FAULT' and not rig.control.input_armed


def test_explicit_auto_start_without_headset_still_requires_hardware_and_peers():
    rig = Rig()
    c = rig.control
    c.input_lost('headset absent')
    with pytest.raises(OwnershipConflict):
        c.start_auto(recording_ready=True, control_ready=False)
    teleop = c.peers.pop('teleop')
    with pytest.raises(OwnershipConflict):
        c.start_auto(recording_ready=True, control_ready=True)
    c.peer('teleop', teleop)
    rig.auto()
    assert not c.input_valid and not c.input_armed
    assert c.allows(c.token)
    c.fault('physical emergency stop', estop=True)
    assert not c.allows(c.token)
    with pytest.raises(OwnershipConflict):
        c.start_auto(recording_ready=True, control_ready=True)


@pytest.mark.parametrize('loss', ['timeout', 'invalid', 'restart'])
@pytest.mark.parametrize('state', ['HOLD', 'AUTO', 'RETURNING'])
def test_program_authority_survives_headset_loss_and_requires_new_release(loss, state):
    rig = Rig()
    c = rig.control
    if state == 'AUTO':
        rig.auto()
    elif state == 'RETURNING':
        c.start_auto(recording_ready=True, control_ready=True)
    token = c.token
    device = 'vr-test'
    if loss == 'timeout':
        rig.ns += 301_000_000
        for peer in c.required_peers:
            c.peer(peer, peer+'-instance')
        c.watchdog()
    elif loss == 'invalid':
        rig.input(False, valid=False)
    else:
        device = 'new-device'
        rig.ns += 1_000_000
        c.input(device_instance=device, sequence=1, stamp_ns=rig.ns, valid=True, gripped=True)
    assert c.state == state and c.token == token and not c.input_armed
    for grip, expected in [(True, state), (False, state), (True, 'TAKEOVER')]:
        rig.seq += 1
        rig.ns += 1_000_000
        c.input(device_instance=device, sequence=rig.seq+10, stamp_ns=rig.ns,
                valid=True, gripped=grip)
        assert c.state == expected


def test_first_held_packet_after_silent_gap_cannot_reuse_old_release():
    rig = Rig()
    rig.auto()
    rig.ns += 301_000_000
    rig.input(True)  # arrives before the periodic watchdog
    assert rig.control.state == 'AUTO' and not rig.control.input_armed
    rig.input(False)
    rig.input(True)
    assert rig.control.state == 'TAKEOVER'


def test_release_queued_before_timeout_cannot_rearm_manual_input():
    rig = Rig()
    rig.auto()
    queued_stamp = rig.ns
    rig.ns += 10_000_000
    rig.control.input_lost('transport disconnected')
    assert not rig.control.input(device_instance='vr-test', sequence=rig.seq+1,
        stamp_ns=queued_stamp, valid=True, gripped=False)
    rig.seq += 1
    rig.input(True)
    assert rig.control.state == 'AUTO' and not rig.control.input_armed


def test_expired_manual_input_cannot_complete_grant_or_publish():
    rig = Rig()
    rig.auto()
    rig.input(True)
    rig.ack()
    assert rig.control.phase == 'grant'
    token = rig.control.token
    rig.ns += 301_000_000
    rig.ack()
    assert rig.control.state == 'FAULT'
    assert not rig.control.allows(token)


def test_auto_still_faults_on_required_process_heartbeat_loss():
    rig = Rig()
    rig.auto()
    rig.ns += 301_000_000
    rig.control.watchdog()
    assert rig.control.state == 'FAULT'
    assert 'heartbeat lost' in rig.control.reason


@pytest.mark.parametrize('state', ['HOLD', 'AUTO', 'TAKEOVER', 'MANUAL'])
def test_runtime_vr_timeout_callback_respects_current_owner(state):
    rig = Rig()
    c = rig.control
    if state != 'HOLD':
        rig.auto()
    if state in {'TAKEOVER', 'MANUAL'}:
        rig.input(True)
    if state == 'MANUAL':
        rig.ack()
        rig.ack()
    runtime = MiddlewareRuntime({}, Path('.'))
    runtime.ros = Obj(ownership=c)
    runtime._on_safety_event = Mock()
    runtime._on_vr_timeout('head tracking invalid')
    assert c.state == ('FAULT' if state in {'TAKEOVER', 'MANUAL'} else state)
    assert not c.input_valid and not c.input_armed
    runtime._on_safety_event.assert_not_called()


@pytest.mark.parametrize('source', ['vr', 'motion'])
def test_ordinary_control_vr_timeout_behavior_is_preserved(source):
    runtime = MiddlewareRuntime({'safety': {'enabled': True, 'stop_topic': '/stop'}}, Path('.'))
    runtime.ros = Obj(ownership=None, status=lambda: {'command_mux': {'source': source}},
                      emergency_stop=Mock(), publish=Mock())
    runtime.emit = Mock()
    runtime._on_vr_timeout('headset disconnected')
    if source == 'vr':
        runtime.ros.emergency_stop.assert_called_once_with('/stop', 'headset disconnected')
    else:
        runtime.ros.emergency_stop.assert_not_called()
        runtime.ros.publish.assert_called_once_with('/control/teleop/enabled', 'std_msgs/msg/Bool', {'data': False})


def test_headless_recovery_requires_fresh_static_feedback_and_only_returns_hold():
    rig = Rig()
    c = rig.control
    c.input_lost('headset absent')
    c.fault('physical stop', estop=True)
    bridge = RosBridge({}, lambda *args: None)
    bridge.ownership, bridge._lock = c, c.lock
    bridge._boundary = Obj(handover_ready=Mock(return_value=False))
    bridge._enqueue = Mock()
    with pytest.raises(ValueError):
        bridge.recover_intervention()
    bridge._enqueue.assert_not_called()
    bridge._boundary.handover_ready.return_value = True
    bridge.recover_intervention()
    assert c.state == 'ESTOP'  # Hardware reset acknowledgement is still required.
    assert bridge._enqueue.call_args.args[0] == 'ownership_recover'
    c.recover(control_ready=True)
    assert c.state == 'HOLD' and not c.resume_auto and not c.allows(c.token)


def test_semantic_resume_returns_from_hold_but_only_recovers_a_fault():
    from unittest.mock import Mock
    rig = Rig()
    c = rig.control
    bridge = RosBridge({}, lambda *args: None)
    bridge.ownership, bridge._lock = c, c.lock
    bridge.control_ready = lambda: True
    c.capture_ready = lambda: True
    bridge.recover_intervention = Mock(side_effect=lambda: c.recover(control_ready=True))

    def send(**buttons):
        rig.ns += 1_000_000
        rig.seq += 1
        return bridge.receive_intervention_input(dict(device_instance='vr-test',
            sequence=rig.seq, stamp_ns=rig.ns, valid=True, gripped=False, **buttons))

    send(resume_requested=True)
    assert c.state == 'RETURNING'
    rig.ack()
    rig.ack()
    send(pause_requested=True, resume_requested=True)
    assert c.state == 'HOLD'
    send()  # release after pause
    send(resume_requested=True)
    assert c.state == 'RETURNING'
    c.fault('test', estop=True)
    send(resume_requested=True)
    assert c.state == 'HOLD' and not c.input_armed
    bridge.recover_intervention.assert_called_once()
    send()
    send(resume_requested=True)
    assert c.state == 'RETURNING'


def test_explicit_start_readiness_stale_requests_and_timeout():
    rig = Rig()
    c = rig.control
    with pytest.raises(OwnershipConflict):
        c.start_auto(recording_ready=False, control_ready=True)
    c.request('start', c.session, c.epoch,
              lambda: c.start_auto(recording_ready=True, control_ready=True))
    epoch = c.epoch
    c.request('start', 'old-session', 0, lambda: pytest.fail('duplicate executed'))
    assert c.epoch == epoch
    with pytest.raises(OwnershipConflict):
        c.request('late', c.session, epoch-1, c.hold)
    rig.ns += 2_100_000_000
    c.advance(control_ready=True, manual_ready=True)
    assert c.state == 'FAULT' and 'timeout' in c.reason


def test_delayed_collision_result_is_rejected_after_revoke_and_reenable():
    bridge = RosBridge({'command_mux': {'source': 'motion'}}, lambda *args: None)
    bridge._ros_now = lambda: 100.1
    bridge.receive_hardware_ready(True)
    entered, finish = threading.Event(), threading.Event()
    committed = []
    def prepare(message):
        entered.set()
        assert finish.wait(2)
        return message
    bridge._boundary = Obj(prepare=prepare, commit=lambda m: committed.append(m) or True)
    message = Obj(name=['axis'], position=[.1], header=Obj(stamp=Obj(sec=100, nanosec=100_000_000)))
    result = []
    thread = threading.Thread(target=lambda: result.append(
        bridge.forward_live_command('motion', message, lambda m: committed.append(m))))
    thread.start()
    assert entered.wait(2)
    bridge.set_command_output_enabled(False)
    bridge.set_command_output_enabled(True)
    finish.set()
    thread.join(2)
    assert result == [False] and committed == []


def test_evidence_keeps_exact_command_time_separate_from_final_publication():
    rig=Rig(); rig.auto(); rig.input(True); rig.ack(); rig.ack()
    bridge=RosBridge({'command_mux': {'source': 'vr'}},lambda *args:None)
    now=[100.11]
    bridge._ros_now=lambda:now[0]
    bridge.receive_hardware_ready(True)
    bridge.ownership=rig.control
    evidence=[]
    bridge._action_sink=evidence.append
    message=Obj(name=['axis'],position=[.1],header=Obj(stamp=Obj(sec=100,nanosec=100_000_123)))
    def publish(_message): now[0]=100.12
    assert bridge.forward_live_command('vr',message,publish,token=rig.control.token)
    assert evidence[0]['command_stamp_ns']==100_100_000_123
    assert evidence[0]['stamp_ns']==100_120_000_000


def test_input_diagnostics_explain_rejection_without_arming_old_input():
    rig=Rig(); control=rig.control
    before=control.snapshot()['input_diagnostics']
    old=rig.ns-400_000_000
    assert not control.input(device_instance='vr-test',sequence=100,
                             stamp_ns=old,valid=True,gripped=True)
    diagnostic=control.snapshot()['input_diagnostics']
    assert diagnostic['last_rejection']=='sample_age_out_of_range'
    assert diagnostic['last_rejected_age_ms']==400.
    assert diagnostic['last_rejected_source_ns']==old
    assert diagnostic['rejected']==before['rejected']+1
    assert control.state=='HOLD' and not control.gripped
    assert before['rejected']==0  # Previously returned snapshots stay stable.
    rig.input(False)
    assert control.snapshot()['input_diagnostics']['accepted']==before['accepted']+1


def test_heartbeat_diagnostic_keeps_real_timeout_fault():
    rig=Rig(); rig.auto()
    rig.ns+=400_000_000
    rig.control.watchdog()
    assert rig.control.state=='FAULT'
    events=[e for e in rig.control.events if e['kind']=='heartbeat_timeout']
    assert len(events)==1
    assert events[0]['participant'] in ('teleop','motion')
    assert events[0]['gap_ms']>events[0]['limit_ms']==300.
