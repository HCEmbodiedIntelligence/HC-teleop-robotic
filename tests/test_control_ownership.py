"""Control facts, transaction races and final-boundary isolation without ROS."""
import threading
from types import SimpleNamespace as Obj

import pytest

from middleware.core.control_ownership import ControlOwnership, OwnershipConflict
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
    bridge.set_command_output_enabled(True)
    bridge._ros_now = lambda: 100.11
    entered, finish = threading.Event(), threading.Event()
    committed = []
    def prepare(message):
        entered.set()
        assert finish.wait(2)
        return message
    bridge._boundary = Obj(prepare=prepare, commit=lambda m: committed.append(m) or True)
    message = Obj(name=['axis'], position=[.1], header=Obj(stamp=Obj(sec=100, nanosec=110_000_000)))
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
    bridge._command_output_enabled = True
    bridge.ownership=rig.control
    evidence=[]
    bridge._action_sink=evidence.append
    message=Obj(name=['axis'],position=[.1],header=Obj(stamp=Obj(sec=100,nanosec=100_000_123)))
    def publish(_message): now[0]=100.12
    assert bridge.forward_live_command('vr',message,publish,token=rig.control.token)
    assert evidence[0]['command_stamp_ns']==100_100_000_123
    assert evidence[0]['stamp_ns']==100_120_000_000
