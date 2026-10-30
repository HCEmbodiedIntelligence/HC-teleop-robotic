import copy
import json
import sys
import types
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from adapters.core.intervention import ClutchInput, InterventionTeleop
from middleware.core.app import MiddlewareRuntime
from middleware.core.control_ownership import ControlOwnership, ControlToken
from middleware.core.ros_bridge import RosBridge


def packet(sequence, grip):
    pose = dict(position=[0., 0., 0.], quaternion=[0., 0., 0., 1.])
    return dict(protocol_version=3, sequence=sequence, vr_timestamp=sequence*.01,
                tracking=dict(head=True, left=True, right=True), poses=dict(left=pose, right=pose),
                inputs=dict(left=dict(grip=0.), right=dict(grip=grip)))


def test_missing_default_zero_and_tracking_loss_cannot_release():
    clutch = ClutchInput({})
    assert clutch.sample(packet(1, 0.)) == (True, False)
    assert clutch.sample(packet(2, 1.)) == (True, True)
    missing = packet(3, 0.)
    del missing['inputs']['right']['grip']
    assert clutch.sample(missing) == (False, True)
    lost = packet(4, 0.)
    lost['tracking']['left'] = False
    assert clutch.sample(lost) == (False, True)
    assert clutch.sample(packet(5, 0.)) == (True, False)
    assert clutch.sample(packet(6, 1.)) == (True, True)
    legacy = dict(packet(7, 0.), protocol_version=1)
    assert clutch.sample(legacy) == (False, True)


def test_hysteresis_reordered_packets_and_profile_mapping():
    clutch = ClutchInput({'takeover_clutch_side': 'left', 'clutch_axis': 1})
    frame = packet(1, 1.)
    assert clutch.sample(frame) == (True, False)
    frame = packet(2, 0.)
    frame['inputs']['left']['grip'] = .6
    assert clutch.sample(frame) == (True, True)
    delayed = copy.deepcopy(frame)
    delayed['inputs']['left']['grip'] = 0.
    assert clutch.sample(delayed) is None
    frame = packet(3, 0.)
    frame['inputs']['left']['grip'] = .52
    assert clutch.sample(frame) == (True, True)
    frame = packet(4, 0.)
    assert clutch.sample(frame) == (True, False)


def test_reordered_sequence_cannot_inject_a_button_sample():
    clutch = ClutchInput({})
    assert clutch.sample(packet(8, 0.)) == (True, False)
    delayed = packet(7, 1.)
    delayed['vr_timestamp'] = 999.
    assert clutch.sample(delayed) is None
    assert clutch.sample(packet(9, 0.)) == (True, False)


@pytest.mark.parametrize('head', [False, None, 1, 'true', 'missing'])
def test_manual_input_requires_explicit_valid_head_tracking(head):
    clutch = ClutchInput({})
    assert clutch.sample(packet(1, 1.)) == (True, True)
    value = packet(2, 0.)
    if head == 'missing':
        del value['tracking']['head']
    else:
        value['tracking']['head'] = head
    assert clutch.sample(value) == (False, True)


def test_persistent_head_loss_cannot_rearm_takeover_and_recovery_requires_new_release(monkeypatch):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=SimpleNamespace))
    now = [2_000_000_000]
    control = ControlOwnership(clock=lambda: now[0], ros_clock=lambda: now[0], recording_enabled=False)
    for peer in control.required_peers:
        control.peer(peer, peer)
    control.enable()

    def acknowledge():
        for _ in range(2):
            for peer in control.required_peers:
                control.acknowledge(control.token, control.phase, peer, peer)
            control.advance(control_ready=True, manual_ready=True)

    control.start_auto(recording_ready=False, control_ready=True)
    acknowledge()
    token = control.token
    bridge = RosBridge({}, lambda *args: None)
    bridge.ownership, bridge._lock = control, control.lock
    receiver = InterventionTeleop.__new__(InterventionTeleop)
    receiver.node = SimpleNamespace(control={}, get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=now[0])))
    receiver.input = ClutchInput({})
    receiver.input_instance, receiver.input_sequence = 'test-device', 0
    receiver.last_input = 2.
    receiver._pause_held = receiver._resume_held = receiver._resume_armed = False
    receiver.input_pub = SimpleNamespace(publish=lambda msg: bridge.receive_intervention_input(json.loads(msg.data)))

    def observe(head, grip):
        now[0] += 1_000_000
        value = packet(receiver.input_sequence+1, grip)
        value['hc_received_ns'] = now[0]
        value['tracking']['head'] = head
        with patch('adapters.core.intervention.time.monotonic', return_value=now[0]*1e-9):
            receiver.observe(json.dumps(value))

    observe(True, 0.)
    assert control.input_armed
    # VrGateway emits this only on the first invalid-head edge. Subsequent
    # hand/controller frames must stay invalid without another timeout call.
    MiddlewareRuntime._on_vr_timeout(SimpleNamespace(ros=bridge), 'head tracking invalid')
    for grip in (0., 1., 0., 1.):
        observe(False, grip)
        assert control.state == 'AUTO' and control.allows(token)
        assert not receiver.valid and not control.input_valid and not control.input_armed
    for _ in range(2):
        observe(True, 1.)
        assert control.state == 'AUTO' and not control.input_armed
    observe(True, 0.)
    observe(True, 1.)
    assert control.state == 'TAKEOVER'
    acknowledge()
    manual_token = control.token
    assert control.state == 'MANUAL' and control.allows(manual_token)
    # The semantic input alone also stops manual output, without relying on
    # ordering or delivery of a gateway timeout notification.
    observe(False, 0.)
    assert control.state == 'FAULT' and not control.allows(manual_token)
    observe(True, 1.)
    assert control.state == 'FAULT'
    control.recover(control_ready=True)
    observe(True, 1.)
    assert control.state == 'HOLD' and not control.input_armed
    observe(True, 0.)
    observe(True, 1.)
    assert control.state == 'TAKEOVER'


def test_pause_resume_edges_share_the_validated_clutch_frame(monkeypatch):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=SimpleNamespace))
    receiver = InterventionTeleop.__new__(InterventionTeleop)
    receiver.node = SimpleNamespace(control={}, get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)))
    receiver.input = ClutchInput({})
    receiver.input_instance, receiver.input_sequence = 'test-device', 0
    receiver.last_input = 2.
    receiver._pause_held = receiver._resume_held = receiver._resume_armed = False
    receiver.input_pub = Mock()

    def observe(sequence, buttons=0, grip=0., **changes):
        value = packet(sequence, grip)
        value['hc_received_ns'] = 2_000_000_000
        value['inputs']['right'].update(held_mask=buttons, pressed_mask=buttons)
        value.update(changes)
        with patch('adapters.core.intervention.time.monotonic', return_value=2.):
            receiver.observe(json.dumps(value))
        return json.loads(receiver.input_pub.publish.call_args.args[0].data)

    assert not observe(1, 1)['resume_requested']  # initially held A cannot start
    observe(2)
    value = observe(3, 3, grip=1.)
    assert value['pause_requested'] and not value['resume_requested'] and value['gripped']
    count = receiver.input_pub.publish.call_count
    observe(3, 3, grip=1.)
    assert receiver.input_pub.publish.call_count == count
    value = observe(4, 3, grip=1.)
    assert not value['pause_requested'] and not value['resume_requested']
    observe(5)
    assert observe(6, 1)['resume_requested']
    value = observe(7, 2, hc_received_ns=500_000_000)
    assert not value['valid'] and not value['pause_requested']
    assert not observe(8, 1)['resume_requested']


@pytest.mark.parametrize('phase', ['revoke', 'grant'])
def test_only_an_active_grant_expires_when_permit_heartbeat_is_late(phase, monkeypatch):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=SimpleNamespace))
    receiver = InterventionTeleop.__new__(InterventionTeleop)
    receiver.active, receiver.granted = True, phase == 'grant'
    receiver.phase, receiver.ack_error = phase, ''
    receiver.permit_time = 1.
    receiver.instance = 'test-teleop'
    receiver.token = ControlToken('controller', 'session', 1, 'vr' if phase == 'grant' else '')
    receiver.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_400_000_000)))
    receiver.ack_pub = Mock()
    with patch('adapters.core.intervention.time.monotonic', return_value=1.4):
        receiver.heartbeat()
    ack = json.loads(receiver.ack_pub.publish.call_args.args[0].data)
    assert not receiver.granted
    assert ack['success'] == (phase == 'revoke')
    assert ack['reason'] == ('' if phase == 'revoke' else 'control permit timeout')


def test_motion_grant_acknowledges_without_headset_or_manual_anchor(monkeypatch):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=SimpleNamespace))
    receiver = InterventionTeleop.__new__(InterventionTeleop)
    receiver.node = Mock()
    receiver.node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_003
    receiver.instance = 'test-teleop'
    receiver.token, receiver.phase, receiver.permit_stamp = None, '', 0
    receiver.active = receiver.granted = receiver.valid = False
    receiver.ack_error = ''
    receiver.ack_pub = Mock()
    receiver.anchor = Mock(side_effect=AssertionError('program grant must not anchor VR'))
    token = ControlToken('controller', 'session', 1, 'motion')
    for ns, phase in [(1, 'revoke'), (2, 'grant')]:
        msg = SimpleNamespace(frame_id='#'.join((token.key(), phase, 'solver', receiver.instance)),
            stamp=SimpleNamespace(sec=1, nanosec=ns))
        receiver.permit(msg)
        ack = json.loads(receiver.ack_pub.publish.call_args.args[0].data)
        assert ack['success'] and ack['phase'] == phase and ack['token'] == token.key()
    assert receiver.active and not receiver.granted and not receiver.valid
    receiver.anchor.assert_not_called()
