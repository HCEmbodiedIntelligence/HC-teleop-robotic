import copy
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from adapters.core.intervention import ClutchInput, InterventionTeleop
from middleware.core.control_ownership import ControlToken


def packet(sequence, grip):
    pose = dict(position=[0., 0., 0.], quaternion=[0., 0., 0., 1.])
    return dict(protocol_version=3, sequence=sequence, vr_timestamp=sequence*.01,
                tracking=dict(left=True, right=True), poses=dict(left=pose, right=pose),
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


@pytest.mark.parametrize('phase', ['revoke', 'grant'])
def test_only_an_active_grant_expires_when_permit_heartbeat_is_late(phase):
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
