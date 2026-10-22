import copy

from adapters.core.intervention import ClutchInput


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
