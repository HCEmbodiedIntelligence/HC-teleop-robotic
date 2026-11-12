"""VR takeover preserves ordinary teleoperation targets, with authority fencing."""
import copy
import math
import time

import pytest

from test_legacy_control import Rig, message


def manual_rig():
    rig = Rig()
    rig.control.input(device_instance='vr', sequence=2, valid=True,
                      gripped=True, stamp_ns=time.time_ns())
    rig.advance()
    rig.advance()
    assert rig.control.state == 'MANUAL'
    rig.feed()
    return rig


def ordinary_rig():
    rig = Rig()
    rig.control.disable()
    rig.bridge._command_source = 'vr'
    rig.bridge._command_output_enabled = True
    rig.feed()
    return rig


def arm_values(rig, delta):
    b = rig.boundary
    values = {n: b.positions[n] for n in b.arms}
    name = b.arms[0]
    lo, hi = b.limits[name]
    values[name] += delta if values[name] + delta <= hi else -delta
    assert lo <= values[name] <= hi
    return values


@pytest.mark.parametrize('delta', [.02, .4])
def test_takeover_arm_target_matches_ordinary_teleoperation(delta):
    ordinary, manual = ordinary_rig(), manual_rig()
    target = message(arm_values(manual, delta))
    assert ordinary.bridge.forward_live_command('vr', target, ordinary.boundary.publish_selected)
    manual.boundary.receive_owned(copy.deepcopy(target), manual.control.token)
    expected = [msg for topic, msg in ordinary.wire if topic == '/action/joint_states'][-1]
    actual = [msg for topic, msg in manual.wire if topic == '/action/joint_states']
    assert len(actual) == 1
    actual_values = dict(zip(actual[0].name, actual[0].position))
    assert [actual_values[n] for n in expected.name] == list(expected.position) == list(target.position)
    assert tuple(actual[0].name) == manual.boundary.arm_fields
    assert all(math.isnan(actual_values[n]) for n in set(actual[0].name)-set(target.name))


def test_takeover_hand_targets_do_not_receive_a_second_program_slew_limit():
    ordinary, manual = ordinary_rig(), manual_rig()
    b = manual.boundary
    for field in ['finger_closed', 'finger_open']:
        target = message(dict(zip(b.hands['right'], b.teleop['grippers']['right'][field])))
        assert ordinary.bridge.forward_live_command('vr', target, ordinary.boundary.publish_selected)
        manual.boundary.receive_owned(copy.deepcopy(target), manual.control.token)
    topic = '/action/right_hand/joint_states'
    expected = [m for t, m in ordinary.wire if t == topic]
    actual = [m for t, m in manual.wire if t == topic]
    assert len(expected) == len(actual) == 2
    assert [list(m.position) for m in actual] == [list(m.position) for m in expected]


def test_program_targets_keep_the_existing_slew_and_point_jump_guards():
    rig = Rig()
    b = rig.boundary
    b.program(message(arm_values(rig, .02)), 'arms')
    sent = [m for t, m in rig.wire if t == '/action/joint_states']
    assert len(sent) == 1
    assert max(abs(q-b.positions[n]) for n,q in zip(sent[0].name,sent[0].position) if n in b.arms) <= .00251
    b.program(message(arm_values(rig, .4)), 'arms')
    assert len([m for t,m in rig.wire if t == '/action/joint_states']) == 1
    assert b.last_rejection.startswith('target too far from measured position:')


@pytest.mark.parametrize('failure', ['stale_command', 'stale_feedback', 'nan', 'joint_limit', 'collision', 'revoked'])
def test_manual_target_still_requires_fresh_safe_data_and_current_authority(failure):
    rig = manual_rig()
    b = rig.boundary
    token = rig.control.token
    values = arm_values(rig, .02)
    stamp = time.time_ns()
    if failure == 'stale_command': stamp -= 310_000_000
    if failure == 'stale_feedback': b.sample_ns[b.arms[0]] -= 300_000_000
    if failure == 'nan': values[b.arms[0]] = float('nan')
    if failure == 'joint_limit': values[b.arms[0]] = b.limits[b.arms[0]][1] + .01
    if failure == 'collision': b.collision.check.return_value = (False, ('arm','body'), False)
    if failure == 'revoked': rig.control.hold()
    before = len(rig.wire)
    b.receive_owned(message(values, stamp), token)
    assert len(rig.wire) == before
