"""DAgger hand metadata accompanies commands; it is not an arm command."""
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
    rig.wire.clear()
    return rig


def hand_values(boundary, sides=('right', 'left')):
    values = {}
    for side in sides:
        names = boundary.hands[side]
        values.update(zip(names, boundary.teleop['grippers'][side]['finger_open']))
        values[boundary.teleop['grippers'][side]['binary_state_joint']] = 0.
    return values


@pytest.mark.parametrize('sides', [('right',), ('left',), ('right', 'left')])
def test_marked_hand_targets_create_only_physical_hand_projections(sides):
    rig = manual_rig()
    b = rig.boundary
    values = hand_values(b, sides)
    target = message(values)
    b.receive_owned(target, rig.control.token, frame_id='')
    assert {topic for topic, _ in rig.wire} == {
        b.teleop['grippers'][side]['finger_topic'] for side in sides}
    assert len(rig.wire) == len(sides)
    assert not b.diagnostics['rejected:command has no physical joints']
    assert not b.diagnostics['accepted_arms']
    for topic, sent in rig.wire:
        side = next(side for side in sides
                    if b.teleop['grippers'][side]['finger_topic'] == topic)
        expected = set(b.hands[side]) | {
            b.teleop['grippers'][side]['binary_state_joint']}
        assert set(sent.name) == expected
        assert dict(zip(sent.name, sent.position)) == {n: values[n] for n in expected}
        assert sent.header.stamp == target.header.stamp
        assert sent.header.frame_id == ''


def test_real_arms_and_marked_hands_still_project_all_three_groups():
    rig = manual_rig()
    b = rig.boundary
    values = {n: b.positions[n] for n in b.arms}
    values.update(hand_values(b))
    b.receive_owned(message(values), rig.control.token, frame_id='')
    assert {topic for topic, _ in rig.wire} == {
        '/action/joint_states', '/action/right_hand/joint_states',
        '/action/left_hand/joint_states'}
    assert len(rig.wire) == 3
    assert not b.diagnostics['rejected:command has no physical joints']


@pytest.mark.parametrize('full_layout', [False, True])
def test_marker_only_packet_is_still_explicitly_rejected(full_layout):
    rig = manual_rig()
    b = rig.boundary
    values = {n: math.nan for n in b.arm_fields} if full_layout else {}
    values.update({n: 0. for n in b.sim_joints})
    b.receive_owned(message(values), rig.control.token, frame_id='')
    assert not rig.wire
    assert b.diagnostics['rejected:command has no physical joints'] == 1
    assert b.last_rejection == 'command has no physical joints'


@pytest.mark.parametrize('case', ['unknown', 'duplicate', 'nonfinite', 'bad_binary',
                                  'bad_other_hand_binary'])
def test_invalid_marked_packet_never_reaches_actuators_and_is_diagnosed(case):
    rig = manual_rig()
    b = rig.boundary
    values = hand_values(b, ('right',))
    if case == 'unknown':
        values['unexpected_axis'] = 0.
    elif case == 'nonfinite':
        values[b.hands['right'][0]] = math.nan
    elif case == 'bad_binary':
        values[b.teleop['grippers']['right']['binary_state_joint']] = 2.
    elif case == 'bad_other_hand_binary':
        values[b.teleop['grippers']['left']['binary_state_joint']] = 2.
    target = message(values)
    if case == 'duplicate':
        target.name.append(target.name[0])
        target.position.append(target.position[0])
    b.receive_owned(target, rig.control.token, frame_id='')
    assert not rig.wire
    assert sum(v for k, v in b.diagnostics.items() if k.startswith('rejected:')) == 1
    assert b.last_rejection


def test_final_prepare_still_rejects_marker_only_publication():
    rig = manual_rig()
    b = rig.boundary
    assert b.prepare(message({n: 0. for n in b.sim_joints})) is None
    assert b.last_rejection == 'command has no physical joints'


def test_home_keeps_physical_arm_projection_without_virtual_only_command():
    rig = manual_rig()
    b = rig.boundary
    values = {n: b.positions[n] for n in b.arms}
    b.receive_owned(message(values), rig.control.token, frame_id='home')
    assert len(rig.wire) == 1
    assert rig.wire[0][0] == '/action/joint_states'
    assert set(b.arms) <= set(rig.wire[0][1].name)
