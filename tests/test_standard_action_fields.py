"""Standard action fields at producers, ownership gate, MCAP and hardware edge."""
import copy
import math
import os
from pathlib import Path
import threading
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('sensor_msgs.msg')
from sensor_msgs.msg import JointState
from rclpy.time import Time
from test_legacy_control import Rig, message


def manual(r):
    r.control.input(device_instance='vr', sequence=2, valid=True, gripped=True, stamp_ns=time.time_ns())
    r.advance(); r.advance(); r.feed()
    assert r.control.state == 'MANUAL'


@pytest.mark.parametrize('side', ['arms', 'right', 'left'])
def test_standard_fields_pass_gate_without_altering_physical_targets(side):
    r = Rig(); b = r.boundary
    names = b.arms if side == 'arms' else b.hands[side]
    values = {n:b.positions[n] for n in names}
    plain = message(values)
    labels = {'R_ban':.8, 'L_ban':.2} if side == 'arms' else {side[0].upper()+'_ban':1.}
    target = message({**values, **labels}, b.stamp(plain))
    a, z = b.prepare(plain), b.prepare(target)
    assert a is not None and z is not None, b.last_rejection
    finite = lambda msg:{n:q for n,q in zip(msg.name,msg.position) if math.isfinite(q)}
    assert finite(z[0]) == {**finite(a[0]), **labels}
    b.program(target, side)
    topic, sent = r.wire[-1]
    assert topic == ('/action/joint_states' if side == 'arms' else f'/action/{side}_hand/joint_states')
    assert sent.name == (list(b.arm_fields) if side == 'arms' else target.name)
    assert sent.header == target.header
    assert not any(t.startswith('/internal/recording') for t, _ in r.wire)
    r.control.hold()
    count = len(r.wire)
    b.program(message({**values, **labels}), side)
    assert len(r.wire) == count


def test_hand_fields_do_not_share_reorder_tracking_with_combined_fields():
    r = Rig(); b = r.boundary
    manual(r)
    stamp = time.time_ns()
    arm = message({**{n:b.positions[n] for n in b.arms}, 'R_ban':.8, 'L_ban':.2}, stamp)
    both = message({**{n:b.positions[n] for n in b.hand_names}, 'R_ban':1., 'L_ban':0.}, stamp)
    b.receive_owned(arm, r.control.token, frame_id='')
    b.receive_owned(both, r.control.token, frame_id='')
    assert len(r.wire) == 3
    assert [len(msg.name) for _, msg in r.wire] == [21, 11, 11]
    token = r.control.token
    r.control.release()
    b.receive_owned(message(dict(zip(arm.name, arm.position))), token, frame_id='')
    assert len(r.wire) == 3


@pytest.mark.parametrize('side', ['arms', 'right', 'left'])
def test_replay_keeps_standard_fields_on_public_action_topic(side):
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.set_message import set_message_fields
    from rosidl_runtime_py.utilities import get_message
    r = Rig(); b = r.boundary; bridge = r.bridge; node = b.node
    node.get_clock.return_value.now.side_effect = lambda: Time(nanoseconds=time.time_ns())
    r.control.disable(); r.feed(); bridge.set_command_output_enabled(True)
    assert bridge.begin_replay()
    names = b.arms if side == 'arms' else b.hands[side]
    values = {n:b.positions[n] for n in names}
    values.update({'R_ban':.8, 'L_ban':.2} if side == 'arms' else {side[0].upper()+'_ban':1.})
    target = message(values, 1_000_000_000)
    topic = '/action/joint_states' if side == 'arms' else f'/action/{side}_hand/joint_states'
    assert bridge.publish_raw(topic, 'sensor_msgs/msg/JointState', serialize_message(target))
    bridge._drain_commands(node, {}, get_message, set_message_fields)
    sent = [m for t,m in r.wire if t == topic]
    assert len(sent) == 1
    assert sent[0].name == target.name and sent[0].position == target.position


def test_actual_gateway_outputs_standard_fields(monkeypatch):
    root = os.environ.get('HC_TJ_GATEWAY_ROOT')
    if not root:
        pytest.skip('HC_TJ_GATEWAY_ROOT required for actual gateway producer')
    monkeypatch.syspath_prepend(root)
    from projects.holobrain.scripts.hc_tj_http_robot_gateway import _ros_node_class
    r = Rig(); b = r.boundary
    action = np.array([*[b.positions[n] for n in b.arms[:7]], .8,
                       *[b.positions[n] for n in b.arms[7:]], .2])
    emitted = {}
    def publisher(channel):
        def publish(msg):
            emitted[channel] = copy.deepcopy(msg)
            b.program(msg, channel)
        return SimpleNamespace(publish=publish)
    fake = SimpleNamespace(
        _action_lock=threading.RLock(), _sync_control=lambda:None, _gripper_targets={},
        config=SimpleNamespace(enable_dagger=True, enable_hands=True, robot_interface='legacy',
            action_timeout_s=1., state_timeout_s=1., control_hz=30.),
        _gate=SimpleNamespace(allows=lambda window:True), _scheduled_window=object(), fault=None,
        _next_publisher_check=float('inf'),
        _action_stream=SimpleNamespace(actions=action, pause_if_timed_out=lambda *a:False, next_action=lambda:action),
        observations=SimpleNamespace(latest_joints=lambda:SimpleNamespace(received_monotonic_s=time.monotonic())),
        _limiter=SimpleNamespace(step=lambda arm, dt:arm),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=time.time_ns())),
        _joint_publisher=publisher('arms'), _right_hand_publisher=publisher('right'), _left_hand_publisher=publisher('left'),
        _latch_fault=Mock())
    _ros_node_class()._control_tick(fake)
    fake._latch_fault.assert_not_called()
    assert len(r.wire) == 3
    assert emitted['arms'].name == list(b.arms)+['R_ban','L_ban']
    assert list(emitted['arms'].position[-2:]) == pytest.approx([3.5/30]*2)
    for side, expected in [('right',1.), ('left',0.)]:
        sent = next(m for t,m in r.wire if t == f'/action/{side}_hand/joint_states')
        assert len(sent.name) == 11 and sent.position[-1] == expected and not sent.header.frame_id
    assert all(not m.header.frame_id for m in emitted.values())


def test_manual_producer_uses_standard_mapping_without_changing_fingers():
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from adapters.core.intervention import InterventionTeleop
    r = Rig(); b = r.boundary; manual(r)
    now = time.monotonic()
    node = SimpleNamespace(backend='v23', config=b.teleop, control=b.teleop['control'],
        homing=False, last_command={}, joint_state={},
        arms={s:SimpleNamespace(active=True, reference_vr=object(), reference_local_ee=object()) for s in b.hands},
        _update_arm_target=Mock(), _publish_target_poses=Mock(), _publish_status=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=time.time_ns())))
    node._gripper_position = MethodType(RobotArmTeleopNode._gripper_position, node)
    owner = InterventionTeleop.__new__(InterventionTeleop)
    owner.node=node; owner.token=r.control.token; owner.granted=owner.valid=True
    owner.input=SimpleNamespace(pressed=True); owner.last_input=owner.permit_time=now
    owner.last_hand_step=now-.01; owner.triggers={'right':.8, 'left':.2}
    owner.grippers={}
    for side in b.hands:
        cfg=b.teleop['grippers'][side]
        for n, opened, closed in zip(cfg['finger_joint_names'], cfg['finger_open'], cfg['finger_closed']):
            owner.grippers[n]=[side,owner.triggers[side],opened,opened,opened,closed]
    owner.aux_pub=Mock(); owner.owned_pub=Mock()
    owner.tick(now, True)
    aux=owner.aux_pub.publish.call_args.args[0]
    assert aux.name == ['R_ban','L_ban'] and list(aux.position) == pytest.approx([.035,.035])
    hand=owner.owned_pub.publish.call_args.args[0]
    values=dict(zip(hand.name,hand.position))
    assert values['R_ban']==1. and values['L_ban']==0.
    for n, (side, trigger, initial, previous, opened, closed) in owner.grippers.items():
        assert values[n] == pytest.approx(np.clip(opened+trigger*(closed-opened), opened-.035, opened+.035))
    b.receive_owned(hand, owner.token, frame_id='')
    assert [len(m.name) for _,m in r.wire] == [11,11]


def test_hardware_smoother_excludes_only_virtual_grippers(monkeypatch):
    root=os.environ.get('HC_X1_ROOT')
    if not root:
        pytest.skip('HC_X1_ROOT required for actual hardware boundary')
    monkeypatch.syspath_prepend(root)
    from hc_x1.joint_command_smoother import JointCommandSmootherNode
    fake=SimpleNamespace(hardware_ready=True)
    target=message({'Joint1_R':.2,'R_ban':.8,'Joint1_L':-.3,'L_ban':.4,'leg_1':.1})
    JointCommandSmootherNode._target_callback(fake,target)
    assert fake.target_names == ['Joint1_R','Joint1_L','leg_1']
    assert fake.target_positions == [.2,-.3,.1]
    assert target.name == ['Joint1_R','R_ban','Joint1_L','L_ban','leg_1']


def test_oct23_real_action_messages_preserve_names_positions_and_order():
    from collections import Counter
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    bag=os.environ.get('HC_STANDARD_ACTION_BAG')
    if not bag:
        pytest.skip('HC_STANDARD_ACTION_BAG required for historical data regression')
    r=Rig(); b=r.boundary; r.control.disable()
    counts=Counter()
    topics=['/action/joint_states','/action/right_hand/joint_states','/action/left_hand/joint_states']
    with Path(bag).open('rb') as stream:
        for _, channel, record in make_reader(stream).iter_messages(topics=topics):
            old=deserialize_message(record.data, JointState)
            target=copy.deepcopy(old)
            stamp=time.time_ns()
            target.header.stamp.sec, target.header.stamp.nanosec=divmod(stamp,1_000_000_000)
            for n in b.feedback_order:
                b.sample_ns[n]=stamp; b.received[n]=time.monotonic()
            # This is an offline gate test. Supply the frame's physical state
            # to the fake node, including the historical head/waist commands.
            for n,v in zip(target.name,target.position):
                if n not in b.sim_joints:
                    b.positions[n]=v; b.sample_ns[n]=stamp; b.received[n]=time.monotonic()
            prepared=b.prepare(target)
            assert prepared is not None, b.last_rejection
            assert prepared[0].name == old.name
            assert prepared[0].position == old.position
            assert prepared[0].velocity == old.velocity and prepared[0].effort == old.effort
            assert prepared[0].header.frame_id == old.header.frame_id
            counts[(channel.topic,len(old.name))]+=1
    assert counts == {('/action/joint_states',18):1443,('/action/joint_states',21):80,
                      ('/action/right_hand/joint_states',11):3292,
                      ('/action/left_hand/joint_states',11):3293}
