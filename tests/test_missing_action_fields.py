"""Fixed JointState layout with NaN gaps; execution never receives a gap."""
import copy
import json
import math
import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip('sensor_msgs.msg')
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState

from middleware.core.dataset_topics import DatasetTopics
from middleware.core.topic_recorder import TopicRecorder
from test_legacy_control import Rig, message


def enter_manual(rig):
    rig.control.input(device_instance='vr', sequence=2, valid=True,
                      gripped=True, stamp_ns=time.time_ns())
    rig.advance()
    rig.advance()
    assert rig.control.state == 'MANUAL'


def finite_values(msg):
    return {n:v for n,v in zip(msg.name, msg.position) if math.isfinite(v)}


def arm_values(b):
    return {**{n:b.positions[n] for n in b.arms}, 'R_ban':.4, 'L_ban':.2}


@pytest.mark.parametrize('manual', [False, True])
@pytest.mark.parametrize('body', [False, True])
def test_body_fields_preserve_values_or_nan_without_copying_feedback(manual, body):
    rig = Rig()
    b = rig.boundary
    if manual:
        enter_manual(rig)
    values = arm_values(b)
    if body:
        values.update(leg_1=.11, leg_2=.22, zhi=.33, head_1=.17, head_2=-.08)
    original = message(values)
    before = copy.deepcopy(original)
    if manual:
        b.receive_owned(original, rig.control.token, frame_id='')
    else:
        b.program(original, 'arms')
    assert original == before
    topic, sent = rig.wire[-1]
    assert topic == '/action/joint_states'
    assert sent.name == list(b.arm_fields) and len(sent.position) == 21
    assert finite_values(sent) == values
    assert all(math.isnan(q) for n,q in zip(sent.name, sent.position) if n not in values)
    assert not set(b.model_only_joints).intersection(b.positions)
    if not body:
        assert not set(b.waist).intersection(b.last_command_stamp)
    if manual:
        evidence = rig.control.sent(rig.control.token, sent, time.time_ns())
        assert dict(zip(evidence['names'], evidence['positions'])) == values
        json.dumps(evidence, allow_nan=False)


def test_named_nan_gaps_are_ignored_but_invalid_partial_commands_still_fail():
    rig = Rig()
    b = rig.boundary
    values = arm_values(b)
    full = message({n:values.get(n, math.nan) for n in b.arm_fields})
    b.program(full, 'arms')
    assert len(rig.wire) == 1 and finite_values(rig.wire[-1][1]) == values
    invalid = message({**values, b.arms[0]:math.nan})
    b.program(invalid, 'arms')
    assert len(rig.wire) == 1
    infinite = message({n:values.get(n, math.nan) for n in b.arm_fields})
    infinite.position[0] = math.inf
    b.program(infinite, 'arms')
    assert len(rig.wire) == 1
    empty = message({n:math.nan for n in b.arm_fields})
    b.program(empty, 'arms')
    assert len(rig.wire) == 1


def test_nan_observation_does_not_become_feedback():
    rig = Rig()
    b = rig.boundary
    before = dict(b.positions)
    b.state(message({**before, b.waist[0]:math.nan}))
    assert b.positions == before


def test_old_manual_generation_and_hold_cannot_publish_missing_field_actions():
    rig = Rig()
    b = rig.boundary
    enter_manual(rig)
    token = rig.control.token
    b.receive_owned(message(arm_values(b)), token, frame_id='')
    assert len(rig.wire) == 1
    rig.control.input(device_instance='vr', sequence=3, valid=True,
                      gripped=False, stamp_ns=time.time_ns())
    rig.advance()
    rig.advance()
    assert rig.control.state == 'AUTO'
    b.receive_owned(message(arm_values(b)), token, frame_id='')
    assert len(rig.wire) == 1
    b.program(message(arm_values(b)), 'arms')
    assert len(rig.wire) == 2
    rig.control.hold()
    b.program(message(arm_values(b)), 'arms')
    assert len(rig.wire) == 2


@pytest.mark.parametrize('body', [False, True])
def test_mcap_projection_replay_and_hardware_ignore_missing_fields(tmp_path, monkeypatch, body):
    hardware = os.environ.get('HC_X1_ROOT')
    if not hardware:
        pytest.skip('HC_X1_ROOT required for the actual actuator adapter')
    monkeypatch.syspath_prepend(hardware)
    from hc_x1.joint_command_smoother import JointCommandSmootherNode
    rig = Rig()
    b = rig.boundary
    values = arm_values(b)
    if body:
        values.update(leg_1=.11, leg_2=.22, zhi=.33)
    b.program(message(values), 'arms')
    action, = [m for t,m in rig.wire if t == '/action/joint_states']
    waist = []
    topics = ['/action/joint_states', '/action/waist/joint_states']
    projector = SimpleNamespace(groups={'waist':list(b.waist)},
        publishers={topics[1]:SimpleNamespace(publish=waist.append)})
    DatasetTopics.joints(projector, 'action', action)
    assert waist[0].name == list(b.waist)
    assert all(math.isfinite(q) == body for q in waist[0].position)
    raw = {t:serialize_message(m) for t,m in zip(topics, (action, waist[0]))}
    recorder = TopicRecorder({'directory':str(tmp_path)}, tmp_path,
        topics=[{'topic':t, 'type':'sensor_msgs/msg/JointState'} for t in topics])
    try:
        path = Path(recorder.start('missing-action.mcap'))
        for topic,payload in raw.items():
            assert recorder.record({'kind':'ros_message', 'topic':topic,
                'msg_type':'sensor_msgs/msg/JointState', '_raw':payload})
        assert recorder.stop()['error'] is None
        with path.open('rb') as stream:
            records = {c.topic:m.data for _,c,m in make_reader(stream, validate_crcs=True).iter_messages()}
        assert records == raw
        restored = deserialize_message(records[topics[0]], JointState)
        assert restored.name == list(b.arm_fields) and finite_values(restored) == values
    finally:
        recorder.shutdown()
    rig.control.disable()
    rig.feed()
    bridge, node = rig.bridge, b.node
    node.get_clock.return_value.now.side_effect = lambda:Time(nanoseconds=time.time_ns())
    bridge.set_command_output_enabled(True)
    assert bridge.begin_replay()
    rig.wire.clear()
    assert bridge.publish_raw(topics[0], 'sensor_msgs/msg/JointState', records[topics[0]])
    bridge._drain_commands(node, {}, get_message, set_message_fields)
    replayed, = [m for t,m in rig.wire if t == topics[0]]
    assert replayed.name == action.name and finite_values(replayed) == values
    motor = SimpleNamespace(hardware_ready=True, last_target_time=-1.)
    JointCommandSmootherNode._target_callback(motor, replayed)
    assert dict(zip(motor.target_names, motor.target_positions)) == {
        n:v for n,v in values.items() if n not in b.sim_joints}
    assert all(math.isfinite(q) for q in motor.target_positions)
    last = motor.last_target_time
    JointCommandSmootherNode._target_callback(motor, message({n:math.nan for n in b.arm_fields}))
    assert not motor.target_names and not motor.target_positions
    assert motor.last_target_time == last


@pytest.mark.skipif(os.environ.get('HC_RUN_LEGACY_ROS_TESTS') != '1', reason='isolated ROS test')
def test_websocket_joint_payload_uses_null_without_mutating_ros_nan():
    import rclpy
    from middleware.core.ros_bridge import RosBridge
    domain = int(os.environ['ROS_DOMAIN_ID'])
    assert 200 <= domain <= 232 and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    events = []
    rclpy.init()
    probe = rclpy.create_node('missing_action_json_probe',
        cli_args=['--ros-args', '--disable-external-lib-logs'])
    topic = '/action/joint_states'
    publisher = probe.create_publisher(JointState, topic, 10)
    bridge = RosBridge({'enabled':True, 'domain_id':domain, 'subscriptions':[
        {'topic':topic, 'type':'sensor_msgs/msg/JointState', 'outputs':['websocket']}]},
        lambda event, outputs:events.append(event))
    sample = message({'Joint1_R':.2, 'leg_1':math.nan, 'head_1':math.nan})
    try:
        bridge.start()
        deadline = time.monotonic() + 6.
        while not any(e.get('topic') == topic for e in events):
            assert time.monotonic() < deadline, bridge.status()
            publisher.publish(sample)
            rclpy.spin_once(probe, timeout_sec=.02)
        event = next(e for e in events if e.get('topic') == topic)
        assert event['payload']['position'] == [.2, None, None]
        json.dumps(event, allow_nan=False)
        assert math.isnan(sample.position[1]) and math.isnan(sample.position[2])
    finally:
        bridge.stop()
        probe.destroy_node()
        rclpy.shutdown()
