"""Actual ROS direct-client output through the HC gate and MCAP writer."""
import copy
import math
import os
import threading
import time
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('sensor_msgs.msg')
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from sensor_msgs.msg import JointState

from middleware.core.topic_recorder import TopicRecorder
from test_legacy_control import Rig
from middleware.core.dataset_topics import DatasetTopics


@pytest.mark.parametrize('revoked', [False, True])
def test_direct_model_action_survives_gate_and_recording(monkeypatch, tmp_path, revoked):
    root = os.environ.get('HC_TJ_DIRECT_ROOT')
    if not root:
        pytest.skip('HC_TJ_DIRECT_ROOT required for the actual direct client')
    monkeypatch.syspath_prepend(root)
    from projects.holobrain.scripts import hc_tj_async_client as client

    rig = Rig()
    boundary = rig.boundary
    feedback = np.array([boundary.positions[n] for n in client.CONTROL_JOINT_NAMES])
    # Prove the existing waist branch copies current feedback, not defaults.
    feedback[14:] = [.13, .27, .39]
    action = np.r_[feedback[:7], .8, feedback[7:14], .2]
    limiter = client.ArmCommandLimiter(np.full(14, -10.), np.full(14, 10.))
    limiter.reset(feedback[:14])
    emitted, physical = {}, {}

    def publisher(channel):
        def publish(msg):
            emitted[channel] = copy.deepcopy(msg)
            plain = JointState(header=copy.deepcopy(msg.header))
            pairs = [(n, q) for n, q in zip(msg.name, msg.position)
                     if n not in boundary.sim_joints
                     and (channel != 'arms' or n in boundary.arms)]
            plain.name = [n for n, _ in pairs]
            plain.position = [q for _, q in pairs]
            prepared = boundary.prepare(plain)
            if prepared is not None:
                physical[channel] = dict(zip(prepared[0].name, prepared[0].position))
            boundary.program(msg, channel)
        return SimpleNamespace(publish=publish)

    node = SimpleNamespace(
        config=SimpleNamespace(enable_control=True, enable_dagger=True,
            robot_interface=client.RobotInterface.LEGACY,
            allow_competing_publishers=True, control_hz=100.),
        fault=None, _motion=None, _lock=threading.RLock(), _reset_complete=True,
        _native_actions=action[None, :], _received_action_chunk=True,
        _native_started=time.monotonic(), _last_publish_time=None,
        _default_joint_positions=np.array(client.DEFAULT_JOINT_POSITIONS),
        _check_feedback_freshness=lambda sample: None,
        observations=SimpleNamespace(latest_joints=lambda: client.TimedValue(time.time_ns(), feedback)),
        limiter=limiter, _diagnostic_counts=Counter(), _publish_gap_max_ms=0.,
        _gripper_targets={}, _control_window=object(), _sync_control=Mock(),
        _gate=SimpleNamespace(allows=lambda window: True),
        _relay=SimpleNamespace(ready=lambda window: True),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=time.time_ns())),
        _joint_publisher=publisher('arms'), _right_hand_publisher=publisher('right'),
        _left_hand_publisher=publisher('left'), _latch_fault=Mock())
    if revoked:
        rig.control.hold()
    client._ros_node_class()._control_tick_permitted(node)
    node._latch_fault.assert_not_called()
    assert emitted['arms'].name == list(client.CONTROL_JOINT_NAMES) + ['R_ban', 'L_ban']
    assert list(emitted['arms'].position[:17]) == pytest.approx(feedback)
    assert list(emitted['arms'].position[-2:]) == pytest.approx([.035, .035])
    for side, closure in [('right', .8), ('left', .2)]:
        names, positions = client.hand_command(side, closure)
        assert emitted[side].name == list(names) + [side[0].upper() + '_ban']
        assert list(emitted[side].position[:10]) == pytest.approx(positions)
        assert emitted[side].position[-1] == float(closure >= .5)
        assert emitted[side].header == emitted['arms'].header

    action_topics = ['/action/joint_states', '/action/right_hand/joint_states',
                     '/action/left_hand/joint_states']
    selected = action_topics + ['/action/waist/joint_states']
    wire = [(t, m) for t, m in rig.wire if t in action_topics]
    assert len(wire) == (0 if revoked else 3)
    if not revoked:
        for (topic, msg), channel, count in zip(wire, ['arms', 'right', 'left'], [21, 11, 11]):
            assert len(msg.name) == count
            joints = boundary.arms if channel == 'arms' else boundary.hands[channel]
            assert {n:q for n,q in zip(msg.name,msg.position) if n in joints} == {n:q for n,q in physical[channel].items() if n in joints}
            assert not msg.header.frame_id
            assert msg.header == emitted[channel].header
            if channel == 'arms':
                values = dict(zip(msg.name,msg.position))
                assert [values[n] for n in boundary.waist] == pytest.approx(feedback[14:])
                assert math.isnan(values['head_1']) and math.isnan(values['head_2'])
        projector = SimpleNamespace(groups={'waist':list(boundary.waist)},
            publishers={selected[-1]:SimpleNamespace(publish=lambda m:wire.append((selected[-1],m)))})
        DatasetTopics.joints(projector, 'action', wire[0][1])

    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path,
        topics=[{'topic': t, 'type': 'sensor_msgs/msg/JointState'} for t in selected])
    raw = {t: bytes(serialize_message(msg)) for t, msg in wire}
    try:
        path = Path(recorder.start('direct-client.mcap'))
        for topic, payload in raw.items():
            assert recorder.record({'kind': 'ros_message', 'topic': topic,
                'msg_type': 'sensor_msgs/msg/JointState', '_raw': payload})
        assert recorder.stop()['error'] is None
        with path.open('rb') as stream:
            reader = make_reader(stream, validate_crcs=True)
            records = list(reader.iter_messages())
            summary = reader.get_summary()
        assert {c.topic for c in summary.channels.values()} == set(selected)
        assert {c.topic: m.data for _, c, m in records} == raw
        assert sum(c.topic == selected[-1] for _, c, _ in records) == (0 if revoked else 1)
        assert [len(deserialize_message(m.data, JointState).name)
                for _, c, m in records] == ([] if revoked else [21, 11, 11, 3])
    finally:
        recorder.stop()
