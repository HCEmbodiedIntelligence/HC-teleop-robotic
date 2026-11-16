"""Move CDR conversion, feedback parsing and observer work outside authority."""
import math
from types import SimpleNamespace
import threading
import time

import pytest

pytest.importorskip('sensor_msgs.msg')
from sensor_msgs.msg import JointState
from rclpy.serialization import serialize_message, deserialize_message

from test_legacy_control import Rig, message


def live_rig():
    rig = Rig()
    rig.control.disable()
    rig.feed()
    rig.bridge._command_source = 'vr'
    rig.bridge._command_output_enabled = True
    return rig


class NativeLikePublisher:
    msg_type = JointState

    def __init__(self, bridge):
        self.bridge, self.sent = bridge, []

    def publish(self, raw):
        assert self.bridge._lock._is_owned(), 'actual publication escaped final authority gate'
        assert isinstance(raw, bytes)
        self.sent.append(raw)


@pytest.mark.parametrize('channel', ['arms', 'right', 'left'])
def test_prepared_bytes_preserve_fields_route_and_success_accounting(channel):
    rig = live_rig()
    bridge, boundary = rig.bridge, rig.boundary
    boundary._publisher_type = NativeLikePublisher
    boundary.output = NativeLikePublisher(bridge)
    boundary.hand_pubs = {s: NativeLikePublisher(bridge) for s in boundary.hands}
    fields = boundary.arms if channel == 'arms' else boundary.hands[channel]
    values = {n: boundary.positions[n] for n in fields}
    if channel != 'arms':
        values[boundary.teleop['grippers'][channel]['binary_state_joint']] = 1.
    source = message(values)
    encoded = []
    def encode(target):
        assert not bridge._lock._is_owned(), 'CDR conversion holds the final send lock'
        raw = serialize_message(target)
        encoded.append(raw)
        return raw
    boundary._serialize_message = encode
    assert bridge.forward_live_command('vr', source, boundary.publish_selected)
    selected = boundary.output if channel == 'arms' else boundary.hand_pubs[channel]
    assert selected.sent == encoded
    target = deserialize_message(selected.sent[0], JointState)
    assert target.header == source.header
    assert target.name == source.name and target.position == source.position
    assert boundary.diagnostics['published_arms'] == (1 if channel == 'arms' else 0)
    assert (boundary.last_arm_publish_time is not None) == (channel == 'arms')


def test_commit_does_not_change_the_target_that_was_serialized():
    rig = Rig()
    boundary = rig.boundary
    source = message({n: boundary.positions[n] + .001 for n in boundary.arms})
    prepared = boundary.prepare(source)
    assert prepared is not None
    raw = serialize_message(prepared[0])
    # Active program output has the complete model layout and NaN held fields.
    assert any(math.isnan(v) for v in prepared[0].position)
    with rig.bridge._lock:
        assert boundary.commit(prepared, intervention=True)
    assert serialize_message(prepared[0]) == raw


@pytest.mark.parametrize('failure', ['revoke', 'stale'])
def test_slow_serialization_never_replays_a_target_after_authority_or_age_changes(failure):
    rig = live_rig()
    bridge, boundary = rig.bridge, rig.boundary
    boundary._publisher_type = NativeLikePublisher
    boundary.output = NativeLikePublisher(bridge)
    source = message({n: boundary.positions[n] for n in boundary.arms})
    fixed_wall = [time.time_ns()]
    boundary._clock = SimpleNamespace(now_ns=lambda: fixed_wall[0])
    def encode(target):
        assert not bridge._lock._is_owned()
        if failure == 'revoke':
            bridge.set_command_output_enabled(False)
        else:
            fixed_wall[0] += 251_000_000
        return serialize_message(target)
    boundary._serialize_message = encode
    assert not bridge.forward_live_command('vr', source, boundary.output.publish)
    assert not boundary.output.sent
    assert not boundary.last_command_stamp


def test_feedback_field_parsing_is_unlocked_and_freshness_is_rechecked_after_lock():
    rig = live_rig()
    bridge, boundary = rig.bridge, rig.boundary
    original = boundary.values
    def values(*args, **kwargs):
        assert not bridge._lock._is_owned(), 'feedback parsing holds actuator gate'
        return original(*args, **kwargs)
    boundary.values = values
    rig.feed()  # Both arm and hand paths must parse outside the shared lock.
    fixed_wall = [time.time_ns()]
    boundary._clock = SimpleNamespace(now_ns=lambda: fixed_wall[0])
    source = message({boundary.arms[0]: boundary.positions[boundary.arms[0]] + .1}, stamp=fixed_wall[0])
    before = dict(boundary.positions)
    parsed = threading.Event()
    def parsed_values(*args, **kwargs):
        result = values(*args, **kwargs)
        parsed.set()
        return result
    boundary.values = parsed_values
    with bridge._lock:
        thread = threading.Thread(target=lambda: boundary.state(source))
        thread.start()
        assert parsed.wait(1)
        fixed_wall[0] += 251_000_000
    thread.join(1)
    assert not thread.is_alive()
    assert boundary.positions == before


def test_telemetry_count_and_status_observation_do_not_hold_authority_together():
    rig = live_rig()
    bridge = rig.bridge
    done = threading.Event()
    with bridge._lock:
        thread = threading.Thread(target=lambda: (bridge._count_telemetry_message(), done.set()))
        thread.start()
        assert done.wait(1), 'telemetry counter waits for authority lock'
    thread.join(1)
    assert bridge.status()['messages'] == 1
