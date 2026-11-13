"""Exercise the real legacy snapshot, not a mocked boundary.status method."""
from collections import Counter
from types import SimpleNamespace
import threading
import time

import pytest

from middleware.core.legacy_control import LegacyTopicControl
from middleware.core.ros_bridge import RosBridge


def snapshot_rig():
    bridge = RosBridge({}, lambda *args: None)
    boundary = LegacyTopicControl.__new__(LegacyTopicControl)
    boundary.bridge = bridge
    boundary.feedback_order = ['Joint1_R']
    boundary.hands = {'left': ['left_finger'], 'right': ['right_finger']}
    names = ['Joint1_R', 'left_finger', 'right_finger']
    wall, mono = time.time_ns(), time.monotonic()
    boundary.positions = dict.fromkeys(names, .1)
    boundary.received = dict.fromkeys(names, mono)
    boundary.sample_ns = dict.fromkeys(names, wall)
    boundary.max_age = .25
    boundary.arm_velocity, boundary.max_error = .25, .3
    boundary.teleop = {'grippers': {'max_velocity': 3.5}}
    boundary.last_rejection = boundary.recent_rejection = ''
    boundary.last_rejection_ns = 0
    boundary.diagnostics, boundary.last_clip = Counter(), {}
    calls = []

    def clock_now():
        # A ROS clock call can block in native code while releasing the GIL.
        # It must never execute under the lock shared with actuator publication.
        assert not bridge._lock._is_owned(), 'status clock read holds final-send lock'
        calls.append(None)
        return SimpleNamespace(nanoseconds=wall)

    boundary.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=clock_now))
    bridge._legacy_enabled, bridge._boundary = True, boundary
    return bridge, boundary, wall, calls


def test_real_boundary_status_uses_one_clock_outside_final_send_lock():
    bridge, boundary, wall, calls = snapshot_rig()
    status = bridge.status()['legacy_control']
    assert status['feedback_ready']
    assert status['hands_ready'] == {'left': True, 'right': True}
    assert len(calls) == 1


def test_blocked_health_snapshot_does_not_block_final_send_lock():
    bridge = RosBridge({}, lambda *args: None)
    entered, release = threading.Event(), threading.Event()

    def blocked_status(now):
        entered.set()
        assert release.wait(2)
        return {'hz': 100}

    bridge._trackers['/action/joint_states'] = SimpleNamespace(status=blocked_status)
    result = []
    thread = threading.Thread(target=lambda: result.append(bridge.get_topic_health()))
    thread.start()
    acquired = False
    try:
        assert entered.wait(1)
        acquired = bridge._lock.acquire(blocking=False)
        assert acquired, 'topic-health reader holds final-send lock'
    finally:
        if acquired:
            bridge._lock.release()
        release.set()
        thread.join(2)
    assert not thread.is_alive()
    assert result == [{'/action/joint_states': {'hz': 100}}]


@pytest.mark.parametrize('name', ['Joint1_R', 'left_finger', 'right_finger'])
@pytest.mark.parametrize('failure', ['source_stale', 'future', 'receipt_stale', 'missing'])
def test_status_reuses_clock_without_relaxing_freshness(name, failure):
    bridge, boundary, wall, calls = snapshot_rig()
    if failure == 'source_stale':
        boundary.sample_ns[name] = wall - 251_000_000
    elif failure == 'future':
        boundary.sample_ns[name] = wall + 51_000_000
    elif failure == 'receipt_stale':
        boundary.received[name] -= .251
    else:
        del boundary.positions[name]
    status = bridge.status()['legacy_control']
    expected = {'feedback_ready': name != 'Joint1_R',
                'left': name != 'left_finger', 'right': name != 'right_finger'}
    assert status['feedback_ready'] == expected['feedback_ready']
    assert status['hands_ready'] == {side: expected[side] for side in ('left', 'right')}
    assert len(calls) == 1
