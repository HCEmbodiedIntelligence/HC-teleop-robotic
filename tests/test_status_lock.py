"""Status inspection must not wait for ROS clocks under the send lock."""
from types import SimpleNamespace

from middleware.core.ros_bridge import RosBridge


def test_status_reads_clocks_and_tracker_outside_send_lock():
    bridge = RosBridge({}, lambda *args: None)
    def unlocked():
        assert not bridge._lock._is_owned()
    def ownership():
        unlocked()
        return {'state': 'OFF'}
    def clock_now():
        unlocked()
        return SimpleNamespace(nanoseconds=12345)
    def boundary_status(*, wall_ns):
        assert wall_ns == 12345
        return {'clock_checked': True}
    def topic_status(now):
        unlocked()
        return {'hz': 100}
    bridge.ownership.snapshot = ownership
    bridge._legacy_enabled = True
    bridge._boundary = SimpleNamespace(
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=clock_now)),
        status=boundary_status)
    bridge._trackers['test'] = SimpleNamespace(status=topic_status)
    result = bridge.status()
    assert result['legacy_control']['clock_checked']
    assert result['topic_health']['test']['hz'] == 100
