"""Control clocks keep fresh system time and follow runtime ROS-time changes."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.clock import Clock, ClockChange, ClockType, ROSClock
from rclpy.time import Time
from rclpy.time_source import TimeSource

from middleware.core.control_clock import ControlClock


class CallbackClock:
    """Controllable native-clock model whose pre callback precedes activation."""
    clock_type = ClockType.ROS_TIME

    def __init__(self, active=False):
        self.active = active
        self.on_active_read = None
        self.on_native_read = None
        self.attempted, self.changed = threading.Event(), threading.Event()

    def create_jump_callback(self, threshold, *, pre_callback, post_callback):
        self.pre, self.post = pre_callback, post_callback
        return SimpleNamespace(unregister=lambda: None)

    @property
    def ros_time_is_active(self):
        return self.on_active_read() if self.on_active_read else self.active

    def activate(self):
        self.attempted.set()
        self.pre()
        self.active = True
        self.post(SimpleNamespace(clock_change=ClockChange.ROS_TIME_ACTIVATED))
        self.changed.set()

    def now(self):
        if self.on_native_read:
            self.on_native_read()
        return SimpleNamespace(nanoseconds=7_000_000_000)


def test_activation_cannot_be_overwritten_between_initial_version_check_and_flag_write():
    clock = CallbackClock()
    activation = threading.Thread(target=clock.activate, daemon=True)

    class InactiveSnapshot:
        def __bool__(self):
            # Force activation exactly while evaluating the old inactive
            # snapshot, after the initialization generation check has passed.
            activation.start()
            assert clock.attempted.wait(1)
            assert not clock.changed.wait(.02)
            return False

    clock.on_active_read = InactiveSnapshot
    reader = ControlClock(clock)
    try:
        activation.join(1)
        assert not activation.is_alive()
        assert clock.changed.is_set()
        with patch('middleware.core.control_clock.time.time_ns', side_effect=AssertionError('stale initialization flag')):
            assert reader.now_ns() == 7_000_000_000
    finally:
        reader.close()


def test_initialization_during_an_unfinished_jump_keeps_native_reads():
    class ChangingClock(CallbackClock):
        def create_jump_callback(self, *args, **kwargs):
            handle = super().create_jump_callback(*args, **kwargs)
            self.pre()
            return handle

    # Registration can observe a pre callback whose native clock change and
    # post callback have not yet completed. An inactive snapshot at this point
    # must not reopen the fast path, even if the generation is unchanged.
    clock = ChangingClock()
    reader = ControlClock(clock)
    try:
        with patch('middleware.core.control_clock.time.time_ns', side_effect=AssertionError('clock jump still in progress')):
            assert reader.now_ns() == 7_000_000_000
        clock.active = True
        clock.post(SimpleNamespace(clock_change=ClockChange.ROS_TIME_ACTIVATED))
        assert reader.now_ns() == 7_000_000_000
    finally:
        reader.close()


def test_activation_waits_for_a_system_time_sample_already_in_progress():
    clock = CallbackClock()
    reader = ControlClock(clock)
    sampling, release = threading.Event(), threading.Event()
    samples = []

    def system_sample():
        sampling.set()
        assert release.wait(1)
        return 123

    read = threading.Thread(target=lambda: samples.append(reader.now_ns()), daemon=True)
    activation = threading.Thread(target=clock.activate, daemon=True)
    try:
        with patch('middleware.core.control_clock.time.time_ns', side_effect=system_sample):
            read.start()
            assert sampling.wait(1)
            activation.start()
            assert clock.attempted.wait(1)
            assert not clock.changed.wait(.02)
            release.set()
            read.join(1)
            activation.join(1)
        assert not read.is_alive() and not activation.is_alive()
        assert samples == [123]
        with patch('middleware.core.control_clock.time.time_ns', side_effect=AssertionError('wall time after activation')):
            assert reader.now_ns() == 7_000_000_000
    finally:
        release.set()
        read.join(1)
        if activation.ident is not None:
            activation.join(1)
        reader.close()


def test_native_clock_read_can_invoke_jump_callbacks_without_python_lock_deadlock():
    clock = CallbackClock(active=True)
    reader = ControlClock(clock)
    clock.on_native_read = clock.activate
    samples = []
    read = threading.Thread(target=lambda: samples.append(reader.now_ns()), daemon=True)
    try:
        read.start()
        read.join(1)
        assert not read.is_alive()
        assert samples == [7_000_000_000]
    finally:
        reader.close()


def test_system_clock_is_sampled_again_for_every_read_without_native_allocation():
    clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    reader = ControlClock(clock)
    with patch.object(clock, 'now', side_effect=AssertionError('unnecessary native allocation')):
        with patch('middleware.core.control_clock.time.time_ns', side_effect=[100, 101, 90]):
            assert [reader.now_ns(), reader.now_ns(), reader.now_ns()] == [100, 101, 90]
    reader.close()


def test_ros_time_activation_pauses_and_deactivation_follow_the_real_clock():
    clock, source = ROSClock(), TimeSource()
    source.attach_clock(clock)
    reader = ControlClock(clock)
    try:
        with patch('middleware.core.control_clock.time.time_ns', return_value=123):
            assert reader.now_ns() == 123
        source.ros_time_is_active = True
        clock.set_ros_time_override(Time(nanoseconds=7_000_000_000))
        with patch('middleware.core.control_clock.time.time_ns', side_effect=AssertionError('wall clock with /clock active')):
            assert reader.now_ns() == 7_000_000_000
            assert reader.now_ns() == 7_000_000_000  # Paused simulation remains paused.
            clock.set_ros_time_override(Time(nanoseconds=2_000_000_000))
            assert reader.now_ns() == 2_000_000_000  # Backward simulation jump.
        source.ros_time_is_active = False
        with patch('middleware.core.control_clock.time.time_ns', return_value=456):
            assert reader.now_ns() == 456
        source.ros_time_is_active = True
        clock.set_ros_time_override(Time(nanoseconds=9_000_000_000))
        assert reader.now_ns() == 9_000_000_000
    finally:
        reader.close()


def test_clock_active_before_reader_creation_uses_native_time():
    clock, source = ROSClock(), TimeSource()
    source.attach_clock(clock)
    source.ros_time_is_active = True
    clock.set_ros_time_override(Time(nanoseconds=42))
    reader = ControlClock(clock)
    try:
        assert reader.now_ns() == 42
    finally:
        reader.close()


def test_unknown_and_steady_clocks_keep_their_original_native_clock():
    clock = SimpleNamespace(clock_type='unknown', now=Mock(return_value=SimpleNamespace(nanoseconds=987)))
    reader = ControlClock(clock)
    assert reader.now_ns() == 987
    clock.now.assert_called_once_with()
    reader.close()
    steady = Clock(clock_type=ClockType.STEADY_TIME)
    reader = ControlClock(steady)
    before = steady.now().nanoseconds
    assert before <= reader.now_ns() <= steady.now().nanoseconds
    reader.close()
