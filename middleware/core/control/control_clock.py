"""Fresh control timestamps without native ROS clock allocation on system time.

An inactive ROS clock has the same CLOCK_REALTIME epoch as ``time.time_ns``.
Clock-change callbacks disable that fast path before a simulated-time change;
active or unknown clocks always use the real ROS clock. No timestamp is cached.
"""
import threading
import time


class ControlClock:
    def __init__(self, clock):
        from rclpy.clock import ClockType, ClockChange, JumpThreshold

        self.clock = clock
        self._lock = threading.Lock()
        self._system_time = clock.clock_type == ClockType.SYSTEM_TIME
        self._jump_handle = None
        self._changes = 0
        self._changing = False
        if clock.clock_type == ClockType.ROS_TIME:
            def changing():
                # The native activation must wait for any system-time read
                # already in progress before changing the ROS clock's epoch.
                with self._lock:
                    self._changes += 1
                    self._changing = True
                    self._system_time = False

            def changed(jump):
                # A native clock read is required throughout activation and
                # while simulated time is active, including paused /clock.
                with self._lock:
                    self._changes += 1
                    self._changing = False
                    self._system_time = jump.clock_change == ClockChange.ROS_TIME_DEACTIVATED

            self._jump_handle = clock.create_jump_callback(
                JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True),
                pre_callback=changing, post_callback=changed)
            with self._lock:
                changes = self._changes
            # Native reads can wait for a clock-change callback. Never hold
            # the Python lock while entering the native ROS clock.
            active = clock.ros_time_is_active
            with self._lock:
                if changes == self._changes and not self._changing:
                    self._system_time = not active

    def now_ns(self):
        with self._lock:
            if self._system_time:
                return time.time_ns()
        return self.clock.now().nanoseconds

    def close(self):
        handle, self._jump_handle = self._jump_handle, None
        if handle is not None:
            handle.unregister()
