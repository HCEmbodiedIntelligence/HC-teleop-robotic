"""ROS 话题频率标准与健康度统计跟踪器。"""
from __future__ import annotations

from collections import deque
import threading
from typing import Any

DEFAULT_TOPIC_STANDARDS: dict[str, dict[str, float]] = {
    "/status/hardware/ready": {"target_hz": 100.0, "min_hz": 10.0},
    "/observation/joint_states": {"target_hz": 100.0, "min_hz": 20.0},
    "/action/joint_states": {"target_hz": 100.0, "min_hz": 20.0},
    "/control/solver/joint_targets": {"target_hz": 100.0, "min_hz": 20.0},
    "/status/solver/joint_states": {"target_hz": 100.0, "min_hz": 20.0},
    "/control/solver/ee_targets": {"target_hz": 60.0, "min_hz": 20.0},
    "/action/ee_poses": {"target_hz": 60.0, "min_hz": 20.0},
    "/observation/ee_poses": {"target_hz": 60.0, "min_hz": 20.0},
    "/action/base/command": {"target_hz": 60.0, "min_hz": 10.0},
    "/action/left_hand/joint_states": {"target_hz": 10.0, "min_hz": 1.0},
    "/action/right_hand/joint_states": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/hands/joint_states": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/hands/triggers": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/hands/vibration_feedback": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/ee_wrenches": {"target_hz": 100.0, "min_hz": 20.0},
    "/observation/cameras/head/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/head/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/head/calibration": {"target_hz": 2.0, "min_hz": 0.2},
    "/observation/cameras/head/extrinsics": {"target_hz": 2.0, "min_hz": 0.2},
    "/observation/head/pose": {"target_hz": 60.0, "min_hz": 20.0},
    "/action/head/pose": {"target_hz": 60.0, "min_hz": 20.0},
    "/observation/cameras/overhead/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_left/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_right/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_left/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_right/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/status/teleop": {"target_hz": 10.0, "min_hz": 2.0},
    "/vrdata": {"target_hz": 60.0, "min_hz": 30.0},
    "/tf": {"target_hz": 20.0, "min_hz": 5.0},
}


class TopicHealthTracker:
    """跟踪单个已订阅 ROS 话题的消息到达频率与间隔抖动。"""

    def __init__(
        self,
        topic: str,
        msg_type: str,
        target_hz: float = 0.0,
        min_hz: float = 0.0,
        record_enabled: bool = False,
    ):
        self.topic = topic
        self.msg_type = msg_type
        default_std = DEFAULT_TOPIC_STANDARDS.get(
            topic, {"target_hz": 10.0, "min_hz": 1.0}
        )
        self.target_hz = float(target_hz or default_std["target_hz"])
        self.min_hz = float(min_hz or default_std["min_hz"])
        self.record_enabled = record_enabled
        self.messages = 0
        self.last_stamp = 0.0
        self._timestamps: deque[float] = deque(maxlen=40)
        self._lock = threading.Lock()

    def record_message(self, now: float) -> None:
        with self._lock:
            self.messages += 1
            self.last_stamp = now
            self._timestamps.append(now)

    def compute_hz(self, now: float) -> float:
        with self._lock:
            if not self._timestamps or (now - self.last_stamp > 1.8):
                return 0.0
            if len(self._timestamps) < 2:
                return 0.0
            dt = self._timestamps[-1] - self._timestamps[0]
            if dt <= 0.0001:
                return 0.0
            return (len(self._timestamps) - 1) / dt

    def status(self, now: float) -> dict[str, Any]:
        with self._lock:
            messages = self.messages
            last_stamp = self.last_stamp
            timestamps = list(self._timestamps)
            timestamps_len = len(timestamps)
            dt = (timestamps[-1] - timestamps[0]) if timestamps_len >= 2 else 0.0

        if not timestamps_len or (now - last_stamp > 1.8):
            hz = 0.0
        elif timestamps_len < 2 or dt <= 0.0001:
            hz = 0.0
        else:
            hz = round((timestamps_len - 1) / dt, 1)

        has_data = messages > 0 and (now - last_stamp <= 1.8)
        if not has_data:
            state = "no_data"
            message = "未检测到消息发布 (0 Hz)"
        elif self.min_hz > 0 and hz < self.min_hz:
            state = "low_rate"
            message = f"频率偏低 ({hz:.1f} Hz < 标准 {self.min_hz:.1f} Hz)"
        else:
            state = "ok"
            message = f"正常 ({hz:.1f} Hz)"

        intervals = [
            later - earlier for earlier, later in zip(timestamps, timestamps[1:])
        ]
        max_gap_ms = round(max(intervals) * 1000.0, 1) if intervals else None
        if intervals:
            ordered = sorted(intervals)
            p95_gap_ms = round(ordered[int((len(ordered) - 1) * 0.95)] * 1000.0, 1)
        else:
            p95_gap_ms = None

        return {
            "topic": self.topic,
            "type": self.msg_type,
            "messages": messages,
            "hz": hz,
            "target_hz": self.target_hz,
            "min_hz": self.min_hz,
            "record_enabled": self.record_enabled,
            "has_data": has_data,
            "state": state,
            "message": message,
            "last_received_age": round(now - last_stamp, 2) if last_stamp > 0 else None,
            "max_gap_ms": max_gap_ms,
            "p95_gap_ms": p95_gap_ms,
        }

