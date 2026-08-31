from __future__ import annotations

import copy
import math
import threading
import time
from collections import deque
from typing import Any


class StreamMetric:
    """Bounded, monotonic stream statistics independent of ROS time."""

    def __init__(self, window_seconds: float = 2.0, stale_seconds: float = 0.5):
        if not math.isfinite(window_seconds) or window_seconds <= 0.0:
            raise ValueError("window_seconds must be positive")
        if not math.isfinite(stale_seconds) or stale_seconds <= 0.0:
            raise ValueError("stale_seconds must be positive")
        self.window_seconds = float(window_seconds)
        self.stale_seconds = float(stale_seconds)
        self.timestamps: deque[float] = deque(maxlen=1000)
        self.total = 0
        self.last_at: float | None = None
        self.max_gap_seconds = 0.0

    def observe(self, observed_at: float) -> None:
        if not math.isfinite(observed_at):
            return
        if self.last_at is not None and observed_at >= self.last_at:
            self.max_gap_seconds = max(self.max_gap_seconds, observed_at - self.last_at)
        self.last_at = observed_at
        self.timestamps.append(observed_at)
        self.total += 1
        self._trim(observed_at)

    def snapshot(self, now: float) -> dict[str, Any]:
        self._trim(now)
        age = None if self.last_at is None else max(0.0, now - self.last_at)
        rate = 0.0
        if len(self.timestamps) >= 2:
            span = self.timestamps[-1] - self.timestamps[0]
            if span > 0.0:
                rate = (len(self.timestamps) - 1) / span
        return {
            "hz": round(rate, 1),
            "age_ms": None if age is None else round(age * 1000.0, 1),
            "max_gap_ms": round(self.max_gap_seconds * 1000.0, 1),
            "total": self.total,
            "stale": age is None or age > self.stale_seconds,
        }

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_seconds
        while len(self.timestamps) > 1 and self.timestamps[0] < cutoff:
            self.timestamps.popleft()


class DashboardModel:
    """Thread-safe state shared by the ROS executor and HTTP server."""

    STREAMS = {
        "vr": (2.0, 0.25),
        "joints": (2.0, 0.25),
        "cartesian_targets": (2.0, 0.35),
        "backend_candidates": (2.0, 0.35),
        "commands": (2.0, 0.35),
    }

    def __init__(self, robot_id: str, profile: str = "", mode: str = ""):
        if not robot_id:
            raise ValueError("robot_id must not be empty")
        self._lock = threading.RLock()
        self._started_at = time.monotonic()
        self._identity = {"robot_id": robot_id, "profile": profile, "mode": mode}
        self._streams = {
            name: StreamMetric(window, stale)
            for name, (window, stale) in self.STREAMS.items()
        }
        self._safety: dict[str, Any] = {
            "state": 0,
            "label": "UNKNOWN",
            "enabled": False,
            "fault_latched": False,
            "estop_active": False,
            "reason": "waiting for safety state",
            "active_source": "",
            "active_session": "",
        }
        self._vr: dict[str, Any] = {}
        self._joints: dict[str, Any] = {"count": 0, "values": []}
        self._cartesian: dict[str, Any] = {"groups": []}
        self._backend: dict[str, dict[str, Any]] = {}
        self._commands: dict[str, dict[str, Any]] = {}

    def observe(self, stream: str, payload: dict[str, Any] | None = None) -> None:
        now = time.monotonic()
        with self._lock:
            self._streams[stream].observe(now)
            if payload is None:
                return
            if stream == "vr":
                self._vr = copy.deepcopy(payload)
            elif stream == "joints":
                self._joints = copy.deepcopy(payload)
            elif stream == "cartesian_targets":
                self._cartesian = copy.deepcopy(payload)

    def observe_group(self, stream: str, group_name: str, payload: dict[str, Any]) -> None:
        if stream not in {"backend_candidates", "commands"}:
            raise KeyError(stream)
        now = time.monotonic()
        with self._lock:
            self._streams[stream].observe(now)
            target = self._backend if stream == "backend_candidates" else self._commands
            target[group_name] = copy.deepcopy(payload)

    def set_safety(self, payload: dict[str, Any]) -> None:
        with self._lock:
            self._safety = copy.deepcopy(payload)

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            streams = {name: metric.snapshot(now) for name, metric in self._streams.items()}
            return {
                "schema": "hc-dashboard/v1",
                "server_time_ms": int(time.time() * 1000.0),
                "uptime_sec": round(now - self._started_at, 1),
                **self._identity,
                "safety": copy.deepcopy(self._safety),
                "vr": copy.deepcopy(self._vr),
                "joints": copy.deepcopy(self._joints),
                "cartesian": copy.deepcopy(self._cartesian),
                "backend_candidates": copy.deepcopy(self._backend),
                "commands": copy.deepcopy(self._commands),
                "streams": streams,
            }
