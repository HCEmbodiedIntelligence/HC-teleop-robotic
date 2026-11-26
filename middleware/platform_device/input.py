"""Wire conversion and monotonic-time admission on the receiving device."""

import math
import time
from collections import deque

from middleware.core.protocol import PACKET_SIZE, PACKET_FORMAT
from struct import Struct

WIRE = Struct(PACKET_FORMAT)
assert WIRE.size == PACKET_SIZE


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("non-finite sample")
    return value


def vector(value, n, lower, upper):
    if not isinstance(value, list) or len(value) != n:
        raise ValueError("vector dimensions")
    if not all(lower <= number(v) <= upper for v in value):
        raise ValueError("component out of range")
    return value


def pose(value):
    xyz = vector(value["position"], 3, -1000, 1000)
    quat = vector(value["orientation"], 4, -1.001, 1.001)
    if not 0.98 <= sum(v * v for v in quat) <= 1.02:
        raise ValueError("invalid orientation")
    return [*xyz, *quat]


def unpack_input(msg, profile, reflect_z=False):
    hands = msg["controllers"]
    if not isinstance(hands, list) or len(hands) != 2:
        raise ValueError("both hands required")
    by_hand = {h["hand"]: h for h in hands}
    poses = pose(msg["head"])
    controls = []
    for hand in ("left", "right"):
        h = by_hand[hand]
        if profile == "hc-pico-v2" and h.get("profile") != profile:
            raise ValueError("wrong input profile")
        poses.extend(pose(h["grip"]))
        buttons = h["buttons"]
        axes = h["axes"]
        if not isinstance(buttons, list) or not 2 <= len(buttons) <= 32:
            raise ValueError("invalid gamepad buttons")
        if not isinstance(axes, list) or len(axes) not in (0, 2, 4):
            raise ValueError("invalid gamepad axes")
        values = vector(buttons, len(buttons), 0, 1) + [0] * 6
        axes = vector(axes, len(axes), -1, 1)
        primary = axes[2:4] if len(axes) == 4 else (axes or [0, 0])
        secondary = axes[:2] if len(axes) == 4 else [0, 0]
        mask = sum(
            (1 << bit)
            for index, bit in ((0, 3), (1, 2), (3, 5), (4, 0), (5, 1))
            if values[index] > 0.5
        )
        controls.append((mask, values[0], values[1], *primary, *secondary))
    if reflect_z and profile == "webxr":
        for start in (0, 7, 14):
            for index in (2, 3, 4):
                poses[start + index] = -poses[start + index]
    return poses, controls


class Quantiles:
    def __init__(self):
        self.values = deque(maxlen=4096)

    def add(self, value):
        self.values.append(value)

    def report(self):
        data = sorted(self.values)
        if not data:
            return {"count": 0}
        return {
            "count": len(data),
            **{
                key: round(data[math.ceil(len(data) * q) - 1], 3)
                for key, q in (("p50", 0.5), ("p95", 0.95), ("p99", 0.99), ("max", 1))
            },
        }


class InputWindow:
    """The first sample is sent after our enable ACK. This bounds its send time
    on our clock without assuming synchronized browser/device wall clocks."""

    def __init__(self, max_lag_ms, watchdog_ms, now=time.monotonic):
        self.limit = max_lag_ms
        self.watchdog = watchdog_ms / 1000
        self.now = now
        self.age = Quantiles()
        self.handling = Quantiles()
        self.gaps = Quantiles()
        self.accepted = self.skipped = self.reordered = self.expired = 0
        self.last_values = None
        self.wire_seq = 0
        self.enabled = False
        self.anchor = None
        self.prev_stamp = None
        self.prev_seq = -1

    def arm(self):
        self.start = self.last = self.now()
        self.base_stamp = None
        self.masks = None
        self.enabled = True

    def expire(self):
        return self.enabled and self.now() - self.last >= self.watchdog

    def sample(self, msg, profile, reflect_z=False):
        now = self.now()
        if not self.enabled:
            return None
        if now - self.last >= self.watchdog:
            raise TimeoutError("input watchdog")
        seq = msg["seq"]
        stamp = number(msg["time_ms"])
        if type(seq) is not int or not 0 <= seq < 2**53 or stamp < 0:
            raise ValueError("invalid sequence or timestamp")
        if seq <= self.prev_seq or (
            self.prev_stamp is not None and stamp <= self.prev_stamp
        ):
            self.reordered += 1
            return None
        lower = self.start
        if self.base_stamp is None and self.anchor is not None:
            # The unordered motion channel can deliver old frames after a second
            # enable. The previous epoch bounds the sender clock on our monotonic
            # clock. Wait for a provably post-enable frame before rebasing.
            local_start, sender_stamp = self.anchor
            earliest = local_start + (stamp - sender_stamp) * 0.9998 / 1000
            if earliest > now + 0.002:
                self.expired += 1
                raise TimeoutError("input clock ahead of receiver")
            if earliest < self.start:
                self.reordered += 1
                return None
            lower = earliest
        elapsed = (
            now - (self.anchor[0] if self.base_stamp is not None else lower)
        ) * 1000
        delta = stamp - self.base_stamp if self.base_stamp is not None else 0
        # Guard 200 ppm oscillator drift. A future stamp cannot refresh the lease.
        upper = elapsed - delta + 0.0002 * delta
        if upper < -2 or upper > self.limit:
            self.expired += 1
            raise TimeoutError("input latency exceeds budget")
        poses, controls = unpack_input(msg, profile, reflect_z)
        packed = []
        for idx, (mask, *analog) in enumerate(controls):
            old = self.masks[idx] if self.masks is not None else mask
            packed.extend((mask, mask & ~old, old & ~mask, *analog))
        self.wire_seq = (self.wire_seq + 1) & 0xFFFFFFFF
        data = WIRE.pack(b"PICO", 2, self.wire_seq, stamp / 1000, 7, *poses, *packed)
        if self.prev_seq != -1:
            self.skipped += seq - self.prev_seq - 1
            self.gaps.add((now - self.last) * 1000)
        if self.base_stamp is None:
            self.base_stamp = stamp
            self.anchor = (lower, stamp)
        self.masks = [v[0] for v in controls]
        self.prev_stamp, self.prev_seq, self.last = stamp, seq, now
        self.last_values = {
            "seq": seq,
            "left_trigger": controls[0][1],
            "right_trigger": controls[1][1],
        }
        self.accepted += 1
        self.age.add(max(0, upper))
        return data
