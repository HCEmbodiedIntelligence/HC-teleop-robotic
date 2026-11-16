"""Exact wire compatibility with HC-Teleop Assets/UdpPoseSender.cs.

The deployed v1/v2 packets carry no cryptographic identity or session token.
Network isolation is therefore part of deploying this compatibility endpoint.
"""

from dataclasses import dataclass
import math
import struct
from typing import Optional, Tuple

V1 = struct.Struct("<4sBIdB21f")
V2 = struct.Struct("<4sBIdB21f3H6f3H6f")
DISCOVER = b"PICO_DISCOVER_V1"
RELEASE = b"PICO_RELEASE_V1"
GRIP = 1 << 2
PRIMARY = 1 << 0
SECONDARY = 1 << 1
KNOWN_BUTTONS = (1 << 11) - 1


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Pose:
    position: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]

    def as_list(self):
        return list(self.position + self.quaternion)


@dataclass(frozen=True)
class ControllerInput:
    held: int = 0
    pressed: int = 0
    released: int = 0
    trigger: float = 0.0
    grip: float = 0.0
    primary_axis: Tuple[float, float] = (0.0, 0.0)
    secondary_axis: Tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class VrFrame:
    version: int
    sequence: int
    sample_time: float
    tracking_flags: int
    head: Pose
    left: Pose
    right: Pose
    left_input: ControllerInput
    right_input: ControllerInput

    def tracked(self, side):
        return bool(self.tracking_flags & {"head": 1, "left": 2, "right": 4}[side])


def is_newer(sequence: int, previous: Optional[int]) -> bool:
    """RFC-1982 style uint32 ordering, including the sender's wraparound."""
    return previous is None or 0 < ((sequence - previous) & 0xFFFFFFFF) < 0x80000000


def _pose(values, tracked):
    p = tuple(float(x) for x in values[:3])
    q = tuple(float(x) for x in values[3:7])
    norm = math.sqrt(sum(x * x for x in q))
    if tracked and not 0.5 <= norm <= 1.5:
        raise ProtocolError("invalid tracked quaternion")
    # Untracked samples may legitimately carry an all-zero quaternion.
    q = tuple(x / norm for x in q) if norm > 1e-9 else (0.0, 0.0, 0.0, 1.0)
    return Pose(p, q)


def _input(values):
    held, pressed, released = values[:3]
    if (held | pressed | released) & ~KNOWN_BUTTONS:
        raise ProtocolError("unsupported button bits")
    trigger, grip, ax, ay, bx, by = map(float, values[3:])
    if not all(-1e-6 <= x <= 1.000001 for x in (trigger, grip)):
        raise ProtocolError("trigger/grip outside [0, 1]")
    if not all(-1.000001 <= x <= 1.000001 for x in (ax, ay, bx, by)):
        raise ProtocolError("axis outside [-1, 1]")
    return ControllerInput(
        held,
        pressed,
        released,
        max(0.0, min(1.0, trigger)),
        max(0.0, min(1.0, grip)),
        (ax, ay),
        (bx, by),
    )


def decode_packet(packet: bytes) -> VrFrame:
    codec = {V1.size: V1, V2.size: V2}.get(len(packet))
    if codec is None:
        raise ProtocolError("expected 102-byte v1 or 162-byte v2 packet")
    data = codec.unpack(packet)
    magic, version, sequence, timestamp, flags = data[:5]
    if magic != b"PICO" or version != (1 if codec is V1 else 2):
        raise ProtocolError("packet magic/version/size mismatch")
    if flags & ~7 or timestamp < 0 or not math.isfinite(timestamp):
        raise ProtocolError("invalid tracking flags or timestamp")
    values = data[5:]
    if not all(math.isfinite(x) for x in values):
        raise ProtocolError("non-finite pose or input")
    poses = [
        _pose(values[i * 7 : (i + 1) * 7], bool(flags & (1 << i))) for i in range(3)
    ]
    inputs = (
        [_input(values[21:30]), _input(values[30:39])]
        if version == 2
        else [ControllerInput(), ControllerInput()]
    )
    return VrFrame(version, sequence, timestamp, flags, *poses, *inputs)
