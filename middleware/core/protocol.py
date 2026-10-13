from __future__ import annotations

import json
import math
import struct
import time
from dataclasses import dataclass
from typing import Any


DISCOVERY_REQUEST = b"PICO_DISCOVER_V1"
# v2 adds held/pressed/released masks and six analog values per controller.
LEGACY_PACKET_FORMAT = "<4sBIdB21f"
LEGACY_PACKET_SIZE = struct.calcsize(LEGACY_PACKET_FORMAT)
PACKET_FORMAT = "<4sBIdB21f3H6f3H6f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
MAGIC = b"PICO"
PROTOCOL_VERSION = 2

HEAD_TRACKED_FLAG = 1
LEFT_TRACKED_FLAG = 2
RIGHT_TRACKED_FLAG = 4

BUTTON_NAMES = {
    1 << 0: "primary",
    1 << 1: "secondary",
    1 << 2: "grip_button",
    1 << 3: "trigger_button",
    1 << 4: "menu",
    1 << 5: "primary_axis_click",
    1 << 6: "primary_axis_touch",
    1 << 7: "secondary_axis_click",
    1 << 8: "secondary_axis_touch",
    1 << 9: "primary_touch",
    1 << 10: "secondary_touch",
}


class PacketError(ValueError):
    pass


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]

    def as_dict(self) -> dict[str, list[float]]:
        return {
            "position": list(self.position),
            "quaternion": list(self.quaternion),
        }


@dataclass(frozen=True)
class ControllerInput:
    held_mask: int = 0
    pressed_mask: int = 0
    released_mask: int = 0
    trigger: float = 0.0
    grip: float = 0.0
    primary_axis: tuple[float, float] = (0.0, 0.0)
    secondary_axis: tuple[float, float] = (0.0, 0.0)

    @staticmethod
    def decode_buttons(mask: int) -> list[str]:
        return [name for bit, name in BUTTON_NAMES.items() if mask & bit]

    def as_dict(self) -> dict[str, Any]:
        return {
            "held_mask": self.held_mask,
            "pressed_mask": self.pressed_mask,
            "released_mask": self.released_mask,
            "held": self.decode_buttons(self.held_mask),
            "pressed": self.decode_buttons(self.pressed_mask),
            "released": self.decode_buttons(self.released_mask),
            "trigger": self.trigger,
            "grip": self.grip,
            "primary_axis": list(self.primary_axis),
            "secondary_axis": list(self.secondary_axis),
        }

    def joy_axes(self) -> list[float]:
        return [
            self.trigger,
            self.grip,
            *self.primary_axis,
            *self.secondary_axis,
        ]

    def joy_buttons(self) -> list[int]:
        return [int(bool(self.held_mask & (1 << index))) for index in range(11)]


@dataclass(frozen=True)
class PosePacket:
    protocol_version: int
    sequence: int
    vr_timestamp: float
    flags: int
    head: Pose
    left: Pose
    right: Pose
    left_input: ControllerInput = ControllerInput()
    right_input: ControllerInput = ControllerInput()

    def tracked(self, name: str) -> bool:
        masks = {
            "head": HEAD_TRACKED_FLAG,
            "left": LEFT_TRACKED_FLAG,
            "right": RIGHT_TRACKED_FLAG,
        }
        return bool(self.flags & masks[name])

    def as_dict(self) -> dict[str, Any]:
        return {
            "protocol_version": self.protocol_version,
            "sequence": self.sequence,
            "vr_timestamp": self.vr_timestamp,
            "tracking": {
                "head": self.tracked("head"),
                "left": self.tracked("left"),
                "right": self.tracked("right"),
            },
            "poses": {
                "head": self.head.as_dict(),
                "left": self.left.as_dict(),
                "right": self.right.as_dict(),
            },
            "inputs": {
                "left": self.left_input.as_dict(),
                "right": self.right_input.as_dict(),
            },
        }


def _pose(values: tuple[float, ...], offset: int) -> Pose:
    return Pose(
        position=tuple(values[offset : offset + 3]),  # type: ignore[arg-type]
        quaternion=tuple(values[offset + 3 : offset + 7]),  # type: ignore[arg-type]
    )


def _controller_input(values: tuple[Any, ...], offset: int) -> ControllerInput:
    return ControllerInput(
        held_mask=values[offset],
        pressed_mask=values[offset + 1],
        released_mask=values[offset + 2],
        trigger=values[offset + 3],
        grip=values[offset + 4],
        primary_axis=(values[offset + 5], values[offset + 6]),
        secondary_axis=(values[offset + 7], values[offset + 8]),
    )


def decode_pose_packet(packet: bytes) -> PosePacket:
    formats = {
        LEGACY_PACKET_SIZE: (LEGACY_PACKET_FORMAT, 1),
        PACKET_SIZE: (PACKET_FORMAT, PROTOCOL_VERSION),
    }
    if len(packet) not in formats:
        raise PacketError(
            f"expected {LEGACY_PACKET_SIZE} (v1) or {PACKET_SIZE} (v2) bytes, "
            f"received {len(packet)}"
        )
    packet_format, expected_version = formats[len(packet)]
    try:
        unpacked = struct.unpack(packet_format, packet)
    except struct.error as exc:
        raise PacketError(str(exc)) from exc
    magic, version, sequence, vr_timestamp, flags = unpacked[:5]
    if magic != MAGIC:
        raise PacketError("invalid packet magic")
    if version != expected_version:
        raise PacketError(f"unsupported protocol version: {version}")
    values = unpacked[5:]
    if not all(math.isfinite(value) for value in values) or not math.isfinite(vr_timestamp):
        raise PacketError("packet contains a non-finite number")
    return PosePacket(
        protocol_version=version,
        sequence=sequence,
        vr_timestamp=vr_timestamp,
        flags=flags,
        head=_pose(values, 0),
        left=_pose(values, 7),
        right=_pose(values, 14),
        left_input=_controller_input(values, 21) if version >= 2 else ControllerInput(),
        right_input=_controller_input(values, 30) if version >= 2 else ControllerInput(),
    )


def is_sequence_newer(sequence: int, previous: int | None) -> bool:
    if previous is None:
        return True
    difference = (sequence - previous) & 0xFFFFFFFF
    return 0 < difference < 0x80000000


def packet_loss_count(sequence: int, previous: int | None) -> int:
    if previous is None:
        return 0
    difference = (sequence - previous) & 0xFFFFFFFF
    return difference - 1 if 1 < difference < 0x80000000 else 0


def envelope(kind: str, source: str, payload: Any, **metadata: Any) -> dict[str, Any]:
    result = {
        "version": 1,
        "kind": kind,
        "source": source,
        "timestamp": time.time(),
        "payload": payload,
    }
    result.update(metadata)
    return result


def encode_json_packet(value: dict[str, Any]) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
