"""Checks shared by physical and simulated last-hop adapters."""

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class X1Topics:
    command: str
    feedback: str
    ready: str = "/status/hardware/ready"
    base: str = "/action/base/command"
    left_hand: str = "/action/left_hand/joint_states"
    right_hand: str = "/action/right_hand/joint_states"

    @classmethod
    def for_contract(cls, contract):
        if contract == "legacy-model-topic":
            return cls("/action/joint_states", "/observation/joint_states")
        if contract == "internal-model-v2":
            return cls("/internal/robot/joint_commands", "/internal/robot/joint_states")
        raise ValueError("choose legacy-model-topic or internal-model-v2 explicitly")


class OutputGate:
    """Fences and absolute freshness checked again at the actual send boundary."""

    def __init__(self, allowed_names):
        self.allowed_names = frozenset(allowed_names)
        self.boot_id = None
        self.epoch = -1
        self.last_seq = -1
        self.rejected = 0
        self.reason = "AWAITING_FENCE"

    def fence(self, boot_id, epoch):
        if (
            not isinstance(boot_id, str)
            or not boot_id
            or not isinstance(epoch, int)
            or epoch < 0
        ):
            raise ValueError("invalid fence")
        if boot_id == self.boot_id and epoch < self.epoch:
            raise ValueError("old fence")
        self.boot_id, self.epoch, self.last_seq = boot_id, epoch, -1
        self.reason = "FENCED"

    def validate(self, command, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        reason = None
        names, positions = command.get("joint_names", []), command.get("positions", [])
        seq, deadline = command.get("seq"), command.get("deadline_mono_ns")
        if command.get("boot_id") != self.boot_id or command.get("epoch") != self.epoch:
            reason = "STALE_FENCE"
        elif not isinstance(seq, int) or seq <= self.last_seq:
            reason = "STALE_SEQUENCE"
        elif not isinstance(deadline, int) or deadline <= now_ns:
            reason = "EXPIRED_OUTPUT"
        elif len(names) != len(positions) or len(set(names)) != len(names):
            reason = "INVALID_JOINT_ARRAYS"
        elif any(name not in self.allowed_names for name in names):
            reason = "GROUP_NOT_ALLOWED"
        elif not all(
            isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
            for x in positions
        ):
            reason = "NONFINITE_OUTPUT"
        elif command.get("mode") not in ("position", "hold", "stop"):
            reason = "INVALID_OUTPUT_MODE"
        for key, length in (("base_velocity", 3), ("grippers", 2)):
            values = command.get(key, [0] * length)
            if (
                not isinstance(values, list)
                or len(values) != length
                or not all(
                    isinstance(x, (int, float)) and math.isfinite(x) for x in values
                )
            ):
                reason = "INVALID_" + key.upper()
        if not reason and any(not 0 <= x <= 1 for x in command.get("grippers", [0, 0])):
            reason = "INVALID_GRIPPER_RANGE"
        if reason:
            self.rejected += 1
            self.reason = reason
            return False
        self.last_seq = seq
        self.reason = "OK"
        return True


def allowed_joint_names(config, groups=None):
    allowed = set(config.get("allowed_groups", [])) if groups is None else set(groups)
    return {
        name
        for group, names in config.get("groups", {}).items()
        if group in allowed
        for name in names
    }


def map_gripper(calibration, amount):
    names = list(calibration.get("finger_joint_names", []))
    start, end = (
        calibration.get("finger_open", []),
        calibration.get("finger_closed", []),
    )
    if (
        not names
        or len(names) != len(start)
        or len(names) != len(end)
        or len(set(names)) != len(names)
    ):
        raise ValueError("invalid gripper calibration")
    if not math.isfinite(amount) or not 0 <= amount <= 1:
        raise ValueError("gripper amount outside [0,1]")
    positions = [float(a) + amount * (float(b) - float(a)) for a, b in zip(start, end)]
    if not all(math.isfinite(x) for x in positions):
        raise ValueError("invalid gripper calibration values")
    return names, positions
