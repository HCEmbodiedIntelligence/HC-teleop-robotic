"""Smooth velocity/acceleration-bounded alignment to replay starting poses."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any


def step_trapezoidal_axis(
    error: float,
    old_velocity: float,
    max_velocity: float,
    max_acceleration: float,
    dt: float,
) -> tuple[bool, float, float]:
    """Advance one axis with discrete braking to reach error=0 smoothly.

    Returns ``(settled, step_delta, new_velocity)`` where ``settled`` means the
    caller should snap this axis to its exact target with zero velocity.
    """
    stopping_velocity = max(
        0.0,
        math.sqrt(max(0.0, 2.0 * max_acceleration * abs(error)))
        - max_acceleration * dt,
    )
    desired_velocity = (
        math.copysign(min(max_velocity, stopping_velocity), error) if error else 0.0
    )
    max_dv = max_acceleration * dt
    dv = max(-max_dv, min(max_dv, desired_velocity - old_velocity))
    new_velocity = old_velocity + dv
    step = 0.5 * (old_velocity + new_velocity) * dt
    settled_distance = 0.5 * max_acceleration * dt * dt
    if (
        not error
        or abs(step) >= abs(error)
        or step * error < 0.0
        or (abs(error) <= settled_distance and abs(old_velocity) <= max_dv)
    ):
        return True, error, 0.0
    return False, step, new_velocity


@dataclass
class HandAlignmentTrack:
    """Open-loop smooth ramp state for one hand gripper/finger topic."""

    side: str
    topic: str
    joint_names: tuple[str, ...]
    goal: dict[str, float]
    current: dict[str, float]
    extras: dict[str, float] = field(default_factory=dict)
    velocity: dict[str, float] = field(default_factory=dict)

    def max_remaining(self) -> float:
        return max(
            (abs(self.goal[n] - self.current[n]) for n in self.goal),
            default=0.0,
        )

    def step(self, max_velocity: float, max_acceleration: float, dt: float) -> bool:
        done = True
        for name, goal_val in self.goal.items():
            cur_val = self.current[name]
            settled, delta, new_vel = step_trapezoidal_axis(
                goal_val - cur_val,
                self.velocity.get(name, 0.0),
                max_velocity,
                max_acceleration,
                dt,
            )
            if settled:
                self.current[name] = goal_val
                self.velocity[name] = 0.0
            else:
                self.current[name] = cur_val + delta
                self.velocity[name] = new_vel
                done = False
        return done


class ReplayAlignmentPlanner:
    """Stateful 100 Hz smooth alignment trajectory for replay start poses."""

    def __init__(
        self,
        *,
        epoch: int,
        arm_topic: str,
        arm_joint_names: tuple[str, ...],
        goal_positions: dict[str, float],
        align_joints: tuple[str, ...],
        start_positions: dict[str, float],
        hand_tracks: dict[str, HandAlignmentTrack] | None = None,
        max_velocity: float = 0.5,
        max_acceleration: float = 1.0,
        hand_velocity: float = 2.5,
        start_tolerance: float = 0.03,
        settle_tolerance: float = 0.05,
        max_tracking_error: float = 0.30,
        settle_timeout: float = 2.0,
        now_mono: float = 0.0,
    ) -> None:
        self.epoch = epoch
        self.arm_topic = arm_topic
        self.arm_joint_names = tuple(arm_joint_names)
        self.goal_positions = dict(goal_positions)
        self.align_joints = tuple(align_joints)
        self.hand_tracks = dict(hand_tracks or {})
        self.max_velocity = max(0.05, float(max_velocity))
        self.max_acceleration = max(0.05, float(max_acceleration))
        self.hand_velocity = max(0.1, float(hand_velocity))
        self.hand_acceleration = max(4.0, self.hand_velocity * 3.0)
        self.start_tolerance = max(1e-4, float(start_tolerance))
        self.settle_tolerance = max(self.start_tolerance, float(settle_tolerance))
        self.max_tracking_error = max(0.05, float(max_tracking_error))
        self.settle_timeout = max(0.2, float(settle_timeout))

        # Initialize trajectory start from measured feedback on active aligned
        # axes; read-only/non-align placeholders keep goal values directly.
        self.current: dict[str, float] = {}
        for name in self.arm_joint_names:
            if name in self.align_joints:
                self.current[name] = float(start_positions[name])
            else:
                self.current[name] = float(self.goal_positions.get(name, math.nan))
        self.velocity: dict[str, float] = dict.fromkeys(self.align_joints, 0.0)

        self.max_initial_diff = max(
            (abs(self.current[n] - self.goal_positions[n]) for n in self.align_joints),
            default=0.0,
        )
        self.max_hand_diff = max(
            (track.max_remaining() for track in self.hand_tracks.values()),
            default=0.0,
        )
        self.needs_move = (
            self.max_initial_diff > self.start_tolerance
            or self.max_hand_diff > max(0.05, self.start_tolerance)
        )
        self.state = "aligning" if self.needs_move else "aligned"
        self.error = ""
        self.last_tick_mono = now_mono
        self.next_tick_mono = now_mono
        self.reached_mono: float | None = None
        est_traj_sec = (
            self.max_initial_diff / self.max_velocity
            + self.max_velocity / self.max_acceleration
        )
        self.deadline_mono = now_mono + max(15.0, est_traj_sec * 3.0 + self.settle_timeout + 3.0)

    def status_dict(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "error": self.error,
            "max_diff_rad": round(self.max_initial_diff, 4),
        }

    def wait_sec(self, now_mono: float, default_wait: float) -> float:
        if self.state not in {"aligning", "paused"}:
            return default_wait
        return min(default_wait, max(0.0, self.next_tick_mono - now_mono))

    def pause(self) -> None:
        if self.state == "aligning":
            self.state = "paused"
            for name in self.velocity:
                self.velocity[name] = 0.0
            for track in self.hand_tracks.values():
                track.velocity.clear()

    def resume(self, now_mono: float) -> None:
        if self.state == "paused":
            self.state = "aligning"
            self.last_tick_mono = now_mono
            self.next_tick_mono = now_mono

    def abort(self, reason: str) -> None:
        if self.state in {"aligning", "paused"}:
            self.state = "error"
            self.error = str(reason)

    def step(
        self,
        now_mono: float,
        measured: dict[str, float],
        *,
        feedback_fresh: bool,
        replay_allowed: bool,
    ) -> dict[str, Any]:
        """Compute next 100 Hz alignment command and updated convergence state."""
        if self.state not in {"aligning", "paused"}:
            return {"publish": False, "state": self.state, "error": self.error}
        if not replay_allowed:
            self.abort("replay stopped or safety triggered during alignment")
            return {"publish": False, "state": self.state, "error": self.error}
        if not feedback_fresh or any(n not in measured for n in self.align_joints):
            self.abort("required feedback missing/stale during replay alignment")
            return {"publish": False, "state": self.state, "error": self.error}
        if now_mono > self.deadline_mono:
            self.abort("replay alignment timed out before reaching start pose")
            return {"publish": False, "state": self.state, "error": self.error}

        for name in self.align_joints:
            err = abs(self.current[name] - float(measured[name]))
            if err > self.max_tracking_error:
                self.abort(
                    f"target too far from measured position during alignment: "
                    f"{name} cmd={self.current[name]:.4f} "
                    f"measured={float(measured[name]):.4f} "
                    f"limit={self.max_tracking_error:.4f}"
                )
                return {"publish": False, "state": self.state, "error": self.error}

        if now_mono < self.next_tick_mono:
            return {"publish": False, "state": self.state, "error": self.error}

        self.next_tick_mono = now_mono + 0.01
        if self.state == "paused":
            return {
                "publish": True,
                "state": self.state,
                "arm_positions": [self.current[n] for n in self.arm_joint_names],
                "hand_commands": {},
                "done": False,
            }

        dt = min(0.05, max(0.002, now_mono - self.last_tick_mono))
        self.last_tick_mono = now_mono

        arm_done = True
        for name in self.align_joints:
            goal_val = self.goal_positions[name]
            cur_val = self.current[name]
            settled, delta, new_vel = step_trapezoidal_axis(
                goal_val - cur_val,
                self.velocity.get(name, 0.0),
                self.max_velocity,
                self.max_acceleration,
                dt,
            )
            if settled:
                self.current[name] = goal_val
                self.velocity[name] = 0.0
            else:
                self.current[name] = cur_val + delta
                self.velocity[name] = new_vel
                arm_done = False

        hand_commands: dict[str, tuple[tuple[str, ...], list[float]]] = {}
        hands_done = True
        for side, track in self.hand_tracks.items():
            if not track.step(self.hand_velocity, self.hand_acceleration, dt):
                hands_done = False
            values = []
            for name in track.joint_names:
                if name in track.current:
                    values.append(track.current[name])
                else:
                    values.append(track.extras.get(name, math.nan))
            hand_commands[track.topic] = (track.joint_names, values)

        done = False
        if arm_done and hands_done:
            feedback_err = max(
                (abs(float(measured[n]) - self.goal_positions[n]) for n in self.align_joints),
                default=0.0,
            )
            if feedback_err <= self.settle_tolerance:
                self.state = "aligned"
                done = True
            else:
                if self.reached_mono is None:
                    self.reached_mono = now_mono
                elif now_mono - self.reached_mono >= self.settle_timeout:
                    self.abort(
                        f"replay alignment feedback did not settle within tolerance "
                        f"({feedback_err:.4f} > {self.settle_tolerance:.4f} rad)"
                    )
                    return {"publish": False, "state": self.state, "error": self.error}

        return {
            "publish": True,
            "state": self.state,
            "arm_positions": [self.current[n] for n in self.arm_joint_names],
            "hand_commands": hand_commands,
            "done": done,
        }
