#!/usr/bin/env python3
"""Record synchronized VR, end-effector and arm-joint diagnostics to CSV."""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String


SIDES = ("right", "left")
POSE_FIELDS = ("px", "py", "pz", "qx", "qy", "qz", "qw")
ARM_JOINTS = tuple(
    f"Joint{index}_{suffix}"
    for suffix in ("R", "L")
    for index in range(1, 8)
)


def pose_values(pose: Any) -> tuple[float, ...]:
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


class TeleopDiagnosticsNode(Node):
    def __init__(self, output_path: Path, rate_hz: float) -> None:
        super().__init__("hc_tj_teleop_diagnostics")
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=self._field_names())
        self.writer.writeheader()
        self.started = time.monotonic()
        self.last_flush = self.started
        self.sequence = 0
        self.status: dict[str, Any] = {}
        self.samples: dict[str, tuple[float, int, Any]] = {}
        self.source_sequences: dict[str, int] = {}

        for side in SIDES:
            self.create_subscription(
                PoseStamped,
                f"/vr/{side}_controller_pose",
                lambda message, current_side=side: self._controller_callback(
                    current_side, message
                ),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            PoseArray, "/hc_teleop/target_ee_poses", self._target_callback, 10
        )
        self.create_subscription(
            PoseArray, "/hc_teleop/actual_ee_poses", self._actual_callback, 10
        )
        self.create_subscription(
            JointState, "/hc_teleop/joint_states", self._joint_state_callback, 10
        )
        self.create_subscription(
            JointState, "/hc_teleop/joint_cmd", self._joint_command_callback, 10
        )
        self.create_subscription(String, "/teleop/arm/status", self._status_callback, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._record)
        self.get_logger().info(f"teleop diagnostics recording to {self.output_path}")

    @staticmethod
    def _field_names() -> list[str]:
        fields = [
            "sample",
            "elapsed_s",
            "ros_time_s",
            "mode",
            "enabled",
            "feedback_fresh",
            "left_clutch",
            "right_clutch",
        ]
        for source in ("controller", "target", "actual"):
            for side in SIDES:
                prefix = f"{source}_{side}"
                fields.extend((f"{prefix}_seq", f"{prefix}_age_ms"))
                fields.extend(f"{prefix}_{field}" for field in POSE_FIELDS)
        for source in ("joint_state", "joint_command"):
            fields.extend((f"{source}_seq", f"{source}_age_ms"))
            fields.extend(f"{source}_{name}" for name in ARM_JOINTS)
        fields.extend(f"joint_error_{name}" for name in ARM_JOINTS)
        for side in SIDES:
            fields.extend(
                (
                    f"ik_{side}_converged",
                    f"ik_{side}_rejections",
                    f"ik_{side}_rejection_total",
                    f"ik_{side}_seed",
                    f"ik_{side}_damping",
                    f"ik_{side}_position_error",
                    f"ik_{side}_orientation_error",
                )
            )
        return fields

    def _store(self, key: str, value: Any) -> None:
        sequence = self.source_sequences.get(key, 0) + 1
        self.source_sequences[key] = sequence
        self.samples[key] = (time.monotonic(), sequence, value)

    def _controller_callback(self, side: str, message: PoseStamped) -> None:
        self._store(f"controller_{side}", pose_values(message.pose))

    def _pose_array_callback(self, source: str, message: PoseArray) -> None:
        for side, pose in zip(SIDES, message.poses):
            self._store(f"{source}_{side}", pose_values(pose))

    def _target_callback(self, message: PoseArray) -> None:
        self._pose_array_callback("target", message)

    def _actual_callback(self, message: PoseArray) -> None:
        self._pose_array_callback("actual", message)

    def _joint_callback(self, source: str, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        self._store(
            source,
            {
                name: float(values[name])
                for name in ARM_JOINTS
                if name in values
            },
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self._joint_callback("joint_state", message)

    def _joint_command_callback(self, message: JointState) -> None:
        self._joint_callback("joint_command", message)

    def _status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(value, dict):
            self.status = value

    def _add_sample(
        self, row: dict[str, Any], key: str, fields: tuple[str, ...]
    ) -> Any | None:
        sample = self.samples.get(key)
        if sample is None:
            return None
        stamp, sequence, value = sample
        row[f"{key}_seq"] = sequence
        row[f"{key}_age_ms"] = (time.monotonic() - stamp) * 1000.0
        if isinstance(value, dict):
            for field in fields:
                if field in value:
                    row[f"{key}_{field}"] = value[field]
        else:
            for field, item in zip(fields, value):
                row[f"{key}_{field}"] = item
        return value

    def _record(self) -> None:
        now = time.monotonic()
        self.sequence += 1
        row: dict[str, Any] = {
            "sample": self.sequence,
            "elapsed_s": now - self.started,
            "ros_time_s": self.get_clock().now().nanoseconds / 1e9,
            "mode": self.status.get("mode", ""),
            "enabled": int(bool(self.status.get("enabled", False))),
            "feedback_fresh": int(bool(self.status.get("feedback_fresh", False))),
            "left_clutch": int(bool(self.status.get("left_clutch", False))),
            "right_clutch": int(bool(self.status.get("right_clutch", False))),
        }
        for source in ("controller", "target", "actual"):
            for side in SIDES:
                self._add_sample(row, f"{source}_{side}", POSE_FIELDS)
        states = self._add_sample(row, "joint_state", ARM_JOINTS)
        commands = self._add_sample(row, "joint_command", ARM_JOINTS)
        if isinstance(states, dict) and isinstance(commands, dict):
            for name in ARM_JOINTS:
                if name in states and name in commands:
                    row[f"joint_error_{name}"] = commands[name] - states[name]
        arm_status = self.status.get("arms", {})
        for side in SIDES:
            status = arm_status.get(side, {}) if isinstance(arm_status, dict) else {}
            row[f"ik_{side}_converged"] = int(
                bool(status.get("ik_converged", False))
            )
            status_fields = {
                "rejections": "ik_rejections",
                "rejection_total": "ik_rejection_total",
                "seed": "ik_seed",
                "damping": "ik_damping",
                "position_error": "position_error",
                "orientation_error": "orientation_error",
            }
            for output_field, status_field in status_fields.items():
                if status_field in status:
                    row[f"ik_{side}_{output_field}"] = status[status_field]
        self.writer.writerow(row)
        if now - self.last_flush >= 1.0:
            self.stream.flush()
            self.last_flush = now

    def close(self) -> None:
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=30.0)
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")

    rclpy.init(args=[])
    node = TeleopDiagnosticsNode(args.output.expanduser().resolve(), args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"Teleop diagnostics saved: {node.output_path}")


if __name__ == "__main__":
    main()
