from __future__ import annotations

import threading
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from hc_teleop_interfaces.msg import (
    CartesianTargetArray,
    JointCommand,
    JointCommandCandidate,
    SafetyState,
    VrFrame,
)
from hc_teleop_interfaces.srv import ResetFault, SetEnabled

from .state import DashboardModel


def _pose(value: Any) -> dict[str, float]:
    return {
        "x": round(float(value.position.x), 4),
        "y": round(float(value.position.y), 4),
        "z": round(float(value.position.z), 4),
        "qx": round(float(value.orientation.x), 4),
        "qy": round(float(value.orientation.y), 4),
        "qz": round(float(value.orientation.z), 4),
        "qw": round(float(value.orientation.w), 4),
    }


def _tracked_pose(value: Any) -> dict[str, Any]:
    return {
        "tracking_state": int(value.tracking_state),
        "tracked": int(value.tracking_state) == 2,
        "confidence": round(float(value.confidence), 3),
        "velocity_valid": bool(value.velocity_valid),
        "pose": _pose(value.pose),
    }


def _controller_input(value: Any) -> dict[str, Any]:
    return {
        "held_mask": int(value.held_mask),
        "pressed_mask": int(value.pressed_mask),
        "released_mask": int(value.released_mask),
        "grip": round(float(value.grip), 3),
        "trigger": round(float(value.trigger), 3),
        "primary_axis": [round(float(axis), 3) for axis in value.primary_axis],
        "secondary_axis": [round(float(axis), 3) for axis in value.secondary_axis],
    }


def _joint_values(message: JointState) -> list[dict[str, Any]]:
    values = []
    for index, name in enumerate(message.name):
        position = float(message.position[index]) if index < len(message.position) else None
        values.append({"name": name, "position": None if position is None else round(position, 4)})
    return values


def _diagnostic_value(value: str) -> Any:
    lowered = value.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def _diagnostic_level(value: Any) -> int:
    """Normalize ROS uint8 fields across rclpy type-support implementations."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        if len(value) != 1:
            raise ValueError("diagnostic level must contain exactly one byte")
        return int(value[0])
    return int(value)


class DashboardRosBridge(Node):
    def __init__(self) -> None:
        super().__init__("dashboard")
        self.robot_id = str(self.declare_parameter("robot_id", "robot").value)
        self.profile = str(self.declare_parameter("profile", "").value)
        self.mode = str(self.declare_parameter("mode", "").value)
        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 7877).value)
        if not self.robot_id or not self.host or not 1 <= self.port <= 65535:
            raise ValueError("dashboard robot_id/host/port parameters are invalid")
        self.model = DashboardModel(self.robot_id, self.profile, self.mode)

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=16,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        safety_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(VrFrame, "input/vr_frame", self._on_vr, qos_profile_sensor_data)
        self.create_subscription(JointState, "state/joints", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(
            CartesianTargetArray,
            "teleop/cartesian_targets",
            self._on_cartesian,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointCommandCandidate,
            "motion/backend/joint_candidate",
            self._on_backend,
            command_qos,
        )
        self.create_subscription(JointCommand, "control/joint_command", self._on_command, command_qos)
        self.create_subscription(SafetyState, "safety/state", self._on_safety, safety_qos)
        self.create_subscription(
            DiagnosticArray,
            "diagnostics/control_chain",
            self._on_diagnostics,
            safety_qos,
        )
        self._enable_client = self.create_client(SetEnabled, "safety/set_enabled")
        self._reset_client = self.create_client(ResetFault, "safety/reset_fault")

    def _on_vr(self, message: VrFrame) -> None:
        self.model.observe(
            "vr",
            {
                "source_id": message.source_id,
                "protocol_version": int(message.protocol_version),
                "sequence": int(message.sequence),
                "packet_loss_total": int(message.packet_loss_total),
                "tracking": {
                    "head": _tracked_pose(message.head),
                    "left": _tracked_pose(message.left_controller),
                    "right": _tracked_pose(message.right_controller),
                },
                "inputs": {
                    "left": _controller_input(message.left_input),
                    "right": _controller_input(message.right_input),
                },
            },
        )

    def _on_joints(self, message: JointState) -> None:
        self.model.observe(
            "joints", {"count": len(message.name), "values": _joint_values(message)}
        )

    def _on_cartesian(self, message: CartesianTargetArray) -> None:
        self.model.observe(
            "cartesian_targets",
            {
                "source_id": message.source_id,
                "session_id": message.session_id,
                "sequence": int(message.sequence),
                "groups": [
                    {
                        "name": target.group_name,
                        "reference_frame": target.reference_frame,
                        "tip_frame": target.tip_frame,
                        "pose_mode": int(target.pose_mode),
                        "pose": _pose(target.pose),
                    }
                    for target in message.targets
                ],
            },
        )

    def _on_backend(self, message: JointCommandCandidate) -> None:
        self.model.observe_group(
            "backend_candidates",
            message.group_name,
            {
                "sequence": int(message.sequence),
                "source_id": message.source_id,
                "session_id": message.session_id,
                "control_mode": int(message.control_mode),
                "names": list(message.command.name),
                "positions": [round(float(value), 4) for value in message.command.position],
            },
        )

    def _on_command(self, message: JointCommand) -> None:
        self.model.observe_group(
            "commands",
            message.group_name,
            {
                "sequence": int(message.sequence),
                "source_id": message.source_id,
                "session_id": message.session_id,
                "mode": int(message.control_mode),
                "names": list(message.command.name),
                "positions": [round(float(value), 4) for value in message.command.position],
            },
        )

    def _on_safety(self, message: SafetyState) -> None:
        labels = {0: "DISABLED", 1: "STANDBY", 2: "ACTIVE", 3: "FAULT", 4: "ESTOP"}
        state = int(message.state)
        self.model.set_safety(
            {
                "state": state,
                "label": labels.get(state, "UNKNOWN"),
                "enabled": bool(message.enabled),
                "fault_latched": bool(message.fault_latched),
                "estop_active": bool(message.estop_active),
                "reason": message.reason,
                "active_source": message.active_source,
                "active_session": message.active_session,
            }
        )

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        statuses: dict[str, Any] = {}
        for status in message.status:
            name = status.name.rsplit("/", 1)[-1]
            statuses[name] = {
                "level": _diagnostic_level(status.level),
                "message": status.message,
                "values": {
                    item.key: _diagnostic_value(item.value) for item in status.values
                },
            }
        self.model.set_diagnostics({"available": bool(statuses), "statuses": statuses})

    def set_enabled(self, enabled: bool, timeout: float = 2.0) -> dict[str, Any]:
        request = SetEnabled.Request()
        request.enabled = bool(enabled)
        request.requester_id = "hc_dashboard"
        request.reason = "operator dashboard request"
        response = self._call(self._enable_client, request, timeout)
        return {"success": bool(response.success), "reason": response.reason}

    def reset_fault(self, timeout: float = 2.0) -> dict[str, Any]:
        request = ResetFault.Request()
        request.requester_id = "hc_dashboard"
        request.reason = "operator dashboard fault reset"
        response = self._call(self._reset_client, request, timeout)
        return {"success": bool(response.success), "reason": response.reason}

    @staticmethod
    def _call(client: Any, request: Any, timeout: float) -> Any:
        if not client.wait_for_service(timeout_sec=min(timeout, 0.5)):
            raise RuntimeError("ROS service is unavailable")
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda unused: completed.set())
        if not completed.wait(timeout):
            raise TimeoutError("ROS service response timed out")
        error = future.exception()
        if error is not None:
            raise RuntimeError(str(error))
        return future.result()
