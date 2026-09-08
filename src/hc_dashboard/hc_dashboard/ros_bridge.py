from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from rclpy.time import Time
import copy
import re

from hc_teleop_interfaces.msg import (
    CartesianTargetArray,
    CartesianStateArray,
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
        self.dataset_root = str(self.declare_parameter(
            "dataset_root", str(Path.home() / ".local/share/hc_teleop/datasets")
        ).value)
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
        self._standard_commands = self.create_publisher(JointState, 'standard/joint_commands', 10)
        self._standard_candidates = self.create_publisher(JointState, 'standard/joint_targets', 10)
        self._pose_publishers = {}
        from .inspection import profiles
        selected = next((item for item in profiles(self.profile) if item['id'] == self.robot_id), {})
        groups = selected.get('groups', [])
        self._group_order = sorted(groups, key=lambda name: (0 if name == 'right_arm' else 1 if name == 'left_arm' else 2, name))
        self._joint_order = selected.get('joint_names', {})
        self._feedback_order = selected.get('feedback_joint_order', [])
        self._standard_feedback = self.create_publisher(JointState, 'standard/joint_states', 10)
        self._joint_batches = {'commands': {}, 'targets': {}}
        self._common_frame = str(self.declare_parameter('standard_pose_frame', 'zhi_Link' if self.robot_id == 'x1' else '').value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pose_arrays = {key: self.create_publisher(PoseArray, 'standard/' + key, 10)
                             for key in ['controller_target_ee_poses', 'target_ee_poses', 'actual_ee_poses']}

        self.create_subscription(CartesianStateArray, 'state/cartesian', self._on_cartesian_state, qos_profile_sensor_data)
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
        output = copy.deepcopy(message)
        indices = {name: i for i, name in enumerate(message.name)}
        order = [name for name in self._feedback_order if name in indices]
        order.extend(name for name in message.name if name not in order)
        if len(indices) == len(message.name) and all(len(getattr(message, field)) in (0, len(message.name)) for field in ['position', 'velocity', 'effort']):
            output.name = order
            for field in ['position', 'velocity', 'effort']:
                values = getattr(message, field)
                setattr(output, field, [values[indices[name]] for name in order] if len(values) else [])
            self._standard_feedback.publish(output)
        self.model.observe(
            "joints", {"count": len(message.name), "values": _joint_values(message)}
        )

    def _on_cartesian(self, message: CartesianTargetArray) -> None:
        self._publish_pose_array('controller_target_ee_poses', message.header, message.targets, local=True)
        self._publish_pose_array('target_ee_poses', message.header, message.targets)
        for target in message.targets:
            self._publish_pose('target_ee_pose', target.group_name, target.reference_frame, message.header.stamp, target.pose)
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

    def _publish_pose(self, kind, group, frame, stamp, pose):
        # Separate topics preserve each arm's reference frame; PoseArray has only one header.
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', group):
            return
        topic = f'standard/{kind}/{group}'
        if topic not in self._pose_publishers:
            self._pose_publishers[topic] = self.create_publisher(PoseStamped, topic, 10)
        output = PoseStamped()
        output.header.stamp = stamp
        output.header.frame_id = frame
        output.pose = copy.deepcopy(pose)
        self._pose_publishers[topic].publish(output)
        self.model.observe_standard_pose(topic)

    def _on_cartesian_state(self, message: CartesianStateArray) -> None:
        self.model.observe('cartesian_feedback', {'groups': [
            {'name': state.group_name, 'reference_frame': state.reference_frame,
             'tip_frame': state.tip_frame, 'valid': state.valid, 'pose': _pose(state.pose)}
            for state in message.states]})
        self._publish_pose_array('actual_ee_poses', message.header, message.states)
        for state in message.states:
            if state.valid:
                self._publish_pose('actual_ee_pose', state.group_name, state.reference_frame, message.header.stamp, state.pose)

    def _publish_pose_array(self, kind, header, states, local=False):
        by_group = {state.group_name: state for state in states}
        if not self._group_order or any(name not in by_group for name in self._group_order):
            return
        ordered = [by_group[name] for name in self._group_order]
        if any(not getattr(state, 'valid', True) for state in ordered):
            return
        output = PoseArray()
        output.header = copy.deepcopy(header)
        frames = {state.reference_frame for state in ordered}
        if local:
            # Legacy array contract: each item is relative to its own task base.
            output.header.frame_id = next(iter(frames)) if len(frames) == 1 else 'generic_task_bases'
            output.poses = [copy.deepcopy(state.pose) for state in ordered]
        else:
            frame = self._common_frame or (next(iter(frames)) if len(frames) == 1 else '')
            if not frame:
                return
            output.header.frame_id = frame
            try:
                for state in ordered:
                    if state.reference_frame == frame:
                        output.poses.append(copy.deepcopy(state.pose))
                    else:
                        transform = self._tf_buffer.lookup_transform(frame, state.reference_frame, Time.from_msg(header.stamp))
                        output.poses.append(do_transform_pose(state.pose, transform))
            except TransformException:
                # Never relabel local coordinates as a common frame, or reuse stale TF.
                return
        self._pose_arrays[kind].publish(output)
        self.model.observe_standard_pose('standard/' + kind)

    def _publish_joint_batch(self, kind, message):
        batch = self._joint_batches[kind]
        batch[message.group_name] = (time.monotonic(), copy.deepcopy(message))
        if not self._group_order or any(name not in batch for name in self._group_order):
            return
        samples = [batch[name] for name in self._group_order]
        if max(t for t, _ in samples) - min(t for t, _ in samples) > 0.05:
            return
        if len({(m.source_id, m.session_id, m.sequence) for _, m in samples}) != 1:
            return
        output = JointState()
        output.header = copy.deepcopy(max((m.header for _, m in samples), key=lambda h: (h.stamp.sec, h.stamp.nanosec)))
        for _, sample in samples:
            command = sample.command
            if len(command.name) != len(command.position) or len(set(command.name)) != len(command.name):
                return
            indices = {name: i for i, name in enumerate(command.name)}
            order = self._joint_order.get(sample.group_name, list(command.name))
            if set(indices) != set(order):
                return
            output.name.extend(order)
            output.position.extend(command.position[indices[name]] for name in order)
        # Do not invent velocity, effort or absent gripper commands.
        (self._standard_commands if kind == 'commands' else self._standard_candidates).publish(output)
        batch.clear()

    def _on_backend(self, message: JointCommandCandidate) -> None:
        self._publish_joint_batch('targets', message)
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
        self._publish_joint_batch('commands', message)
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
