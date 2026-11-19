"""Standard teleop bridge connecting RobotArmTeleopNode and humanoid_motion_control_node."""
from __future__ import annotations

import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
import yaml

from adapters.nodes.motion_teleop_bridge import profile_settings


def _valid_pose(pose) -> bool:
    pos = pose.position
    quat = pose.orientation
    vals = (pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w)
    if not all(math.isfinite(float(v)) for v in vals):
        return False
    return (quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w) >= 1.0e-12


class _ControllerShim:
    """Minimal controller facade expected by MotionIntervention."""

    def __init__(self, bridge: "StandardMotionTeleopBridge"):
        self._bridge = bridge
        self.free_joint_names = bridge.free_joint_names

    def reset_to_feedback(self) -> None:
        self._bridge.reset_to_feedback()


class StandardMotionTeleopBridge(Node):
    """Drop-in replacement for ControllerNode backed by humanoid_motion_control_node."""

    def __init__(self, profile: Path | str):
        self.profile_dir = Path(profile).resolve()
        teleop_path = self.profile_dir / "arm_teleop.yaml"
        arm_names, base_frame, endpoints, control = profile_settings(teleop_path)
        self.teleop = yaml.safe_load(teleop_path.read_text(encoding="utf-8")) or {}
        self.control = control
        super().__init__("motion_teleop_bridge")

        self.sides = tuple(arm_names)
        self.arm_joints = {side: list(arm_names[side]) for side in self.sides}
        self.free_joint_names = [
            name for side in self.sides for name in self.arm_joints[side]
        ]
        self.base_frame = base_frame
        self.endpoints = endpoints
        self.rate_hz = float(control.get("rate_hz", 100.0))
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("control.rate_hz must be positive")
        self.solver_timeout_ns = int(
            float(control.get("joint_state_timeout", 0.5)) * 1_000_000_000
        )

        self.joint_state: dict[str, float] = {}
        self.command_q: dict[str, float] = {}
        self.arm_target_received = {side: False for side in self.sides}
        self.latest_motion_stamp = {side: 0 for side in self.sides}
        self.motion_cutoff_ns = 0
        self.reset_pending = False
        self.reset_cutoff_ns = 0
        self.clutch_held = False
        self.clutch_held_q: dict[str, float] = {}
        self.clutch_reset_pending = False
        self.clutch_event_ns = 0
        self.clutch_feedback_cutoff_ns = 0
        self.source_target_pending = False
        self.source_target_cutoff_ns = 0
        self.control_source = "vr"
        self.feedback_received_total = 0
        self.last_feedback_receive_ns = 0
        self.last_feedback_source_ns = 0
        self.last_reset_wait_log = float("-inf")
        self.feedback_rejections: dict[str, int] = {}
        self.last_feedback_rejection_log = float("-inf")

        latest_reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        feedback_reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
        )
        sensor_depth4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
        )
        motion_cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.command_pub = self.create_publisher(
            JointState,
            control.get("generic_command_topic", "/control/solver/joint_targets"),
            latest_reliable_qos,
        )
        solver_topic = control.get("solver_topic", "/status/solver/joint_states")
        self.solver_pub = (
            self.create_publisher(JointState, solver_topic, latest_reliable_qos)
            if solver_topic
            else None
        )
        self.reset_ack_pub = self.create_publisher(
            Header,
            control.get("solver_reset_ack_topic", "/status/solver/reset_ack"),
            feedback_reliable_qos,
        )
        self.servo_p_pubs = {
            side: self.create_publisher(
                PoseStamped, endpoints[(side, "servo_p")], qos_profile_sensor_data
            )
            for side in self.sides
        }
        self.servo_j_pubs = {
            side: self.create_publisher(
                JointState, endpoints[(side, "servo_j")], qos_profile_sensor_data
            )
            for side in self.sides
        }

        self.controller = _ControllerShim(self)
        self.intervention = None
        if control.get("legacy_intervention", False):
            from adapters.core.motion_intervention import MotionIntervention

            self.intervention = MotionIntervention(
                self, self.teleop.get("grippers"), expected_pose_frame=self.base_frame
            )

        self.create_subscription(
            JointState,
            control.get("joint_state_topic", "/observation/joint_states"),
            self._joint_callback,
            sensor_depth4_qos,
        )
        self.create_subscription(
            JointState,
            "/control/motion/joint_targets",
            self._motion_target_callback,
            motion_cmd_qos,
        )
        self.create_subscription(
            PoseArray,
            control.get("target_pose_topic", "/action/ee_poses"),
            self._ee_callback,
            feedback_reliable_qos,
        )
        self.create_subscription(
            JointState,
            "/control/solver/target_joint_from_vr",
            self._servo_j_callback,
            feedback_reliable_qos,
        )
        reset_topic = control.get("solver_reset_topic", "/control/solver/reset")
        if reset_topic:
            self.create_subscription(
                Bool, reset_topic, self._reset_callback, feedback_reliable_qos
            )
        self.create_subscription(
            Header,
            "/control/solver/clutch",
            self._clutch_callback,
            feedback_reliable_qos,
        )
        self.create_subscription(
            String,
            control.get("control_source_topic", "/control/source"),
            self._control_source_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"motion teleop bridge started: {len(self.free_joint_names)} arm joints, "
            f"base_frame={self.base_frame}, rate={self.rate_hz:.1f} Hz"
        )

    def reset_to_feedback(self) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        self.motion_cutoff_ns = max(self.motion_cutoff_ns, now_ns)
        self.arm_target_received = {side: False for side in self.sides}
        self.latest_motion_stamp = {side: 0 for side in self.sides}
        for name in self.free_joint_names:
            if name in self.joint_state:
                self.command_q[name] = self.joint_state[name]

    def _control_source_callback(self, message: String) -> None:
        source = message.data.strip().lower()
        if source not in {"vr", "motion", "exoskeleton"} or source == self.control_source:
            return
        self.control_source = source
        self.clutch_held = False
        self.clutch_held_q.clear()
        self.clutch_reset_pending = False
        if self.intervention and self.intervention.active:
            return
        self._reset_references()

    def _reset_references(self) -> None:
        self.clutch_held = False
        self.clutch_held_q.clear()
        self.clutch_reset_pending = False
        self.reset_pending = True
        now_ns = int(self.get_clock().now().nanoseconds)
        self.reset_cutoff_ns = now_ns
        self.clutch_event_ns = max(self.clutch_event_ns, now_ns)
        self.source_target_pending = True
        self.motion_cutoff_ns = max(self.motion_cutoff_ns, now_ns)
        self.arm_target_received = {side: False for side in self.sides}

    def _reset_callback(self, message: Bool) -> None:
        if self.intervention and self.intervention.active:
            return
        if message.data:
            self._reset_references()

    def _clutch_callback(self, message: Header) -> None:
        if self.control_source != "vr" or (self.intervention and self.intervention.active):
            return
        stamp_ns = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
        now_ns = int(self.get_clock().now().nanoseconds)
        if message.frame_id not in {"hold", "resume", "clear"} or not -50_000_000 <= now_ns - stamp_ns <= 250_000_000:
            return
        if stamp_ns <= self.clutch_event_ns:
            return
        self.clutch_event_ns = stamp_ns
        if message.frame_id == "clear":
            self.clutch_held = False
            self.clutch_held_q.clear()
            self.clutch_reset_pending = False
        elif message.frame_id == "hold":
            self.clutch_held = True
            self.clutch_reset_pending = False
            self.motion_cutoff_ns = max(self.motion_cutoff_ns, now_ns)
            self.arm_target_received = {side: False for side in self.sides}
            self.clutch_held_q = {
                name: float(self.command_q.get(name, self.joint_state[name]))
                for name in self.free_joint_names
                if name in self.command_q or name in self.joint_state
            }
        else:
            self.clutch_held = True
            self.clutch_reset_pending = True
            self.clutch_feedback_cutoff_ns = now_ns
            self.source_target_pending = True
            self.source_target_cutoff_ns = now_ns
            self.motion_cutoff_ns = max(self.motion_cutoff_ns, now_ns)
            self.arm_target_received = {side: False for side in self.sides}
        self.get_logger().info(
            f"arm clutch {message.frame_id}: event_ns={stamp_ns}, held={self.clutch_held}"
        )

    def _joint_callback(self, message: JointState) -> None:
        self.feedback_received_total += 1
        now_ns = int(self.get_clock().now().nanoseconds)
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.last_feedback_receive_ns = now_ns
        self.last_feedback_source_ns = stamp_ns

        resetting = self.reset_pending and not (self.intervention and self.intervention.active)
        reason = None
        if len(message.name) != len(message.position):
            reason = "length_mismatch"
        elif len(set(message.name)) != len(message.name):
            reason = "duplicate_names"
        elif not set(self.free_joint_names) <= set(message.name):
            reason = "incomplete_joints"
        elif not all(math.isfinite(float(v)) for v in message.position):
            reason = "nonfinite_position"
        elif not -50_000_000 <= now_ns - stamp_ns <= 250_000_000:
            reason = "stale_or_future_feedback"
        elif resetting and stamp_ns <= self.reset_cutoff_ns:
            reason = "before_reset_cutoff"
        elif self.clutch_reset_pending and stamp_ns <= self.clutch_feedback_cutoff_ns:
            reason = "before_clutch_cutoff"
        if reason:
            self.feedback_rejections[reason] = self.feedback_rejections.get(reason, 0) + 1
            mono_now = time.monotonic()
            if mono_now - self.last_feedback_rejection_log >= 2.0:
                self.last_feedback_rejection_log = mono_now
                self.get_logger().warning(
                    f"joint feedback rejected before state update: reason={reason}, "
                    f"source_ns={stamp_ns}, receive_ns={now_ns}, "
                    f"age_ms={(now_ns - stamp_ns) / 1e6:.3f}, "
                    f"reset_pending={self.reset_pending}, reset_cutoff_ns={self.reset_cutoff_ns}"
                )
            return

        for name, pos in zip(message.name, message.position):
            self.joint_state[name] = float(pos)

        if resetting:
            self.reset_to_feedback()
            self.reset_pending = False
            if self.source_target_pending:
                self.source_target_cutoff_ns = now_ns
            acknowledgement = Header()
            acknowledgement.stamp = self.get_clock().now().to_msg()
            self.reset_ack_pub.publish(acknowledgement)
            self.get_logger().warning("motion solver reset to fresh joint feedback")
        if self.clutch_reset_pending and not resetting:
            self.reset_to_feedback()
            self.clutch_reset_pending = False
            self.clutch_held = False
            self.clutch_held_q.clear()
            self.source_target_cutoff_ns = now_ns
            self.reset_ack_pub.publish(
                Header(stamp=self.get_clock().now().to_msg(), frame_id="clutch")
            )
            self.get_logger().info(
                "arm clutch integrators reset to fresh feedback; waiting for new targets"
            )
        if self.intervention:
            self.intervention.feedback(message)

    def _ee_callback(self, message: PoseArray, *, owned: bool = False):
        if self.intervention and self.intervention.active and not owned:
            return False
        if not owned:
            if self.control_source != "vr" or self.reset_pending or self.clutch_held:
                return False
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if self.source_target_pending and stamp_ns <= self.source_target_cutoff_ns:
                return False
            if message.header.frame_id and message.header.frame_id != self.base_frame:
                self.get_logger().warning(
                    f"ignoring EE targets in frame '{message.header.frame_id}'; expected '{self.base_frame}'",
                    throttle_duration_sec=2.0,
                )
                return False
        if len(message.poses) != len(self.sides):
            self.get_logger().warning(
                f"ignoring {len(message.poses)} EE targets; expected {len(self.sides)}",
                throttle_duration_sec=2.0,
            )
            return False
        for pose in message.poses:
            if not _valid_pose(pose):
                self.get_logger().warning("ignoring invalid non-finite/zero-quat EE target")
                return False
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        for side, pose in zip(self.sides, message.poses):
            out = PoseStamped()
            out.header.stamp = stamp
            out.header.frame_id = self.base_frame
            out.pose = pose
            self.servo_p_pubs[side].publish(out)
        self.source_target_pending = False
        return True

    def _servo_j_callback(self, message: JointState) -> None:
        if self.intervention and self.intervention.active:
            return
        if self.control_source != "vr" or self.reset_pending or self.clutch_held:
            return
        if len(message.name) != len(message.position) or not message.name:
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.source_target_pending and stamp_ns <= self.source_target_cutoff_ns:
            return
        try:
            values = {n: float(v) for n, v in zip(message.name, message.position)}
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(v) for v in values.values()):
            return
        out_stamp = message.header.stamp
        if out_stamp.sec == 0 and out_stamp.nanosec == 0:
            out_stamp = self.get_clock().now().to_msg()
        published = False
        for side in self.sides:
            joints = self.arm_joints[side]
            if all(n in values for n in joints):
                out = JointState()
                out.header.stamp = out_stamp
                out.name = list(joints)
                out.position = [values[n] for n in joints]
                self.servo_j_pubs[side].publish(out)
                published = True
        if published:
            self.source_target_pending = False

    def _motion_target_callback(self, message: JointState) -> None:
        if (
            not message.name
            or len(message.name) != len(message.position)
            or len(set(message.name)) != len(message.name)
        ):
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp_ns <= self.motion_cutoff_ns:
            return
        try:
            values = {n: float(v) for n, v in zip(message.name, message.position)}
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(v) for v in values.values()):
            return
        for side in self.sides:
            joints = self.arm_joints[side]
            if all(n in values for n in joints):
                for n in joints:
                    self.command_q[n] = values[n]
                self.latest_motion_stamp[side] = stamp_ns
                self.arm_target_received[side] = True

    def _tick(self) -> None:
        owned = self.intervention if self.intervention and self.intervention.active else None
        if owned and not owned.authorized():
            return
        if self.reset_pending:
            mono_now = time.monotonic()
            if mono_now - self.last_reset_wait_log >= 2.0:
                self.last_reset_wait_log = mono_now
                now_ns = int(self.get_clock().now().nanoseconds)
                receive_age = (
                    (now_ns - self.last_feedback_receive_ns) / 1e6
                    if self.last_feedback_receive_ns
                    else None
                )
                source_age = (
                    (now_ns - self.last_feedback_source_ns) / 1e6
                    if self.last_feedback_source_ns
                    else None
                )
                self.get_logger().warning(
                    f"solver reset waiting for fresh feedback: cutoff_ns={self.reset_cutoff_ns}, "
                    f"feedback_received={self.feedback_received_total}, "
                    f"receive_age_ms={receive_age}, source_age_ms={source_age}, "
                    f"rejected={self.feedback_rejections}; output remains blocked"
                )
            return
        if not owned and (self.control_source != "vr" or self.source_target_pending):
            return
        if owned and owned.homing:
            if owned.home_command is not None:
                message, owned.home_command = owned.home_command, None
                owned.output.publish(message)
            return

        now_ns = int(self.get_clock().now().nanoseconds)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.free_joint_names)

        if self.clutch_held and not owned:
            if not all(n in self.clutch_held_q for n in self.free_joint_names):
                return
            message.header.frame_id = "solver_clutch_hold"
            message.position = [float(self.clutch_held_q[n]) for n in self.free_joint_names]
        else:
            if not all(self.arm_target_received.values()):
                return
            if not all(n in self.command_q for n in self.free_joint_names):
                return
            if any(
                now_ns - self.latest_motion_stamp[side] > self.solver_timeout_ns
                for side in self.sides
            ):
                return
            message.position = [float(self.command_q[n]) for n in self.free_joint_names]

        if owned:
            owned.publish(message)
            if owned.record_joint_target.get_subscription_count():
                owned.record_joint_target.publish(
                    JointState(
                        header=Header(stamp=message.header.stamp),
                        name=message.name,
                        position=message.position,
                    )
                )
        else:
            self.command_pub.publish(message)
        if self.solver_pub is not None:
            self.solver_pub.publish(message)


def main(profile: Path | str) -> None:
    try:
        from middleware.core.qos_event_observer import QosEventObserver
        from middleware.core.ros_executor import create_single_threaded_executor
    except ImportError:
        QosEventObserver = None
        create_single_threaded_executor = None

    rclpy.init()
    node = None
    executor = None
    qos_observer = None
    try:
        node = StandardMotionTeleopBridge(profile)
        if QosEventObserver is not None:
            qos_observer = QosEventObserver([node])
            qos_observer.start()
        if create_single_threaded_executor is not None:
            executor = create_single_threaded_executor()
            executor.add_node(node)
            executor.spin()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if qos_observer is not None:
            try:
                qos_observer.close()
            except Exception:
                pass
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
            if node is not None:
                executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
