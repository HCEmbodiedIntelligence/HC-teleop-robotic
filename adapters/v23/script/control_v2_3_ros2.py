#!/usr/bin/env python3
"""ROS 2 wrapper for the behavioral controller_v2_3 reconstruction."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from tf2_msgs.msg import TFMessage

HERE = Path(__file__).resolve()
REPO_ROOT = HERE.parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SOURCE = HERE.parent.parent / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

try:
    from middleware.core.control.qos_event_observer import QosEventObserver
    from middleware.core.control.ros_executor import create_single_threaded_executor
except ImportError:
    QosEventObserver = None
    create_single_threaded_executor = None

from controller_v2_3 import ControllerV23, TargetTransform  # noqa: E402


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    quaternion = np.asarray([w, x, y, z], dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("target quaternion has zero length")
    w, x, y, z = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ]
    )


class ControllerNode(Node):
    def __init__(self, config_path: str):
        self.controller = ControllerV23(config_path)
        ros_config = self.controller.cfg["ros_interface"]
        super().__init__(str(ros_config.get("node_name", "controller_v2_3")))
        subscriptions = ros_config.get("sub_topic", {})
        publications = ros_config.get("pub_topic", {})
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
        self.command_pub = self.create_publisher(
            JointState, publications["joint_target"], latest_reliable_qos
        )
        self.solver_pub = (
            self.create_publisher(
                JointState, publications["solver_state"], latest_reliable_qos
            )
            if publications.get("solver_state")
            else None
        )
        self.reset_ack_pub = self.create_publisher(
            Header,
            publications.get("reset_ack", "/status/solver/reset_ack"),
            feedback_reliable_qos,
        )
        self.create_subscription(
            JointState,
            subscriptions["joint_state"],
            self._joint_callback,
            latest_reliable_qos,
        )
        if subscriptions.get("reset"):
            self.create_subscription(
                Bool, subscriptions["reset"], self._reset_callback, feedback_reliable_qos
            )
        if subscriptions.get("tf_target"):
            self.create_subscription(
                TFMessage, subscriptions["tf_target"], self._tf_callback, 10
            )
        self.ee_pose_order = [
            (task.root, task.frame) for task in self.controller.pose_tasks
        ]
        if subscriptions.get("ee_target"):
            self.create_subscription(
                PoseArray,
                subscriptions["ee_target"],
                self._ee_callback,
                latest_reliable_qos,
            )
        rate = float(ros_config.get("rate", 400.0))
        if rate <= 0.0:
            raise ValueError("ros_interface.rate must be positive")
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.failure_count = 0
        self.last_failure_log = 0.0
        self.reset_pending = False
        self.clutch_held = False
        self.clutch_reset_pending = False
        self.clutch_event_ns = 0
        self.clutch_feedback_cutoff_ns = 0
        self.feedback_received_total = 0
        self.last_feedback_receive_ns = 0
        self.last_feedback_source_ns = 0
        self.last_reset_wait_log = float("-inf")
        self.feedback_rejections = {}
        self.last_feedback_rejection_log = float("-inf")
        self.reset_cutoff_ns = 0
        self.control_source = "vr"
        self.source_target_pending = False
        self.source_target_cutoff_ns = 0
        self.create_subscription(
            String, subscriptions.get("control_source", "/control/source"),
            self._control_source_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.intervention = None
        import yaml
        teleop_file = Path(config_path).resolve().parent / 'arm_teleop.yaml'
        teleop_config = (yaml.safe_load(teleop_file.read_text()) or {}) if teleop_file.is_file() else {}
        if teleop_config.get('control', {}).get('legacy_intervention', False):
            from adapters.core.v23_intervention import V23Intervention
            self.intervention = V23Intervention(self, teleop_config.get('grippers'))
        arm_names = {name for arm in teleop_config.get("arms", {}).values() for name in arm.get("joint_names", [])}
        self.clutch_columns = np.array([i for i, name in enumerate(self.controller.free_joint_names) if name in arm_names], dtype=int)
        self.clutch_q_indices = self.controller.interface.q_indices[self.clutch_columns]
        self.create_subscription(Header, "/control/solver/clutch", self._clutch_callback, feedback_reliable_qos)
        self.get_logger().info(
            "reconstructed controller_v2_3 started: "
            f"{len(self.controller.free_joint_names)} free joints, "
            f"{len(self.controller.tasks)} tasks"
        )

    def _control_source_callback(self, message: String) -> None:
        source = message.data.strip().lower()
        if source not in {"vr", "motion", "exoskeleton"} or source == self.control_source:
            return
        self.control_source = source
        self.clutch_held = False
        self.clutch_reset_pending = False
        if self.intervention and self.intervention.active:
            return
        self._reset_references()

    def _reset_references(self) -> None:
        self.clutch_held = False
        self.clutch_reset_pending = False
        self.reset_pending = True
        self.reset_cutoff_ns = int(self.get_clock().now().nanoseconds)
        self.clutch_event_ns = max(self.clutch_event_ns, self.reset_cutoff_ns)
        self.source_target_pending = True
        self.controller.targets.transforms.clear()

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
            self.clutch_reset_pending = False
        elif message.frame_id == "hold":
            self.clutch_held = True
            self.clutch_reset_pending = False
            self.controller.previous_velocity[self.clutch_columns] = 0.0
        else:
            self.clutch_held = True  # Keep holding until post-request feedback arrives.
            self.clutch_reset_pending = True
            self.clutch_feedback_cutoff_ns = now_ns
            self.source_target_pending = True
            self.source_target_cutoff_ns = now_ns
        self.get_logger().info(f"arm clutch {message.frame_id}: event_ns={stamp_ns}, held={self.clutch_held}")

    def _joint_callback(self, message: JointState) -> None:
        self.feedback_received_total += 1
        self.last_feedback_receive_ns = int(self.get_clock().now().nanoseconds)
        self.last_feedback_source_ns = (message.header.stamp.sec * 1_000_000_000
                                        + message.header.stamp.nanosec)
        try:
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            now_ns = int(self.get_clock().now().nanoseconds)
            resetting = self.reset_pending and not (self.intervention and self.intervention.active)
            reason = None
            if len(message.name) != len(message.position):
                reason = "length_mismatch"
            elif len(set(message.name)) != len(message.name):
                reason = "duplicate_names"
            elif not set(self.controller.free_joint_names) <= set(message.name):
                reason = "incomplete_joints"
            elif not all(np.isfinite(value) for value in message.position):
                reason = "nonfinite_position"
            elif not -50_000_000 <= now_ns - stamp_ns <= 250_000_000:
                reason = "stale_or_future_feedback"
            elif resetting and stamp_ns <= self.reset_cutoff_ns:
                reason = "before_reset_cutoff"
            elif self.clutch_reset_pending and stamp_ns <= self.clutch_feedback_cutoff_ns:
                reason = "before_clutch_cutoff"
            if reason:
                self.feedback_rejections[reason] = self.feedback_rejections.get(reason, 0) + 1
                now = time.monotonic()
                if now - self.last_feedback_rejection_log >= 2.0:
                    self.last_feedback_rejection_log = now
                    self.get_logger().warning(
                        f"joint feedback rejected before state update: reason={reason}, "
                        f"source_ns={stamp_ns}, receive_ns={now_ns}, "
                        f"age_ms={(now_ns-stamp_ns)/1e6:.3f}, "
                        f"reset_pending={self.reset_pending}, reset_cutoff_ns={self.reset_cutoff_ns}"
                    )
                return
            self.controller.update_joint_state(message.name, message.position)
            if resetting:
                self.controller.reset_to_feedback()
                self.reset_pending = False
                if self.source_target_pending:
                    self.source_target_cutoff_ns = now_ns
                acknowledgement = Header()
                acknowledgement.stamp = self.get_clock().now().to_msg()
                self.reset_ack_pub.publish(acknowledgement)
                self.get_logger().warning(
                    "retarget integrator reset to fresh joint feedback"
                )
            if self.clutch_reset_pending and not resetting:
                self.controller.command_q[self.clutch_q_indices] = self.controller.q[self.clutch_q_indices]
                self.controller.previous_velocity[self.clutch_columns] = 0.0
                self.clutch_reset_pending = False
                self.clutch_held = False
                self.source_target_cutoff_ns = now_ns
                self.reset_ack_pub.publish(Header(stamp=self.get_clock().now().to_msg(), frame_id="clutch"))
                self.get_logger().info("arm clutch integrators reset to fresh feedback; waiting for new targets")
            if self.intervention:
                self.intervention.feedback(message)
        except ValueError as exception:
            self.get_logger().warning(f"ignoring invalid joint feedback: {exception}")

    def _reset_callback(self, message: Bool) -> None:
        if self.intervention and self.intervention.active:
            return  # Ownership permits exclusively manage resets during a session.
        if message.data:
            self._reset_references()

    def _tf_callback(self, message: TFMessage) -> None:
        if self.intervention and self.intervention.active:
            return
        if self.control_source != "vr" or self.reset_pending:
            return
        if self.source_target_pending and (not message.transforms or any(
                transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
                <= self.source_target_cutoff_ns for transform in message.transforms)):
            return
        targets = {}
        try:
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                targets[(parent, child)] = TargetTransform(
                    np.array(
                        [translation.x, translation.y, translation.z], dtype=float
                    ),
                    quaternion_to_rotation(
                        rotation.x, rotation.y, rotation.z, rotation.w
                    ),
                )
        except ValueError as exception:
            self.get_logger().warning(f"ignoring invalid TF target: {exception}")
            return
        self.controller.update_tf_targets(targets)
        if all(self.controller.targets.relative_target(task.root, task.frame) is not None
               for task in self.controller.pose_tasks):
            self.source_target_pending = False

    def _ee_callback(self, message: PoseArray, *, owned=False):
        if self.intervention and self.intervention.active and not owned:
            return False
        if not owned:
            if self.control_source != "vr" or self.reset_pending:
                return False
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if self.source_target_pending and stamp_ns <= self.source_target_cutoff_ns:
                return False
        if len(message.poses) != len(self.ee_pose_order):
            self.get_logger().warning(
                f"ignoring {len(message.poses)} EE targets; "
                f"expected {len(self.ee_pose_order)}"
            )
            return
        targets = {}
        try:
            for key, pose in zip(self.ee_pose_order, message.poses):
                position, orientation = pose.position, pose.orientation
                targets[key] = TargetTransform(
                    np.array([position.x, position.y, position.z], dtype=float),
                    quaternion_to_rotation(
                        orientation.x, orientation.y, orientation.z, orientation.w
                    ),
                )
        except ValueError as exception:
            self.get_logger().warning(f"ignoring invalid EE target: {exception}")
            return
        self.controller.update_tf_targets(targets)
        self.source_target_pending = False
        return True

    def _tick(self) -> None:
        owned = self.intervention if self.intervention and self.intervention.active else None
        if owned and not owned.authorized():
            return
        if self.reset_pending:
            now = time.monotonic()
            if now - self.last_reset_wait_log >= 2.0:
                self.last_reset_wait_log = now
                now_ns = int(self.get_clock().now().nanoseconds)
                receive_age = ((now_ns-self.last_feedback_receive_ns)/1e6
                               if self.last_feedback_receive_ns else None)
                source_age = ((now_ns-self.last_feedback_source_ns)/1e6
                              if self.last_feedback_source_ns else None)
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
            # Serialize home and IK in this executor; never let both publish.
            if owned.home_command is not None:
                message, owned.home_command = owned.home_command, None
                owned.output.publish(message)
            return
        try:
            held_q = self.controller.command_q[self.clutch_q_indices].copy() if self.clutch_held and not owned else None
            names, positions = self.controller.step()
            if held_q is not None:
                self.controller.command_q[self.clutch_q_indices] = held_q
                self.controller.previous_velocity[self.clutch_columns] = 0.0
                positions[self.clutch_columns] = held_q
        except RuntimeError:
            return
        except Exception as exception:
            self.failure_count += 1
            now = time.monotonic()
            if now - self.last_failure_log >= 1.0:
                self.last_failure_log = now
                self.get_logger().error(
                    f"controller step failed; holding feedback: {exception}"
                )
            return
        self.failure_count = 0
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        if self.clutch_held and not owned:
            message.header.frame_id = "solver_clutch_hold"
        message.name = list(names)
        message.position = [float(position) for position in positions]
        if owned:
            owned.publish(message)
            if owned.record_joint_target.get_subscription_count():
                owned.record_joint_target.publish(JointState(
                    header=Header(stamp=message.header.stamp), name=message.name, position=message.position))
        else:
            self.command_pub.publish(message)
        if self.solver_pub is not None:
            self.solver_pub.publish(message)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="path to controller_v2_3 YAML")
    arguments, ros_arguments = parser.parse_known_args(argv)
    rclpy.init(args=ros_arguments)
    node = None
    executor = None
    qos_observer = None
    try:
        node = ControllerNode(arguments.config)
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


if __name__ == "__main__":
    main()
