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
SOURCE = HERE.parent.parent / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

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
            latest_reliable_qos,
        )
        self.create_subscription(
            JointState,
            subscriptions["joint_state"],
            self._joint_callback,
            latest_reliable_qos,
        )
        if subscriptions.get("reset"):
            self.create_subscription(
                Bool, subscriptions["reset"], self._reset_callback, latest_reliable_qos
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
        if teleop_file.is_file() and (yaml.safe_load(teleop_file.read_text()) or {}).get('control', {}).get('legacy_intervention', False):
            from adapters.core.v23_intervention import V23Intervention
            self.intervention = V23Intervention(self)
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
        if self.intervention and self.intervention.active:
            return
        self.reset_pending = True
        self.reset_cutoff_ns = int(self.get_clock().now().nanoseconds)
        self.source_target_pending = True
        self.controller.targets.transforms.clear()

    def _joint_callback(self, message: JointState) -> None:
        try:
            self.controller.update_joint_state(message.name, message.position)
            if self.reset_pending and not (self.intervention and self.intervention.active):
                stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                now_ns = int(self.get_clock().now().nanoseconds)
                if (stamp_ns <= self.reset_cutoff_ns
                        or not -50_000_000 <= now_ns-stamp_ns <= 250_000_000
                        or len(message.name) != len(message.position)
                        or len(set(message.name)) != len(message.name)
                        or not set(self.controller.free_joint_names) <= set(message.name)
                        or not all(np.isfinite(value) for value in message.position)):
                    return
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
            if self.intervention:
                self.intervention.feedback(message)
        except ValueError as exception:
            self.get_logger().warning(f"ignoring invalid joint feedback: {exception}")

    def _reset_callback(self, message: Bool) -> None:
        if self.intervention and self.intervention.active:
            return  # Ownership permits exclusively manage resets during a session.
        if message.data:
            self.reset_pending = True
            self.reset_cutoff_ns = int(self.get_clock().now().nanoseconds)

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
        if not owned and (self.control_source != "vr" or self.source_target_pending):
            return
        token = owned.token if owned else None
        if self.reset_pending:
            return
        if owned and owned.homing:
            # Serialize home and IK in this executor; never let both publish.
            if owned.home_command is not None:
                message, owned.home_command = owned.home_command, None
                owned.output.publish(message)
            return
        try:
            names, positions = self.controller.step()
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
        message.name = list(names)
        message.position = [float(position) for position in positions]
        if owned:
            message.header.frame_id = token.key()+"#"
            owned.output.publish(message)
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
    try:
        node = ControllerNode(arguments.config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
