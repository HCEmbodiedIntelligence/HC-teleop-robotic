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
from sensor_msgs.msg import JointState
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
        self.command_pub = self.create_publisher(
            JointState, publications["joint_target"], 10
        )
        self.solver_pub = (
            self.create_publisher(JointState, publications["solver_state"], 10)
            if publications.get("solver_state")
            else None
        )
        self.create_subscription(
            JointState, subscriptions["joint_state"], self._joint_callback, 10
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
                PoseArray, subscriptions["ee_target"], self._ee_callback, 10
            )
        rate = float(ros_config.get("rate", 400.0))
        if rate <= 0.0:
            raise ValueError("ros_interface.rate must be positive")
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.failure_count = 0
        self.last_failure_log = 0.0
        self.get_logger().info(
            "reconstructed controller_v2_3 started: "
            f"{len(self.controller.free_joint_names)} free joints, "
            f"{len(self.controller.tasks)} tasks"
        )

    def _joint_callback(self, message: JointState) -> None:
        try:
            self.controller.update_joint_state(message.name, message.position)
        except ValueError as exception:
            self.get_logger().warning(f"ignoring invalid joint feedback: {exception}")

    def _tf_callback(self, message: TFMessage) -> None:
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

    def _ee_callback(self, message: PoseArray) -> None:
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

    def _tick(self) -> None:
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
