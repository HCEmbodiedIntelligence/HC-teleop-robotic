#!/usr/bin/env python3
"""Safe ROS scheduling wrapper around the copied generic Controller.Run_IK."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
import yaml
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import JointState


PROJECT_DIR = Path(__file__).resolve().parent
COPIED_CONTROL_DIR = PROJECT_DIR / "vendor/io_unicontroller_ros2/control"
# The copied CasADi objective has an undefined derivative at (or extremely near)
# zero SE(3) error. 0.2 mm / 0.011 deg is below the simulator and VR noise floor,
# so treat that state as reached instead of feeding it back into Ipopt.
REACHED_TASK_TOLERANCE = 2e-4
sys.path.insert(0, str(COPIED_CONTROL_DIR / "script"))
sys.path.insert(0, str(COPIED_CONTROL_DIR / "src"))

from control_sol_q import Controller, SE3_to_msg, msg_to_SE3  # noqa: E402


class SafeGenericSolQ(Node):
    def __init__(self, config_path: str):
        super().__init__("robot_control_node")
        self.config_path = Path(config_path).expanduser().resolve()
        with self.config_path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        topics = config["control_ros"]
        self.rate = float(topics["rate"])
        self.controller = Controller(str(self.config_path))
        self.joint_names = list(self.controller.joint_name)
        self.feedback: np.ndarray | None = None
        self.joint_target: np.ndarray | None = None
        self.last_solution: np.ndarray | None = None
        self.target_poses: list[pin.SE3] | None = None
        self.target_signature: np.ndarray | None = None
        self.target_dirty = False
        self.next_retry = 0.0
        self.failure_count = 0
        self.last_failure_log = 0.0

        self.solution_pub = self.create_publisher(JointState, topics["sol_q"], 10)
        self.ee_state_pub = self.create_publisher(PoseArray, topics["ee_state"], 10)
        self.create_subscription(
            JointState, topics["joint_state"], self._joint_state_callback, 10
        )
        self.create_subscription(
            JointState, topics["joint_target"], self._joint_target_callback, 10
        )
        self.create_subscription(
            PoseArray, topics["ee_target"], self._target_pose_callback, 10
        )
        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            "safe generic sol_q ready: copied Controller.Run_IK with hold/retry guard"
        )

    def _ordered_positions(self, message: JointState) -> np.ndarray | None:
        if len(message.name) != len(message.position):
            return None
        values = dict(zip(message.name, message.position))
        if any(name not in values for name in self.joint_names):
            return None
        result = np.asarray([values[name] for name in self.joint_names], dtype=float)
        return result if np.all(np.isfinite(result)) else None

    def _joint_state_callback(self, message: JointState) -> None:
        values = self._ordered_positions(message)
        if values is None:
            return
        self.feedback = values
        if self.joint_target is None:
            self.joint_target = values.copy()
        if self.last_solution is None:
            self.last_solution = values.copy()

    def _joint_target_callback(self, message: JointState) -> None:
        values = self._ordered_positions(message)
        if values is not None:
            self.joint_target = values
            self.target_dirty = True

    @staticmethod
    def _pose_signature(message: PoseArray) -> np.ndarray | None:
        if not message.poses:
            return None
        values = np.asarray(
            [
                value
                for pose in message.poses
                for value in (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            return None
        quaternions = values.reshape(-1, 7)[:, 3:]
        if np.any(np.linalg.norm(quaternions, axis=1) < 1e-8):
            return None
        return values

    def _target_pose_callback(self, message: PoseArray) -> None:
        if len(message.poses) != len(self.controller.ee_name):
            self.get_logger().warning(
                f"ignoring {len(message.poses)} EE targets; expected "
                f"{len(self.controller.ee_name)}"
            )
            return
        signature = self._pose_signature(message)
        if signature is None:
            self.get_logger().warning("ignoring non-finite EE target")
            return
        changed = self.target_signature is None or not np.allclose(
            signature, self.target_signature, atol=1e-10, rtol=0.0
        )
        self.target_poses = [msg_to_SE3(pose) for pose in message.poses]
        self.target_signature = signature
        if changed:
            self.target_dirty = True
            self.next_retry = 0.0

    def _task_error(self, q: np.ndarray) -> float:
        assert self.target_poses is not None
        actual = self.controller.GetEEPose(q)
        errors = [
            float(np.linalg.norm(pin.log6(current.inverse() * target).vector))
            for current, target in zip(actual, self.target_poses)
        ]
        return max(errors, default=0.0)

    def _publish_solution(self) -> None:
        if self.last_solution is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.joint_names
        message.position = [float(value) for value in self.last_solution]
        self.solution_pub.publish(message)

    def _publish_ee_state(self) -> None:
        if self.feedback is None:
            return
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = [SE3_to_msg(pose) for pose in self.controller.GetEEPose(self.feedback)]
        self.ee_state_pub.publish(message)

    def _tick(self) -> None:
        if (
            self.feedback is None
            or self.joint_target is None
            or self.target_poses is None
        ):
            return
        now = time.monotonic()
        task_error = self._task_error(self.feedback)
        feedback_error = (
            float(np.max(np.abs(self.last_solution - self.feedback)))
            if self.last_solution is not None
            else math.inf
        )
        # The copied optimizer has an undefined gradient at exactly zero SE(3)
        # error. Once feedback has reached the unchanged target, publish the last
        # valid solution without calling Ipopt again.
        if task_error <= REACHED_TASK_TOLERANCE and feedback_error <= 1e-4:
            self.target_dirty = False

        if self.target_dirty and now >= self.next_retry:
            try:
                solution, converged = self.controller.Run_IK(
                    self.target_poses, self.joint_target, self.feedback
                )
            except Exception as exc:  # protected vendor code can raise RuntimeError
                solution, converged = None, False
                if now - self.last_failure_log >= 1.0:
                    self.get_logger().warning(f"generic IK exception; holding: {exc}")
            valid = (
                converged
                and solution is not None
                and np.asarray(solution).shape == (len(self.joint_names),)
                and np.all(np.isfinite(solution))
            )
            if valid:
                self.last_solution = np.asarray(solution, dtype=float)
                self.failure_count = 0
                self.next_retry = now
            else:
                self.failure_count += 1
                self.next_retry = now + 0.2
                if now - self.last_failure_log >= 1.0:
                    self.last_failure_log = now
                    self.get_logger().warning(
                        "generic IK did not converge; holding last valid solution "
                        f"(task_error={task_error:.6f}, failures={self.failure_count})"
                    )

        self._publish_solution()
        self._publish_ee_state()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = SafeGenericSolQ(args.config)
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
