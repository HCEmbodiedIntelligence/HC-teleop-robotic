#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError

def default_teleop_config() -> str:
    root = Path(__file__).resolve().parents[2]
    profile_cfg = root / "adapters" / "robots" / "x1" / "arm_teleop.yaml"
    if profile_cfg.is_file():
        return str(profile_cfg)
    return str(root / "arm_teleop.yaml")


def main() -> None:
    parser = argparse.ArgumentParser(description="Profile-driven VR relative-pose teleop")
    parser.add_argument(
        "--config",
        default=default_teleop_config(),
    )
    parser.add_argument(
        "--backend",
        choices=("v23", "generic", "motion", "legacy"),
        help="override control.backend from YAML",
    )
    args, ros_args = parser.parse_known_args()
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from middleware.core.ros_executor import create_single_threaded_executor

    if hasattr(sys, "setswitchinterval"):
        try:
            sys.setswitchinterval(0.001)
        except ValueError:
            pass
    rclpy.init(args=ros_args)
    node = None
    command_node = None
    executor = None
    command_executor = None
    command_thread = None
    try:
        command_node = rclpy.create_node(
            "hc_robot_vr_teleop_cmd",
            enable_rosout=False,
            start_parameter_services=False,
        )
        node = RobotArmTeleopNode(args.config, backend=args.backend, command_node=command_node)
        command_executor = create_single_threaded_executor()
        command_executor.add_node(command_node)
        command_thread = threading.Thread(
            target=command_executor.spin,
            name="arm-command-executor",
            daemon=True,
        )
        command_thread.start()
        executor = create_single_threaded_executor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if command_executor is not None:
            command_executor.shutdown(timeout_sec=1.0)
            if command_node is not None:
                command_executor.remove_node(command_node)
        if command_thread is not None:
            command_thread.join(timeout=1.0)
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
            if node is not None:
                executor.remove_node(node)
        # Close RobotArmTeleopNode (and its QosEventObserver) before destroying
        # command_node so moved waitables restore cleanly onto command_node first.
        if node is not None:
            node.destroy_node()
        if command_node is not None:
            command_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
