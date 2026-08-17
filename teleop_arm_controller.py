#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError

from hc_teleop_middleware.arm_teleop_node import HcTjArmTeleopNode


def main() -> None:
    parser = argparse.ArgumentParser(description="VR relative-pose teleop for HC-TJ arms")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("arm_teleop.yaml")),
    )
    parser.add_argument(
        "--backend",
        choices=("v23", "generic", "legacy"),
        help="override control.backend from YAML",
    )
    args = parser.parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = HcTjArmTeleopNode(args.config, backend=args.backend)
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
