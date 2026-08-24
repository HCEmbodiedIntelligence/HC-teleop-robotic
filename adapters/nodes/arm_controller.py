#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
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
    parser = argparse.ArgumentParser(description="VR relative-pose teleop for HC-TJ arms")
    parser.add_argument(
        "--config",
        default=default_teleop_config(),
    )
    parser.add_argument(
        "--backend",
        choices=("v23", "generic", "legacy"),
        help="override control.backend from YAML",
    )
    args = parser.parse_args()
    from adapters.core.arm_teleop_node import HcTjArmTeleopNode

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
