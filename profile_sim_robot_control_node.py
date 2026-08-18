#!/usr/bin/env python3
"""Run the IO PyBullet simulator with an imported profile directory."""
from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path


def main() -> None:
    io_root = Path(os.environ.get("HC_IO_ROOT", "/home/maple/hc_io_suit")).resolve()
    profile_root = Path(
        os.environ.get(
            "HC_ROBOT_CONFIG_ROOT", str(Path(__file__).with_name("robot_configs"))
        )
    ).resolve()
    source = io_root / "src/scripts/general_sim_robot_control_node_ros2.py"
    if not source.is_file():
        raise SystemExit(f"IO simulator not found: {source}")
    spec = importlib.util.spec_from_file_location("hc_io_profile_sim", source)
    if spec is None or spec.loader is None:
        raise SystemExit(f"Unable to load IO simulator: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    original_resolver = module.SimRobotController._resolve_config_file

    def resolve_config(instance, config_path: str) -> Path:
        candidate = Path(config_path).expanduser()
        if not candidate.is_absolute():
            return original_resolver(instance, config_path)
        candidate = candidate.resolve()
        try:
            candidate.relative_to(profile_root)
        except ValueError as exc:
            raise ValueError(
                f"Imported simulation config must be inside {profile_root}"
            ) from exc
        if not candidate.is_file():
            raise FileNotFoundError(f"Simulation config does not exist: {candidate}")
        return candidate

    module.SimRobotController._resolve_config_file = resolve_config
    module.main()


if __name__ == "__main__":
    main()
