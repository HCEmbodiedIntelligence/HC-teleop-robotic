#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

from hc_teleop_middleware.robot_profiles import RobotProfileError, RobotProfileManager


def load_selection(config_path: Path) -> tuple[str, RobotProfileManager]:
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise RobotProfileError(f"unable to read middleware config: {exc}") from exc
    profile_config = config.get("robot_profiles", {})
    if not isinstance(profile_config, dict):
        raise RobotProfileError("middleware robot_profiles must be an object")
    profile_id = str(profile_config.get("active", "")).strip()
    if not profile_id:
        raise RobotProfileError("no active robot profile is selected")
    root = Path(str(profile_config.get("root", "robot_configs"))).expanduser()
    if not root.is_absolute():
        root = config_path.parent / root
    return profile_id, RobotProfileManager(root)


def main() -> None:
    parser = argparse.ArgumentParser(description="Resolve the active robot profile")
    parser.add_argument(
        "field",
        choices=("active", "root", "profile-dir", "controller", "teleop", "simulation"),
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("middleware.yaml")),
    )
    arguments = parser.parse_args()
    try:
        config_path = Path(arguments.config).expanduser().resolve()
        profile_id, manager = load_selection(config_path)
        if arguments.field == "active":
            print(profile_id)
            return
        if arguments.field == "root":
            print(manager.root)
            return
        profile = manager.get(profile_id)
        profile_dir = Path(profile["path"])
        paths = {
            "profile-dir": profile_dir,
            "controller": profile_dir / "controller_v23.yml",
            "teleop": profile_dir / "arm_teleop.yaml",
            "simulation": profile_dir / "vr_configs.yml",
        }
        selected = paths[arguments.field]
        if arguments.field != "profile-dir" and not selected.is_file():
            raise RobotProfileError(
                f"active profile {profile_id} has no {selected.name}"
            )
        print(selected)
    except RobotProfileError as exc:
        print(f"Robot profile error: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
