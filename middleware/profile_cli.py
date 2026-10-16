#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from middleware.core.robot_profiles import RobotProfileError, RobotProfileManager
from middleware.core.robot_api import RobotApiProfile, selected_profile


def load_selection(config_path: Path) -> tuple[str, RobotProfileManager]:
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise RobotProfileError(f"unable to read middleware config: {exc}") from exc
    profile_config = config.get("robot_profiles", {})
    if not isinstance(profile_config, dict):
        raise RobotProfileError("middleware robot_profiles must be an object")
    selected = selected_profile(config, config_path.parent)
    return selected.name, RobotProfileManager(selected.parent)


def main() -> None:
    parser = argparse.ArgumentParser(description="Resolve the active robot profile")
    parser.add_argument(
        "field",
        choices=("active", "root", "profile-dir", "controller", "teleop", "simulation", "api", "generate", "export", "capture-config"),
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "config.yaml"),
    )
    parser.add_argument("--profile", help="explicit profile directory (offline inspection/generation)")
    parser.add_argument("--output", help="runtime directory for generate; zip file for export")
    parser.add_argument("--cameras", help="camera YAML for capture-config; defaults to the selected profile cameras.yaml")
    arguments = parser.parse_args()
    try:
        config_path = Path(arguments.config).expanduser().resolve()
        if arguments.profile:
            selected = Path(arguments.profile).expanduser().resolve()
            profile_id, manager = selected.name, RobotProfileManager(selected.parent)
        else:
            profile_id, manager = load_selection(config_path)
        if arguments.field == "active":
            print(profile_id)
            return
        if arguments.field == "root":
            print(manager.root)
            return
        profile = manager.get(profile_id)
        profile_dir = Path(profile["path"])
        if arguments.field == "capture-config":
            from middleware.core.profile_capture import capture_config
            if not arguments.output:
                raise RobotProfileError("capture-config requires --output")
            output = Path(arguments.output).expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(yaml.safe_dump(capture_config(profile_dir, arguments.cameras), allow_unicode=True, sort_keys=False))
            print(output)
            return
        if arguments.field in {"api", "generate"}:
            import json
            api = RobotApiProfile(profile_dir)
            if arguments.field == "api":
                print(json.dumps(api.describe(), indent=2, allow_nan=False))
            elif not arguments.output:
                raise RobotProfileError("generate requires --output")
            else:
                print(api.write_runtime(arguments.output))
            return
        if arguments.field == "export":
            if not arguments.output:
                raise RobotProfileError("export requires --output")
            Path(arguments.output).write_bytes(manager.export_archive(profile_id))
            print(Path(arguments.output).resolve())
            return
        paths = {
            "profile-dir": profile_dir,
            "controller": profile_dir / "controller_v23.yml",
            "teleop": profile_dir / "arm_teleop.yaml",
            "simulation": (
                profile_dir / "arm_teleop.yaml"
                if not (profile_dir / "vr_configs.yml").is_file() and (profile_dir / "arm_teleop.yaml").is_file()
                else profile_dir / "vr_configs.yml"
            ),
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
