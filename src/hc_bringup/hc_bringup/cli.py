from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

from .profile import ProfileError, load_profile, resolve_profile


def _result(name: str, state: str, message: str) -> dict[str, str]:
    return {"check": name, "state": state, "message": message}


def _package_available(package_name: str) -> bool:
    try:
        from ament_index_python.packages import get_package_share_directory

        get_package_share_directory(package_name)
        return True
    except Exception:
        return False


def doctor(profile_value: str, *, strict: bool = False) -> tuple[bool, list[dict[str, str]]]:
    checks: list[dict[str, str]] = []
    try:
        path = resolve_profile(profile_value)
        profile = load_profile(path)
    except ProfileError as error:
        return False, [_result("profile", "error", str(error))]
    checks.append(_result("profile", "ok", f"{profile.robot_id}: {profile.path}"))

    for resource_name in profile.value.get("resources", {}):
        try:
            resource = profile.resource(resource_name)
            state = "ok" if resource.exists() else "error"
            checks.append(
                _result(
                    f"resource:{resource_name}",
                    state,
                    str(resource) if resource.exists() else f"missing: {resource}",
                )
            )
        except ProfileError as error:
            checks.append(_result(f"resource:{resource_name}", "error", str(error)))

    packages = sorted(
        {
            str(component["adapter"]["package"])
            for component in profile.components
            if component.get("enabled", True)
        }
    )
    motion_package = profile.value.get("motion", {}).get("backend_package")
    if motion_package:
        packages.append(str(motion_package))
    for package_name in sorted(set(packages)):
        available = _package_available(package_name)
        checks.append(
            _result(
                f"package:{package_name}",
                "ok" if available else ("error" if strict else "warning"),
                "installed" if available else "not present in the current ament index",
            )
        )

    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    checks.append(_result("ros_domain", "ok", domain))
    success = not any(item["state"] == "error" for item in checks)
    return success, checks


def _print_checks(checks: list[dict[str, str]], json_output: bool) -> None:
    if json_output:
        print(json.dumps({"checks": checks}, ensure_ascii=False, indent=2))
        return
    icons = {"ok": "OK", "warning": "WARN", "error": "ERROR"}
    for item in checks:
        print(f"[{icons[item['state']]}] {item['check']}: {item['message']}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="hcctl", description="HC Teleop workspace doctor")
    subparsers = parser.add_subparsers(dest="command", required=True)

    doctor_parser = subparsers.add_parser("doctor", help="validate a runnable robot profile")
    doctor_parser.add_argument("--profile", required=True)
    doctor_parser.add_argument("--strict", action="store_true")
    doctor_parser.add_argument("--json", action="store_true")

    profile_parser = subparsers.add_parser("profile", help="profile operations")
    profile_subparsers = profile_parser.add_subparsers(dest="profile_command", required=True)
    validate_parser = profile_subparsers.add_parser("validate")
    validate_parser.add_argument("path")
    validate_parser.add_argument("--json", action="store_true")

    args = parser.parse_args(argv)
    try:
        if args.command == "doctor":
            success, checks = doctor(args.profile, strict=args.strict)
            _print_checks(checks, args.json)
            return 0 if success else 1
        if args.command == "profile" and args.profile_command == "validate":
            profile = load_profile(Path(args.path))
            value: dict[str, Any] = {
                "ok": True,
                "robot_id": profile.robot_id,
                "components": len(profile.components),
                "path": str(profile.path),
            }
            if args.json:
                print(json.dumps(value, ensure_ascii=False, indent=2))
            else:
                print(
                    f"OK {value['robot_id']}: {value['components']} components ({value['path']})"
                )
            return 0
    except ProfileError as error:
        print(f"hcctl: {error}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
