"""Small process-level plugin registry for interchangeable IK backends.

Solver processes intentionally remain outside the VR controller process.  A
backend may therefore use a different Python/ROS environment (the reconstructed
V2.3 solver does) or be a native ROS 2 executable (Motion Server does).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class SolverPlugin:
    name: str
    display_name: str
    target_protocol: str
    command_protocol: str


_PLUGINS = {
    "v23": SolverPlugin(
        name="v23",
        display_name="HC Pinocchio V2.3",
        target_protocol="geometry_msgs/PoseArray",
        command_protocol="sensor_msgs/JointState via /hc_teleop/joint_cmd_arm",
    ),
    "motion_server": SolverPlugin(
        name="motion_server",
        display_name="Humanoid Motion Server",
        target_protocol="geometry_msgs/PoseStamped ServoP",
        command_protocol="sensor_msgs/JointState via /hc_teleop/joint_cmd_vr",
    ),
}


def available_plugins() -> tuple[SolverPlugin, ...]:
    return tuple(_PLUGINS.values())


def resolve_plugin(
    config: str | Path | Mapping[str, Any], override: str | None = None
) -> SolverPlugin:
    if isinstance(config, Mapping):
        value = config
    else:
        with Path(config).expanduser().open(encoding="utf-8") as stream:
            value = yaml.safe_load(stream) or {}
    control = value.get("control", {}) if isinstance(value, Mapping) else {}
    configured = control.get("backend", "v23") if isinstance(control, Mapping) else "v23"
    name = str(override or configured).strip().lower().replace("-", "_")
    try:
        return _PLUGINS[name]
    except KeyError as error:
        choices = ", ".join(_PLUGINS)
        raise ValueError(f"unknown solver backend '{name}'; choose one of: {choices}") from error


def motion_server_resources(config_path: str | Path) -> tuple[Path, ...]:
    """Resolve a profile's Motion Server files without allowing path escape."""
    path = Path(config_path).expanduser().resolve()
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    settings = config.get("motion_server")
    robot = config.get("robot")
    if not isinstance(settings, Mapping) or not isinstance(robot, Mapping):
        raise ValueError("profile does not contain motion_server and robot resources")
    values = [
        settings.get("motion_params"),
        settings.get("channel_config"),
        settings.get("sdk_config"),
        settings.get("tool_config"),
        robot.get("urdf_path"),
    ]
    profile_dir = path.parent
    result = []
    for value in values:
        if not isinstance(value, str) or not value:
            raise ValueError("motion_server resource paths must be non-empty strings")
        candidate = (profile_dir / value).resolve()
        try:
            candidate.relative_to(profile_dir)
        except ValueError as error:
            raise ValueError(f"motion_server resource escapes profile: {value}") from error
        result.append(candidate)
    return tuple(result)
