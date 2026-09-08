from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

import yaml


SCHEMA = "hc-teleop-profile/v1"
ROBOT_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_-]{0,63}$")
COMPONENT_KINDS = {
    "arm",
    "gripper",
    "dexterous_hand",
    "waist",
    "base",
    "camera",
}
TOP_LEVEL_KEYS = {
    "schema",
    "robot_id",
    "display_name",
    "description",
    "resources",
    "components",
    "motion",
    "simulation",
    "teleop",
    "safety",
    "recording",
    "standard_interfaces",
    "diagnostics",
    "hardware",
    "vr",
}


class ProfileError(ValueError):
    """A robot profile is invalid or references an unsafe resource."""


@dataclass(frozen=True)
class RobotProfile:
    path: Path
    value: dict[str, Any]

    @property
    def robot_id(self) -> str:
        return str(self.value["robot_id"])

    @property
    def components(self) -> tuple[dict[str, Any], ...]:
        return tuple(self.value["components"])

    def component(self, component_id: str) -> dict[str, Any]:
        for component in self.components:
            if component["id"] == component_id:
                return component
        raise KeyError(component_id)

    def resource(
        self,
        name: str,
        package_lookup: Callable[[str], str] | None = None,
    ) -> Path:
        resources = self.value.get("resources", {})
        if name not in resources:
            raise ProfileError(f"profile has no resource named '{name}'")
        return resolve_resource(str(resources[name]), self.path.parent, package_lookup)


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProfileError(f"{name} must be an object")
    return value


def _nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ProfileError(f"{name} must be a non-empty string")
    return value.strip()


def _string_list(value: Any, name: str, *, allow_empty: bool = False) -> list[str]:
    if not isinstance(value, list) or (not value and not allow_empty):
        raise ProfileError(f"{name} must be a {'possibly empty ' if allow_empty else ''}list")
    result = [_nonempty_string(item, f"{name}[]") for item in value]
    if len(result) != len(set(result)):
        raise ProfileError(f"{name} contains duplicate values")
    return result


def _validate_relative_interface(value: Any, name: str) -> None:
    if value is None:
        return
    interface = _nonempty_string(value, name)
    if interface.startswith("/"):
        raise ProfileError(f"{name} must be relative so robot namespaces can be applied")


def _validate_axis_mapping(value: Any, name: str) -> None:
    if (
        not isinstance(value, list)
        or len(value) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in value)
        or any(
            not isinstance(item, (int, float)) or not math.isfinite(float(item))
            for row in value
            for item in row
        )
    ):
        raise ProfileError(f"{name} must be a finite 3x3 matrix")


def validate_profile(value: Any) -> dict[str, Any]:
    profile = dict(_mapping(value, "profile"))
    unknown = sorted(set(profile) - TOP_LEVEL_KEYS)
    if unknown:
        raise ProfileError(f"unknown top-level fields: {', '.join(unknown)}")
    if profile.get("schema") != SCHEMA:
        raise ProfileError(f"schema must be exactly {SCHEMA}")
    robot_id = _nonempty_string(profile.get("robot_id"), "robot_id")
    if not ROBOT_ID_PATTERN.fullmatch(robot_id):
        raise ProfileError("robot_id must match ^[a-z][a-z0-9_-]{0,63}$")
    profile["display_name"] = _nonempty_string(
        profile.get("display_name", robot_id), "display_name"
    )

    resources = dict(_mapping(profile.get("resources", {}), "resources"))
    for name, uri in resources.items():
        _nonempty_string(name, "resources key")
        _nonempty_string(uri, f"resources.{name}")
    profile["resources"] = resources

    components_value = profile.get("components")
    if not isinstance(components_value, list) or not components_value:
        raise ProfileError("components must be a non-empty list")
    components: list[dict[str, Any]] = []
    component_ids: set[str] = set()
    arm_count = 0
    for index, raw_component in enumerate(components_value):
        component = dict(_mapping(raw_component, f"components[{index}]"))
        component_id = _nonempty_string(component.get("id"), f"components[{index}].id")
        if not ROBOT_ID_PATTERN.fullmatch(component_id):
            raise ProfileError(f"component id '{component_id}' has an invalid format")
        if component_id in component_ids:
            raise ProfileError(f"duplicate component id '{component_id}'")
        component_ids.add(component_id)
        kind = _nonempty_string(component.get("kind"), f"components[{index}].kind")
        if kind not in COMPONENT_KINDS:
            raise ProfileError(f"unsupported component kind '{kind}'")
        if kind == "arm":
            arm_count += 1
        component["enabled"] = bool(component.get("enabled", True))
        adapter = dict(_mapping(component.get("adapter", {}), f"components[{index}].adapter"))
        _nonempty_string(adapter.get("package"), f"components[{index}].adapter.package")
        if not any(adapter.get(key) for key in ("plugin", "executable", "hardware_plugin")):
            raise ProfileError(
                f"components[{index}].adapter requires plugin, executable, or hardware_plugin"
            )
        component["adapter"] = adapter

        if kind in {"arm", "gripper", "dexterous_hand", "waist"}:
            component["joint_names"] = _string_list(
                component.get("joint_names"), f"components[{index}].joint_names"
            )
        if kind == "arm":
            frames = dict(_mapping(component.get("frames"), f"components[{index}].frames"))
            for field in ("base", "tip"):
                _nonempty_string(frames.get(field), f"components[{index}].frames.{field}")
            component["frames"] = frames
            component["command_modes"] = _string_list(
                component.get("command_modes", ["servo_p"]),
                f"components[{index}].command_modes",
            )
            if "target_filter" in component:
                target_filter = dict(
                    _mapping(
                        component["target_filter"],
                        f"components[{index}].target_filter",
                    )
                )
                allowed_filter_fields = {
                    "enabled",
                    "workspace_min_m",
                    "workspace_max_m",
                    "min_radius_m",
                    "max_radius_m",
                    "max_position_step_m",
                    "max_orientation_step_rad",
                }
                unknown_filter_fields = sorted(
                    set(target_filter) - allowed_filter_fields
                )
                if unknown_filter_fields:
                    raise ProfileError(
                        f"components[{index}].target_filter has unknown fields: "
                        + ", ".join(unknown_filter_fields)
                    )
                target_filter["enabled"] = bool(target_filter.get("enabled", True))
                for field in ("workspace_min_m", "workspace_max_m"):
                    values = target_filter.get(field)
                    if (
                        not isinstance(values, list)
                        or len(values) != 3
                        or any(
                            not isinstance(item, (int, float))
                            or not math.isfinite(float(item))
                            for item in values
                        )
                    ):
                        raise ProfileError(
                            f"components[{index}].target_filter.{field} "
                            "must contain three finite numbers"
                        )
                    target_filter[field] = [float(item) for item in values]
                if any(
                    lower >= upper
                    for lower, upper in zip(
                        target_filter["workspace_min_m"],
                        target_filter["workspace_max_m"],
                    )
                ):
                    raise ProfileError(
                        f"components[{index}].target_filter workspace bounds "
                        "must be increasing"
                    )
                for field in (
                    "min_radius_m",
                    "max_radius_m",
                    "max_position_step_m",
                    "max_orientation_step_rad",
                ):
                    field_value = target_filter.get(field)
                    if (
                        not isinstance(field_value, (int, float))
                        or not math.isfinite(float(field_value))
                        or float(field_value) <= 0.0
                    ):
                        raise ProfileError(
                            f"components[{index}].target_filter.{field} "
                            "must be positive and finite"
                        )
                    target_filter[field] = float(field_value)
                if target_filter["min_radius_m"] >= target_filter["max_radius_m"]:
                    raise ProfileError(
                        f"components[{index}].target_filter radii must be increasing"
                    )
                component["target_filter"] = target_filter
        if kind in {"gripper", "dexterous_hand"}:
            _nonempty_string(component.get("attached_to"), f"components[{index}].attached_to")
        if kind == "base":
            _nonempty_string(component.get("frame"), f"components[{index}].frame")
            component["command_modes"] = _string_list(
                component.get("command_modes", ["velocity"]),
                f"components[{index}].command_modes",
            )
        if kind == "camera":
            _nonempty_string(component.get("frame"), f"components[{index}].frame")
            for field in ("image_topic", "camera_info_topic", "h264_topic"):
                _validate_relative_interface(component.get(field), f"components[{index}].{field}")
            if not any(component.get(field) for field in ("image_topic", "h264_topic")):
                raise ProfileError(
                    f"components[{index}] camera requires image_topic or h264_topic"
                )
        components.append(component)
    if arm_count > 2:
        raise ProfileError("the v1 runtime supports at most two arm components")

    component_by_id = {component["id"]: component for component in components}
    for component in components:
        attached_to = component.get("attached_to")
        if attached_to:
            parent = component_by_id.get(attached_to)
            if parent is None or parent["kind"] != "arm":
                raise ProfileError(
                    f"component '{component['id']}' must attach to an existing arm"
                )
    profile["components"] = components

    for section in (
        "motion", "simulation", "teleop", "safety", "recording", "diagnostics",
        "hardware", "vr", "standard_interfaces"
    ):
        profile[section] = dict(_mapping(profile.get(section, {}), section))

    standard = profile["standard_interfaces"]
    unknown_standard = sorted(set(standard) - {"feedback_joint_order"})
    if unknown_standard:
        raise ProfileError("unknown standard_interfaces fields: " + ", ".join(unknown_standard))
    if "feedback_joint_order" in standard:
        standard["feedback_joint_order"] = _string_list(
            standard["feedback_joint_order"], "standard_interfaces.feedback_joint_order",
            allow_empty=True,
        )

    motion = profile["motion"]
    backend_package = motion.get("backend_package")
    if backend_package is not None:
        backend_package = _nonempty_string(backend_package, "motion.backend_package")
    if backend_package == "hc_motion_backend_robo_manip" and "robo_manip_sdk" not in resources:
        raise ProfileError(
            "resources.robo_manip_sdk is required by hc_motion_backend_robo_manip"
        )
    for field in (
        "servo_nominal_rate_hz",
        "robo_manip_joint_max_velocity_rad_s",
        "robo_manip_joint_max_acceleration_rad_s2",
        "robo_manip_joint_max_jerk_rad_s3",
        "robo_manip_cartesian_max_linear_velocity_m_s",
        "robo_manip_cartesian_max_linear_acceleration_m_s2",
        "robo_manip_cartesian_max_linear_jerk_m_s3",
        "robo_manip_cartesian_max_angular_velocity_rad_s",
        "robo_manip_cartesian_max_angular_acceleration_rad_s2",
        "robo_manip_cartesian_max_angular_jerk_rad_s3",
        "robo_manip_ik_position_tolerance_m",
        "robo_manip_ik_orientation_tolerance_rad",
        "robo_manip_ik_regularization_task_weight",
        "robo_manip_ik_joint_task_weight",
    ):
        if field in motion and (
            not isinstance(motion[field], (int, float))
            or not math.isfinite(float(motion[field]))
            or float(motion[field]) <= 0.0
        ):
            raise ProfileError(f"motion.{field} must be positive and finite")
    if "robo_manip_servo_lease_ms" in motion and (
        not isinstance(motion["robo_manip_servo_lease_ms"], int)
        or isinstance(motion["robo_manip_servo_lease_ms"], bool)
        or motion["robo_manip_servo_lease_ms"] < 1
    ):
        raise ProfileError(
            "motion.robo_manip_servo_lease_ms must be an integer >= 1"
        )
    for field in (
        "robo_manip_ik_enable_regularization_task",
        "robo_manip_ik_enable_joint_task",
    ):
        if field in motion and not isinstance(motion[field], bool):
            raise ProfileError(f"motion.{field} must be boolean")

    simulation = profile["simulation"]
    if "instant_position_tracking" in simulation and not isinstance(
        simulation["instant_position_tracking"], bool
    ):
        raise ProfileError("simulation.instant_position_tracking must be boolean")
    simulation_robo_limits = dict(
        _mapping(simulation.get("robo_manip_limits", {}), "simulation.robo_manip_limits")
    )
    allowed_simulation_robo_limits = {
        "joint_max_velocity_rad_s",
        "joint_max_acceleration_rad_s2",
        "joint_max_jerk_rad_s3",
        "cartesian_max_linear_velocity_m_s",
        "cartesian_max_linear_acceleration_m_s2",
        "cartesian_max_linear_jerk_m_s3",
        "cartesian_max_angular_velocity_rad_s",
        "cartesian_max_angular_acceleration_rad_s2",
        "cartesian_max_angular_jerk_rad_s3",
    }
    unknown_simulation_robo_limits = sorted(
        set(simulation_robo_limits) - allowed_simulation_robo_limits
    )
    if unknown_simulation_robo_limits:
        raise ProfileError(
            "unknown simulation.robo_manip_limits fields: "
            + ", ".join(unknown_simulation_robo_limits)
        )
    for field, value in simulation_robo_limits.items():
        if (
            not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) <= 0.0
        ):
            raise ProfileError(
                f"simulation.robo_manip_limits.{field} must be positive and finite"
            )
    simulation["robo_manip_limits"] = simulation_robo_limits

    diagnostics = profile["diagnostics"]
    for field in (
        "vr_receive_gap_warn_ms",
        "vr_callback_delay_warn_ms",
        "ik_latency_warn_ms",
        "arbiter_latency_warn_ms",
        "joint_step_warn_rad",
        "missing_candidate_warn_ms",
        "stream_stale_ms",
        "trace_retention_ms",
        "capture_pre_seconds",
        "capture_post_seconds",
        "summary_period_seconds",
        "capture_cooldown_seconds",
    ):
        if field in diagnostics and (
            not isinstance(diagnostics[field], (int, float))
            or not math.isfinite(float(diagnostics[field]))
            or float(diagnostics[field]) <= 0.0
        ):
            raise ProfileError(f"diagnostics.{field} must be positive and finite")

    hardware = profile["hardware"]
    allowed_hardware_fields = {
        "legacy_joint_state_topic",
        "legacy_joint_command_topic",
        "legacy_left_finger_topic",
        "legacy_right_finger_topic",
        "command_publish_rate_hz",
        "command_progress_timeout_ms",
        "feedback_warn_timeout_ms",
    }
    unknown_hardware_fields = sorted(set(hardware) - allowed_hardware_fields)
    if unknown_hardware_fields:
        raise ProfileError(
            "unknown hardware fields: " + ", ".join(unknown_hardware_fields)
        )
    for field in (
        "legacy_joint_state_topic",
        "legacy_joint_command_topic",
        "legacy_left_finger_topic",
        "legacy_right_finger_topic",
    ):
        if field in hardware:
            topic = _nonempty_string(hardware[field], f"hardware.{field}")
            if not topic.startswith("/"):
                raise ProfileError(
                    f"hardware.{field} must be an absolute external-driver topic"
                )
    for field in (
        "command_publish_rate_hz",
        "command_progress_timeout_ms",
        "feedback_warn_timeout_ms",
    ):
        if field in hardware and (
            not isinstance(hardware[field], (int, float))
            or not math.isfinite(float(hardware[field]))
            or float(hardware[field]) <= 0.0
        ):
            raise ProfileError(f"hardware.{field} must be positive and finite")

    teleop = profile["teleop"]
    bindings = teleop.get("bindings", [])
    if not isinstance(bindings, list):
        raise ProfileError("teleop.bindings must be a list")
    seen_groups: set[str] = set()
    seen_controllers: set[str] = set()
    for index, raw_binding in enumerate(bindings):
        binding = _mapping(raw_binding, f"teleop.bindings[{index}]")
        group = _nonempty_string(binding.get("group"), f"teleop.bindings[{index}].group")
        controller = _nonempty_string(
            binding.get("controller"), f"teleop.bindings[{index}].controller"
        )
        if (
            group not in component_by_id
            or component_by_id[group]["kind"] != "arm"
            or not component_by_id[group].get("enabled", True)
        ):
            raise ProfileError(f"teleop binding group '{group}' is not an enabled arm")
        if controller not in {"left", "right"}:
            raise ProfileError("teleop binding controller must be left or right")
        if group in seen_groups or controller in seen_controllers:
            raise ProfileError("teleop binding groups and controllers must be unique")
        if binding.get("axis_mapping") is not None:
            _validate_axis_mapping(
                binding["axis_mapping"], f"teleop.bindings[{index}].axis_mapping"
            )
        seen_groups.add(group)
        seen_controllers.add(controller)
    clutch_controller = teleop.get("clutch_controller", "binding")
    if clutch_controller not in {"binding", "left", "right"}:
        raise ProfileError("teleop.clutch_controller must be binding, left, or right")
    tools = teleop.get("tools", [])
    if not isinstance(tools, list):
        raise ProfileError("teleop.tools must be a list")
    seen_tools: set[str] = set()
    for index, raw_tool in enumerate(tools):
        tool = _mapping(raw_tool, f"teleop.tools[{index}]")
        group = _nonempty_string(tool.get("group"), f"teleop.tools[{index}].group")
        component = component_by_id.get(group)
        if (
            component is None
            or component["kind"] not in {"gripper", "dexterous_hand"}
            or not component.get("enabled", True)
        ):
            raise ProfileError(f"teleop tool group '{group}' is not an enabled gripper or dexterous hand")
        if group in seen_tools:
            raise ProfileError(f"duplicate teleop tool group '{group}'")
        seen_tools.add(group)
        controller = _nonempty_string(
            tool.get("controller", "right"), f"teleop.tools[{index}].controller"
        )
        if controller not in {"left", "right"}:
            raise ProfileError("teleop tool controller must be left or right")
        joints = _string_list(tool.get("joint_names"), f"teleop.tools[{index}].joint_names")
        opened = tool.get("open")
        closed = tool.get("closed")
        if (
            not isinstance(opened, list)
            or not isinstance(closed, list)
            or len(opened) != len(joints)
            or len(closed) != len(joints)
        ):
            raise ProfileError("teleop tool open/closed positions must match joint_names")
        for values, name in ((opened, "open"), (closed, "closed")):
            if any(
                not isinstance(value, (int, float)) or not math.isfinite(float(value))
                for value in values
            ):
                raise ProfileError(f"teleop.tools[{index}].{name} must contain finite numbers")
        if set(joints) != set(component["joint_names"]):
            raise ProfileError(f"teleop tool '{group}' joint_names must match its component")
    mapping = teleop.get("axis_mapping")
    if mapping is not None:
        _validate_axis_mapping(mapping, "teleop.axis_mapping")
    for field in ("position_scale", "clutch_threshold"):
        if field in teleop and (
            not isinstance(teleop[field], (int, float))
            or not math.isfinite(float(teleop[field]))
        ):
            raise ProfileError(f"teleop.{field} must be finite")
    if float(teleop.get("position_scale", 1.0)) <= 0.0:
        raise ProfileError("teleop.position_scale must be positive")
    threshold = float(teleop.get("clutch_threshold", 0.5))
    if not 0.0 <= threshold <= 1.0:
        raise ProfileError("teleop.clutch_threshold must be in [0, 1]")

    for field in ("feedback_topic", "candidate_topic"):
        _validate_relative_interface(profile["motion"].get(field), f"motion.{field}")
    recording_format = profile["recording"].get("format", "mcap")
    if recording_format != "mcap":
        raise ProfileError("recording.format must be mcap in profile schema v1")
    return profile


def load_profile(path: str | Path) -> RobotProfile:
    target = Path(path).expanduser().resolve()
    if not target.is_file():
        raise ProfileError(f"profile does not exist: {target}")
    try:
        with target.open(encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ProfileError(f"unable to read profile {target}: {error}") from error
    return RobotProfile(target, validate_profile(value))


def resolve_resource(
    uri: str,
    profile_directory: Path,
    package_lookup: Callable[[str], str] | None = None,
) -> Path:
    if uri.startswith("package://"):
        suffix = uri[len("package://") :]
        package_name, separator, relative = suffix.partition("/")
        if not separator or not package_name or not relative:
            raise ProfileError(f"invalid package resource URI: {uri}")
        if package_lookup is None:
            try:
                from ament_index_python.packages import get_package_share_directory
            except ImportError as error:
                raise ProfileError("ament_index_python is required for package:// resources") from error
            package_lookup = get_package_share_directory
        try:
            package_root = Path(package_lookup(package_name))
        except Exception as error:
            raise ProfileError(f"ROS package not found for resource {uri}: {error}") from error
        rel_path = Path(relative)
        if rel_path.is_absolute() or ".." in rel_path.parts:
            raise ProfileError(f"package resource escapes its package: {uri}")
        target = package_root / rel_path
        if not target.exists():
            raise ProfileError(f"package resource not found: {uri} at {target}")
        return target
    if "://" in uri or Path(uri).is_absolute():
        raise ProfileError("resources must use package:// or profile-relative paths")
    rel_path = Path(uri)
    if rel_path.is_absolute() or ".." in rel_path.parts:
        raise ProfileError(f"relative resource escapes the profile directory: {uri}")
    target = profile_directory / rel_path
    if not target.exists():
        raise ProfileError(f"relative resource not found: {uri} at {target}")
    return target


def resolve_profile(value: str | Path) -> Path:
    candidate = Path(value).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    profile_id = str(value)
    if not ROBOT_ID_PATTERN.fullmatch(profile_id):
        raise ProfileError(f"profile is neither a file nor a valid profile id: {value}")
    package_name = f"hc_robot_{profile_id.replace('-', '_')}"
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory(package_name)) / "config" / "profile.yaml"
        if installed.is_file():
            return installed.resolve()
    except Exception:
        pass
    workspace = Path.cwd()
    for root in (workspace, *workspace.parents):
        source = root / "src" / package_name / "config" / "profile.yaml"
        if source.is_file():
            return source.resolve()
    raise ProfileError(f"unable to resolve installed or source profile '{profile_id}'")


def resolve_motion_backend(requested: str, motion: dict) -> str:
    """Select the current backend contract without an implicit legacy fallback."""
    if requested not in {"profile", "robo_manip", "external"}:
        raise ProfileError("motion_backend must be profile, robo_manip, or external")
    if requested != "profile":
        return requested
    if motion.get("backend_package") == "hc_motion_backend_robo_manip":
        return "robo_manip"
    raise ProfileError(
        "profile motion.backend_package must be hc_motion_backend_robo_manip; "
        "use motion_backend:=external for an out-of-tree backend"
    )
