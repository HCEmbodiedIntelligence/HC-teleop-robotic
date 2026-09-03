from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from hc_bringup.profile import load_profile, resolve_profile


def _truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _auto_bool(value: str, default: bool) -> bool:
    value = value.strip().lower()
    return default if value in {"", "auto"} else _truthy(value)


def _arms(profile):
    return [
        component
        for component in profile.components
        if component.get("kind") == "arm" and component.get("enabled", True)
    ]


def _sim_components(profile):
    """All kinematic joint groups that the simulator must accept commands for."""
    return [
        component for component in profile.components
        if component.get("kind") in {"arm", "gripper", "dexterous_hand", "waist"}
        and component.get("enabled", True)
        and component.get("frames", {}).get("base")
        and component.get("frames", {}).get("tip")
    ]


def _group_parameters(components):
    parameters = {"group_names": [str(component["id"]) for component in components]}
    for component in components:
        group = str(component["id"])
        parameters[f"{group}.joint_names"] = list(component["joint_names"])
        parameters[f"{group}.base_frame"] = str(component["frames"]["base"])
        parameters[f"{group}.tip_frame"] = str(component["frames"]["tip"])
    return parameters


def _target_filter_parameters(components):
    parameters = {}
    for component in components:
        group = str(component["id"])
        config = component.get("target_filter")
        if not config:
            continue
        prefix = f"{group}.target_filter"
        parameters[f"{prefix}.enabled"] = bool(config.get("enabled", True))
        parameters[f"{prefix}.workspace_min_m"] = [
            float(value) for value in config["workspace_min_m"]
        ]
        parameters[f"{prefix}.workspace_max_m"] = [
            float(value) for value in config["workspace_max_m"]
        ]
        for field in (
            "min_radius_m",
            "max_radius_m",
            "max_position_step_m",
            "max_orientation_step_rad",
        ):
            parameters[f"{prefix}.{field}"] = float(config[field])
    return parameters


def _sim_adapter(arms):
    """Resolve the simulator from the profile's adapter boundary."""
    packages = {str(arm.get("adapter", {}).get("package", "")) for arm in arms}
    if len(packages) != 1:
        raise ValueError("simulation requires all arm components to use one adapter package")
    package = packages.pop()
    known = {
        "hc_adapter_openarmx": (
            "hc_adapter_openarmx::OpenArmXSimNode", "openarmx_sim_node", "openarmx_sim_adapter"),
        "hc_adapter_x1": (
            "hc_adapter_x1::X1SimNode", "x1_sim_node", "x1_sim_adapter"),
    }
    if package not in known:
        raise ValueError(
            f"no simulator is registered for adapter package '{package}'; "
            "set start_sim_adapter:=false or install its adapter package"
        )
    plugin, executable, name = known[package]
    return package, plugin, executable, name


def _hardware_adapter(arms):
    """Resolve an in-workspace bridge for an independently launched driver stack."""
    packages = {str(arm.get("adapter", {}).get("package", "")) for arm in arms}
    if len(packages) != 1:
        raise ValueError("hardware adaptation requires all arms to use one adapter package")
    package = packages.pop()
    known = {
        "hc_adapter_x1": (
            "hc_adapter_x1::X1HardwareAdapterNode",
            "x1_hardware_adapter_node",
            "x1_hardware_adapter",
        ),
    }
    if package not in known:
        raise ValueError(
            f"no in-workspace hardware bridge is registered for '{package}'; "
            "install its external hc-adapter package or set start_hardware_adapter:=false"
        )
    plugin, executable, name = known[package]
    return package, plugin, executable, name


def _component(
    package: str,
    plugin: str,
    name: str,
    namespace: str,
    parameters: dict,
    *,
    intra_process: bool = True,
    **kwargs,
):
    return ComposableNode(
        package=package,
        plugin=plugin,
        name=name,
        namespace=namespace,
        parameters=[parameters],
        extra_arguments=[{"use_intra_process_comms": intra_process}],
        **kwargs,
    )


def _node(package: str, executable: str, name: str, namespace: str, parameters: dict, **kwargs):
    return Node(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        parameters=[parameters],
        output="screen",
        **kwargs,
    )


def _setup(context: LaunchContext):
    fastdds_cfg = Path(__file__).resolve().parents[3] / "config" / "fastdds_udp.xml"
    if fastdds_cfg.is_file() and "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ:
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(fastdds_cfg)

    profile_argument = LaunchConfiguration("profile").perform(context)
    profile = load_profile(resolve_profile(profile_argument))
    robot_id = LaunchConfiguration("robot_id").perform(context).strip() or profile.robot_id
    namespace = f"robots/{robot_id}"
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    composition = LaunchConfiguration("composition").perform(context).strip().lower()
    backend = LaunchConfiguration("motion_backend").perform(context).strip().lower()
    if mode not in {"sim", "shadow", "real"}:
        raise ValueError("mode must be sim, shadow, or real")
    if composition not in {"compact", "isolated"}:
        raise ValueError("composition must be compact or isolated")
    if backend not in {"profile", "kdl", "robo_manip", "external"}:
        raise ValueError("motion_backend must be profile, kdl, robo_manip, or external")

    motion = profile.value.get("motion", {})
    if backend == "profile":
        profile_backend = str(
            motion.get("backend_package", "hc_motion_backend_kdl")
        ).strip()
        backend_by_package = {
            "hc_motion_backend_kdl": "kdl",
            "hc_motion_backend_robo_manip": "robo_manip",
        }
        try:
            backend = backend_by_package[profile_backend]
        except KeyError as error:
            raise ValueError(
                "profile motion.backend_package must be hc_motion_backend_kdl or "
                "hc_motion_backend_robo_manip; use motion_backend:=external for an "
                "out-of-tree backend"
            ) from error

    shadow_arg = LaunchConfiguration("shadow").perform(context).strip().lower()
    shadow = mode == "shadow" if shadow_arg in {"", "auto"} else _truthy(shadow_arg)
    start_vr = _auto_bool(LaunchConfiguration("start_vr").perform(context), True)
    start_arbiter = _auto_bool(LaunchConfiguration("start_arbiter").perform(context), True)
    start_mapper = _auto_bool(LaunchConfiguration("start_vr_mapper").perform(context), True)
    start_router = _auto_bool(LaunchConfiguration("start_motion_router").perform(context), True)
    start_kdl = _auto_bool(
        LaunchConfiguration("start_kdl_backend").perform(context),
        backend == "kdl",
    )
    start_robo_manip = _auto_bool(
        LaunchConfiguration("start_robo_manip_backend").perform(context),
        backend == "robo_manip",
    )
    if start_kdl and start_robo_manip:
        raise ValueError("KDL and RoboManip backends cannot own the same backend topics")
    start_sim = _auto_bool(LaunchConfiguration("start_sim_adapter").perform(context), mode == "sim")
    hardware_argument = LaunchConfiguration("start_hardware_adapter").perform(context)
    start_lease = _auto_bool(LaunchConfiguration("start_auto_lease").perform(context), mode == "sim")
    start_rsp = _auto_bool(
        LaunchConfiguration("start_robot_state_publisher").perform(context), mode == "sim"
    )
    start_dashboard = _auto_bool(
        LaunchConfiguration("start_dashboard").perform(context), True
    )
    start_diagnostics = _auto_bool(
        LaunchConfiguration("start_diagnostics").perform(context), True
    )
    dashboard_host = LaunchConfiguration("dashboard_host").perform(context).strip()
    try:
        dashboard_port = int(LaunchConfiguration("dashboard_port").perform(context))
    except ValueError as error:
        raise ValueError("dashboard_port must be an integer") from error
    if not dashboard_host or not 1 <= dashboard_port <= 65535:
        raise ValueError("dashboard_host/dashboard_port are invalid")

    arms = _arms(profile)
    if not arms:
        raise ValueError("profile must contain at least one enabled arm")
    arm_adapter_packages = {
        str(arm.get("adapter", {}).get("package", "")) for arm in arms
    }
    has_builtin_hardware_adapter = arm_adapter_packages == {"hc_adapter_x1"}
    start_hardware = _auto_bool(
        hardware_argument,
        mode in {"shadow", "real"} and has_builtin_hardware_adapter,
    )
    if start_sim and start_hardware:
        raise ValueError("simulation and hardware adapters cannot be started together")
    arm_by_id = {str(arm["id"]): arm for arm in arms}
    group_parameters = _group_parameters(arms)
    urdf_path = str(profile.resource("urdf"))
    vr = profile.value.get("vr", {})
    safety = profile.value.get("safety", {})
    teleop = profile.value.get("teleop", {})
    simulation = profile.value.get("simulation", {})
    diagnostics = profile.value.get("diagnostics", {})
    command_topic = "shadow/control/joint_command" if shadow else "control/joint_command"

    simulation_robo_limits = simulation.get("robo_manip_limits", {})
    if not isinstance(simulation_robo_limits, dict):
        raise ValueError("simulation.robo_manip_limits must be a mapping")

    def robo_limit(name: str, default: float) -> float:
        if mode == "sim" and name in simulation_robo_limits:
            return float(simulation_robo_limits[name])
        return float(motion.get(f"robo_manip_{name}", default))

    gateway_parameters = {
        "listen_host": str(vr.get("listen_host", "0.0.0.0")),
        "pose_port": int(vr.get("pose_port", 5005)),
        "discovery_port": int(vr.get("discovery_port", 5006)),
        "timeout_ms": int(vr.get("pose_timeout_ms", 150)),
        "output_topic": "input/vr_frame",
    }
    arbiter_parameters = {
        "enabled_on_start": bool(safety.get("enabled_on_start", False)),
        "command_timeout_sec": float(safety.get("command_timeout_ms", 150)) / 1000.0,
        "candidate_topic": "control/joint_candidate",
        "command_topic": command_topic,
        "safety_topic": "safety/state",
    }
    router_parameters = {
        "allowed_groups": [str(arm["id"]) for arm in arms],
        "target_input_topic": "teleop/cartesian_targets",
        "backend_target_topic": "motion/backend/cartesian_targets",
        "backend_candidate_topic": "motion/backend/joint_candidate",
        "candidate_output_topic": "control/joint_candidate",
    }
    bindings = teleop.get("bindings", [])
    if not bindings:
        bindings = [
            {"group": group, "controller": controller}
            for group, controller in zip(arm_by_id, ("left", "right"))
        ]
    axis_mapping = teleop.get(
        "axis_mapping", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    binding_axis_mappings = [
        float(value)
        for item in bindings
        for row in item.get("axis_mapping", axis_mapping)
        for value in row
    ]
    mapper_parameters = {
        "group_names": [str(item["group"]) for item in bindings],
        "controllers": [str(item["controller"]) for item in bindings],
        # The mapper's candidate source must match the source that the
        # auto-lease node acquires.  Profiles may override the default
        # (currently ``pico``); leaving the mapper's node default at ``vr``
        # causes every otherwise-valid candidate to be rejected by the arbiter.
        "source_id": str(vr.get("session_id", "vr")),
        "base_frames": [arm_by_id[str(item["group"])] ["frames"]["base"] for item in bindings],
        "tip_frames": [arm_by_id[str(item["group"])] ["frames"]["tip"] for item in bindings],
        "axis_mapping": [float(value) for row in axis_mapping for value in row],
        "binding_axis_mappings": binding_axis_mappings,
        "position_scale": float(teleop.get("position_scale", 1.0)),
        "clutch_threshold": float(teleop.get("clutch_threshold", 0.5)),
        "clutch_controller": str(teleop.get("clutch_controller", "binding")),
        "feedback_timeout_sec": float(teleop.get("feedback_timeout_ms", 200)) / 1000.0,
        "command_ttl_sec": float(teleop.get("command_ttl_ms", 100)) / 1000.0,
        "vr_topic": "input/vr_frame",
        "cartesian_state_topic": "state/cartesian",
        "target_topic": "teleop/cartesian_targets",
        "candidate_topic": "control/joint_candidate",
    }
    tools = teleop.get("tools", [])
    mapper_parameters.update({
        "tool_group_names": [str(item["group"]) for item in tools],
        "tool_controllers": [str(item.get("controller", "right")) for item in tools],
        "tool_joint_counts": [len(item["joint_names"]) for item in tools],
        "tool_joint_names": [name for item in tools for name in item["joint_names"]],
        "tool_open_positions": [float(value) for item in tools for value in item["open"]],
        "tool_closed_positions": [float(value) for item in tools for value in item["closed"]],
    })
    enabled_components = [
        component for component in profile.components if component.get("enabled", True)
    ]
    motion_components = [
        component for component in enabled_components
        if component.get("kind") in {"arm", "waist"}
    ]
    hardware = profile.value.get("hardware", {})
    controller_arms = {
        str(binding.get("controller", "")): str(binding.get("group", ""))
        for binding in bindings
    }

    def tool_for_controller(controller: str):
        arm_group = controller_arms.get(controller, "")
        for tool in tools:
            component = next(
                (
                    item for item in enabled_components
                    if str(item.get("id", "")) == str(tool.get("group", ""))
                ),
                {},
            )
            if str(component.get("attached_to", "")) == arm_group:
                return tool
        return {}

    left_tool = tool_for_controller("left")
    right_tool = tool_for_controller("right")
    hardware_parameters = {
        "command_output_enabled": mode == "real" and not shadow,
        "command_topic": command_topic,
        "safety_topic": "safety/state",
        "joint_state_topic": "state/joints",
        "legacy_joint_state_topic": str(
            hardware.get("legacy_joint_state_topic", "/hc_teleop/joint_states")
        ),
        "legacy_joint_command_topic": str(
            hardware.get("legacy_joint_command_topic", "/hc_teleop/joint_cmd")
        ),
        "legacy_left_finger_topic": str(
            hardware.get(
                "legacy_left_finger_topic", "/hc_teleop/joint_cmd_finger_left"
            )
        ),
        "legacy_right_finger_topic": str(
            hardware.get(
                "legacy_right_finger_topic", "/hc_teleop/joint_cmd_finger_right"
            )
        ),
        "command_publish_rate_hz": float(
            hardware.get("command_publish_rate_hz", 100.0)
        ),
        "command_progress_timeout_sec": float(
            hardware.get("command_progress_timeout_ms", 150.0)
        ) / 1000.0,
        "feedback_warn_timeout_sec": float(
            hardware.get("feedback_warn_timeout_ms", 500.0)
        ) / 1000.0,
        "motion_group_names": [str(item["id"]) for item in motion_components],
    }
    for component in motion_components:
        hardware_parameters[f"{component['id']}.joint_names"] = list(
            component["joint_names"]
        )
    if left_tool:
        hardware_parameters.update({
            "left_finger_group": str(left_tool["group"]),
            "left_finger_input_joint": str(left_tool["joint_names"][0]),
            "left_finger_open": float(left_tool["open"][0]),
            "left_finger_closed": float(left_tool["closed"][0]),
        })
    if right_tool:
        hardware_parameters.update({
            "right_finger_group": str(right_tool["group"]),
            "right_finger_input_joint": str(right_tool["joint_names"][0]),
            "right_finger_open": float(right_tool["open"][0]),
            "right_finger_closed": float(right_tool["closed"][0]),
        })
    kdl_parameters = {
        **group_parameters,
        "urdf_path": urdf_path,
        "target_topic": "motion/backend/cartesian_targets",
        "joint_state_topic": "state/joints",
        "candidate_topic": "motion/backend/joint_candidate",
        "feedback_timeout_sec": float(teleop.get("feedback_timeout_ms", 200)) / 1000.0,
        "command_velocity_scale": float(motion.get("servo_velocity_scale", 0.5)),
        "command_acceleration_limit": float(motion.get("servo_acceleration_limit", 20.0)),
        "command_nominal_rate_hz": float(motion.get("servo_nominal_rate_hz", 60.0)),
        "command_reset_timeout_sec": float(motion.get("servo_reset_timeout_ms", 250)) / 1000.0,
        "command_tracking_error_reset": float(motion.get("servo_tracking_error_reset", 0.5)),
        "cartesian_state_topic": "state/cartesian",
        "publish_cartesian_state": True,
    }
    robo_manip_parameters = {
        **group_parameters,
        **_target_filter_parameters(arms),
        "urdf_path": urdf_path,
        "target_topic": "motion/backend/cartesian_targets",
        "joint_state_topic": "state/joints",
        "candidate_topic": "motion/backend/joint_candidate",
        "cartesian_state_topic": "state/cartesian",
        "publish_fk": True,
        "feedback_timeout_sec": float(teleop.get("feedback_timeout_ms", 200)) / 1000.0,
        "session_reset_timeout_sec": float(
            motion.get("servo_reset_timeout_ms", 250)
        ) / 1000.0,
        "nominal_rate_hz": float(motion.get("servo_nominal_rate_hz", 60.0)),
        "tick_failure_reset_count": int(
            motion.get("robo_manip_tick_failure_reset_count", 3)
        ),
        "ik_position_tolerance_m": float(
            motion.get("robo_manip_ik_position_tolerance_m", 0.001)
        ),
        "ik_orientation_tolerance_rad": float(
            motion.get("robo_manip_ik_orientation_tolerance_rad", 0.015)
        ),
        "ik_enable_regularization_task": bool(
            motion.get("robo_manip_ik_enable_regularization_task", True)
        ),
        "ik_regularization_task_weight": float(
            motion.get("robo_manip_ik_regularization_task_weight", 0.0005)
        ),
        "ik_enable_joint_task": bool(
            motion.get("robo_manip_ik_enable_joint_task", True)
        ),
        "ik_joint_task_weight": float(
            motion.get("robo_manip_ik_joint_task_weight", 0.001)
        ),
        "joint_max_velocity_rad_s": robo_limit("joint_max_velocity_rad_s", 0.6),
        "joint_max_acceleration_rad_s2": robo_limit("joint_max_acceleration_rad_s2", 2.4),
        "joint_max_jerk_rad_s3": robo_limit("joint_max_jerk_rad_s3", 9.6),
        "cartesian_max_linear_velocity_m_s": robo_limit(
            "cartesian_max_linear_velocity_m_s", 0.1
        ),
        "cartesian_max_linear_acceleration_m_s2": robo_limit(
            "cartesian_max_linear_acceleration_m_s2", 0.3
        ),
        "cartesian_max_linear_jerk_m_s3": robo_limit(
            "cartesian_max_linear_jerk_m_s3", 1.5
        ),
        "cartesian_max_angular_velocity_rad_s": robo_limit(
            "cartesian_max_angular_velocity_rad_s", 0.5
        ),
        "cartesian_max_angular_acceleration_rad_s2": robo_limit(
            "cartesian_max_angular_acceleration_rad_s2", 2.0
        ),
        "cartesian_max_angular_jerk_rad_s3": robo_limit(
            "cartesian_max_angular_jerk_rad_s3", 10.0
        ),
    }
    if start_robo_manip:
        robo_manip_parameters["sdk_config_path"] = str(
            profile.resource("robo_manip_sdk")
        )
    sim_group_parameters = _group_parameters(_sim_components(profile))
    sim_parameters = {
        **sim_group_parameters,
        "urdf_path": urdf_path,
        "command_topic": command_topic,
        "joint_state_topic": "state/joints",
        "cartesian_state_topic": "state/cartesian",
        "publish_rate_hz": float(simulation.get("publish_rate_hz", 100.0)),
        "max_velocity_scale": float(simulation.get("max_velocity_scale", 0.2)),
        "fallback_max_velocity": float(simulation.get("fallback_max_velocity", 1.0)),
        # The active motion backend (RoboManip or KDL) owns measured FK.
        # This guarantees exactly one state/cartesian publisher and keeps
        # mapper feedback consistent with the active solver.
        "publish_cartesian_state": not (start_robo_manip or start_kdl),
    }
    if str(arms[0].get("adapter", {}).get("package", "")) == "hc_adapter_x1":
        # The selected motion backend already rate-limits and the arbiter still
        # owns the final command. Avoid a third interpolation layer in the X1
        # visualization, which otherwise looks like control latency.
        sim_parameters["instant_position_tracking"] = bool(
            simulation.get("instant_position_tracking", True)
        )
    initial_positions = simulation.get("initial_positions", {})
    if not isinstance(initial_positions, dict):
        raise ValueError("simulation.initial_positions must be a mapping of joint name to radians")
    # Do not pass empty YAML sequences through launch_ros: Humble converts an
    # empty list to an empty tuple and rejects it as a scalar parameter.
    if initial_positions:
        sim_parameters["initial_joint_names"] = [str(name) for name in initial_positions]
        sim_parameters["initial_positions"] = [float(value) for value in initial_positions.values()]
    lease_parameters = {
        "enabled": bool(start_lease),
        "source_id": str(vr.get("session_id", "vr")),
        "vr_topic": "input/vr_frame",
        "enable_service": "safety/set_enabled",
        "acquire_service": "control/acquire",
        "lease_duration_sec": 1.0,
        "renew_margin_sec": 0.35,
    }
    diagnostics_log_directory = LaunchConfiguration(
        "diagnostics_log_directory"
    ).perform(context).strip()
    if not diagnostics_log_directory:
        diagnostics_log_directory = str(
            Path.home() / ".ros" / "hc_teleop_diagnostics" / robot_id
        )
    diagnostics_parameters = {
        "log_directory": diagnostics_log_directory,
        "diagnostics_topic": "diagnostics/control_chain",
        "vr_receive_gap_warn_ms": float(
            diagnostics.get("vr_receive_gap_warn_ms", 80.0)
        ),
        "vr_callback_delay_warn_ms": float(
            diagnostics.get("vr_callback_delay_warn_ms", 20.0)
        ),
        "ik_latency_warn_ms": float(diagnostics.get("ik_latency_warn_ms", 20.0)),
        "arbiter_latency_warn_ms": float(
            diagnostics.get("arbiter_latency_warn_ms", 20.0)
        ),
        "joint_step_warn_rad": float(diagnostics.get("joint_step_warn_rad", 0.25)),
        "missing_candidate_warn_ms": float(
            diagnostics.get("missing_candidate_warn_ms", 80.0)
        ),
        "stream_stale_ms": float(diagnostics.get("stream_stale_ms", 350.0)),
        "trace_retention_ms": float(diagnostics.get("trace_retention_ms", 2000.0)),
        "capture_pre_seconds": float(diagnostics.get("capture_pre_seconds", 5.0)),
        "capture_post_seconds": float(diagnostics.get("capture_post_seconds", 3.0)),
        "summary_period_seconds": float(
            diagnostics.get("summary_period_seconds", 5.0)
        ),
        "capture_cooldown_seconds": float(
            diagnostics.get("capture_cooldown_seconds", 5.0)
        ),
    }
    sim_package = sim_plugin = sim_executable = sim_node_name = None
    if start_sim:
        sim_package, sim_plugin, sim_executable, sim_node_name = _sim_adapter(arms)
    hardware_package = hardware_plugin = hardware_executable = hardware_node_name = None
    if start_hardware:
        hardware_package, hardware_plugin, hardware_executable, hardware_node_name = (
            _hardware_adapter(arms)
        )

    actions = []
    if start_dashboard:
        actions.append(
            _node(
                "hc_dashboard",
                "dashboard",
                "dashboard",
                namespace,
                {
                    "robot_id": robot_id,
                    "profile": profile_argument,
                    "mode": mode,
                    "host": dashboard_host,
                    "port": dashboard_port,
                },
            )
        )
    if start_rsp:
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                name="robot_state_publisher",
                parameters=[{"robot_description": Path(urdf_path).read_text(encoding="utf-8")}],
                remappings=[("joint_states", "state/joints")],
                output="screen",
            )
        )
    if start_robo_manip:
        # The private SDK remains process-isolated from the composable control
        # runtime so its pinned ABI cannot contaminate the default process.
        actions.append(
            _node(
                "hc_motion_backend_robo_manip",
                "robo_manip_backend_node",
                "robo_manip_backend",
                namespace,
                robo_manip_parameters,
            )
        )

    descriptions = []
    if start_vr:
        descriptions.append(_component("hc_vr_gateway", "hc_vr_gateway::VrGatewayNode", "vr_gateway", namespace, gateway_parameters))
    if start_arbiter:
        descriptions.append(_component("hc_teleop_core", "hc_teleop_core::CommandArbiterNode", "command_arbiter", namespace, arbiter_parameters))
    if start_lease:
        descriptions.append(_component("hc_teleop_core", "hc_teleop_core::AutoLeaseNode", "auto_lease", namespace, lease_parameters))
    if start_mapper:
        descriptions.append(_component("hc_teleop_core", "hc_teleop_core::VrMapperNode", "vr_mapper", namespace, mapper_parameters))
    if start_router:
        descriptions.append(_component("hc_motion", "hc_motion::MotionRouterNode", "motion_router", namespace, router_parameters))
    if start_kdl:
        descriptions.append(_component("hc_motion_backend_kdl", "hc_motion_backend_kdl::KdlIkBackendNode", "kdl_ik_backend", namespace, kdl_parameters))
    if start_sim:
        descriptions.append(_component(sim_package, sim_plugin, sim_node_name, namespace, sim_parameters))
    if start_hardware:
        descriptions.append(_component(
            hardware_package, hardware_plugin, hardware_node_name, namespace,
            hardware_parameters, intra_process=False))
    if start_diagnostics:
        descriptions.append(_component(
            "hc_diagnostics", "hc_diagnostics::ControlChainNode",
            "control_chain_diagnostics", namespace, diagnostics_parameters,
            # Humble rejects transient-local publishers when intra-process is enabled.
            # Diagnostics stay DDS-backed so a restarted dashboard receives the last state.
            intra_process=False))

    if composition == "compact":
        if descriptions:
            actions.append(
                ComposableNodeContainer(
                    name="hc_teleop_container",
                    namespace=namespace,
                    package="rclcpp_components",
                    executable="component_container_mt",
                    composable_node_descriptions=descriptions,
                    output="screen",
                )
            )
        return actions

    if start_vr:
        actions.append(_node("hc_vr_gateway", "hc_vr_gateway_node", "vr_gateway", namespace, gateway_parameters))
    if start_arbiter:
        actions.append(_node("hc_teleop_core", "command_arbiter_node", "command_arbiter", namespace, arbiter_parameters))
    if start_lease:
        actions.append(_node("hc_teleop_core", "auto_lease_node", "auto_lease", namespace, lease_parameters))
    if start_mapper:
        actions.append(_node("hc_teleop_core", "vr_mapper_node", "vr_mapper", namespace, mapper_parameters))
    if start_router:
        actions.append(_node("hc_motion", "motion_router_node", "motion_router", namespace, router_parameters))
    if start_kdl:
        actions.append(_node("hc_motion_backend_kdl", "kdl_ik_backend_node", "kdl_ik_backend", namespace, kdl_parameters))
    if start_sim:
        actions.append(_node(sim_package, sim_executable, sim_node_name, namespace, sim_parameters))
    if start_hardware:
        actions.append(_node(
            hardware_package, hardware_executable, hardware_node_name, namespace,
            hardware_parameters))
    if start_diagnostics:
        actions.append(_node(
            "hc_diagnostics", "control_chain_diagnostics_node",
            "control_chain_diagnostics", namespace, diagnostics_parameters))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="openarmx", description="Profile id or profile.yaml path"),
            DeclareLaunchArgument("robot_id", default_value="", description="Runtime robot id override"),
            DeclareLaunchArgument("mode", default_value="sim", description="sim, shadow, or real"),
            DeclareLaunchArgument("composition", default_value="compact", description="compact or isolated"),
            DeclareLaunchArgument(
                "motion_backend", default_value="profile",
                description="profile, kdl, robo_manip, or external"),
            DeclareLaunchArgument("start_vr", default_value="true"),
            DeclareLaunchArgument("start_arbiter", default_value="true"),
            DeclareLaunchArgument("start_vr_mapper", default_value="true"),
            DeclareLaunchArgument("start_motion_router", default_value="true"),
            DeclareLaunchArgument("start_kdl_backend", default_value="auto"),
            DeclareLaunchArgument("start_robo_manip_backend", default_value="auto"),
            DeclareLaunchArgument("start_sim_adapter", default_value="auto"),
            DeclareLaunchArgument("start_hardware_adapter", default_value="auto"),
            DeclareLaunchArgument("start_auto_lease", default_value="auto"),
            DeclareLaunchArgument("start_robot_state_publisher", default_value="auto"),
            DeclareLaunchArgument("start_dashboard", default_value="true"),
            DeclareLaunchArgument("start_diagnostics", default_value="true"),
            DeclareLaunchArgument("diagnostics_log_directory", default_value=""),
            DeclareLaunchArgument("dashboard_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("dashboard_port", default_value="7877"),
            DeclareLaunchArgument("shadow", default_value="auto", description="auto follows mode:=shadow"),
            OpaqueFunction(function=_setup),
        ]
    )
