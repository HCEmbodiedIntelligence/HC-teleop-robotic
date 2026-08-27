from __future__ import annotations

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from hc_bringup.profile import load_profile, resolve_profile


def _truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _setup(context: LaunchContext):
    import os
    from pathlib import Path

    fastdds_cfg = Path(__file__).resolve().parents[3] / "config" / "fastdds_udp.xml"
    if fastdds_cfg.is_file() and "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ:
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(fastdds_cfg)

    profile = load_profile(resolve_profile(LaunchConfiguration("profile").perform(context)))
    requested_robot_id = LaunchConfiguration("robot_id").perform(context).strip()
    robot_id = requested_robot_id or profile.robot_id
    namespace = f"robots/{robot_id}"
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in {"compact", "isolated", "sim", "simulation"}:
        raise ValueError("mode must be compact, isolated, or sim")
    is_sim = mode in {"sim", "simulation"}

    start_sim_val = LaunchConfiguration("start_sim").perform(context).strip()
    start_sim = _truthy(start_sim_val) if start_sim_val else is_sim

    start_auto_lease_val = LaunchConfiguration("start_auto_lease").perform(context).strip()
    start_auto_lease = _truthy(start_auto_lease_val) if start_auto_lease_val else is_sim

    start_vr = _truthy(LaunchConfiguration("start_vr").perform(context))
    start_arbiter = _truthy(LaunchConfiguration("start_arbiter").perform(context))
    start_motion_router = _truthy(
        LaunchConfiguration("start_motion_router").perform(context)
    )
    start_vr_mapper = _truthy(LaunchConfiguration("start_vr_mapper").perform(context))
    start_legacy_bridge = _truthy(
        LaunchConfiguration("start_legacy_bridge").perform(context)
    )
    shadow_val = LaunchConfiguration("shadow").perform(context).strip()
    shadow = _truthy(shadow_val) if shadow_val else (not is_sim)

    vr = profile.value.get("vr", {})
    safety = profile.value.get("safety", {})
    teleop = profile.value.get("teleop", {})
    command_topic = (
        "shadow/control/joint_command" if shadow else "control/joint_command"
    )
    gateway_parameters = {
        "listen_host": str(vr.get("listen_host", "0.0.0.0")),
        "pose_port": int(vr.get("pose_port", 5005)),
        "discovery_port": int(vr.get("discovery_port", 5006)),
        "timeout_ms": int(vr.get("pose_timeout_ms", 150)),
        "output_topic": "input/vr_frame",
    }
    arbiter_parameters = {
        "enabled_on_start": True if is_sim else bool(safety.get("enabled_on_start", False)),
        "command_timeout_sec": float(safety.get("command_timeout_ms", 300)) / 1000.0,
        "candidate_topic": "control/joint_candidate",
        "command_topic": command_topic,
        "safety_topic": "safety/state",
    }
    motion_parameters = {
        "allowed_groups": [
            component["id"]
            for component in profile.components
            if component["kind"] == "arm" and component.get("enabled", True)
        ],
        "target_input_topic": "teleop/cartesian_targets",
        "backend_target_topic": "motion/backend/cartesian_targets",
        "backend_candidate_topic": "motion/backend/joint_candidate",
        "candidate_output_topic": "control/joint_candidate",
    }
    arm_by_id = {
        component["id"]: component
        for component in profile.components
        if component["kind"] == "arm" and component.get("enabled", True)
    }
    bindings = teleop.get("bindings", [])
    if not bindings and arm_by_id:
        bindings = [
            {"group": group, "controller": controller}
            for group, controller in zip(arm_by_id, ("left", "right"))
        ]
    axis_mapping = teleop.get(
        "axis_mapping",
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    )
    mapper_parameters = {
        "group_names": [str(item["group"]) for item in bindings],
        "controllers": [str(item["controller"]) for item in bindings],
        "base_frames": [arm_by_id[str(item["group"])]["frames"]["base"] for item in bindings],
        "tip_frames": [arm_by_id[str(item["group"])]["frames"]["tip"] for item in bindings],
        "axis_mapping": [float(value) for row in axis_mapping for value in row],
        "position_scale": float(teleop.get("position_scale", 1.0)),
        "clutch_threshold": float(teleop.get("clutch_threshold", 0.5)),
        "feedback_timeout_sec": float(teleop.get("feedback_timeout_ms", 200)) / 1000.0,
        "command_ttl_sec": float(teleop.get("command_ttl_ms", 300)) / 1000.0,
        "source_id": str(vr.get("session_id", "vr")),
        "vr_topic": "input/vr_frame",
        "cartesian_state_topic": "state/cartesian",
        "target_topic": "teleop/cartesian_targets",
    }

    comp_by_id = {component["id"]: component for component in profile.components}
    sim_groups = [
        component["id"]
        for component in profile.components
        if component.get("enabled", True)
        and component.get("joint_names")
        and component.get("frames")
    ]
    urdf_path = str(profile.resource("urdf"))
    sim_parameters = {
        "urdf_path": urdf_path,
        "group_names": sim_groups,
        "command_topic": command_topic,
        "joint_state_topic": "state/joints",
        "cartesian_state_topic": "state/cartesian",
        "publish_rate_hz": 100.0,
        "max_velocity_scale": 1.0,
    }
    for grp in sim_groups:
        comp = comp_by_id[grp]
        sim_parameters[f"{grp}.joint_names"] = comp["joint_names"]
        sim_parameters[f"{grp}.base_frame"] = comp["frames"]["base"]
        sim_parameters[f"{grp}.tip_frame"] = comp["frames"]["tip"]
    arm_groups = [
        component["id"]
        for component in profile.components
        if component["kind"] == "arm" and component.get("enabled", True)
    ]
    kdl_parameters = {
        "urdf_path": urdf_path,
        "group_names": arm_groups,
        "target_topic": "motion/backend/cartesian_targets",
        "joint_state_topic": "state/joints",
        "candidate_topic": "motion/backend/joint_candidate",
        "feedback_timeout_sec": 0.2,
        "ik_max_iterations": 80.0,
        "ik_eps": 1e-4,
    }
    for grp in arm_groups:
        kdl_parameters[f"{grp}.joint_names"] = arm_by_id[grp]["joint_names"]
        kdl_parameters[f"{grp}.base_frame"] = arm_by_id[grp]["frames"]["base"]
        kdl_parameters[f"{grp}.tip_frame"] = arm_by_id[grp]["frames"]["tip"]

    auto_lease_parameters = {
        "enabled": True,
        "source_id": str(vr.get("session_id", "vr")),
        "vr_topic": "input/vr_frame",
        "enable_service": "safety/set_enabled",
        "acquire_service": "control/acquire",
    }

    start_rviz = _truthy(LaunchConfiguration("rviz").perform(context))
    rviz_nodes = []
    if start_rviz:
        from pathlib import Path
        urdf_content = Path(urdf_path).read_text(encoding="utf-8")
        rviz_config_file = profile.path.parent / "teleop.rviz"
        rviz_args = ["-d", str(rviz_config_file)] if rviz_config_file.is_file() else []
        rviz_nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": urdf_content}],
                remappings=[("joint_states", f"{namespace}/state/joints" if namespace else "state/joints")],
                output="screen",
            )
        )
        rviz_nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=rviz_args,
                output="screen",
            )
        )

    if mode in {"compact", "sim", "simulation"}:
        components = []
        if start_vr:
            components.append(
                ComposableNode(
                    package="hc_vr_gateway",
                    plugin="hc_vr_gateway::VrGatewayNode",
                    name="vr_gateway",
                    namespace=namespace,
                    parameters=[gateway_parameters],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            )
        if start_arbiter:
            components.append(
                ComposableNode(
                    package="hc_teleop_core",
                    plugin="hc_teleop_core::CommandArbiterNode",
                    name="command_arbiter",
                    namespace=namespace,
                    parameters=[arbiter_parameters],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            )
        if start_vr_mapper:
            components.append(
                ComposableNode(
                    package="hc_teleop_core",
                    plugin="hc_teleop_core::VrMapperNode",
                    name="vr_mapper",
                    namespace=namespace,
                    parameters=[mapper_parameters],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            )
        if start_motion_router:
            components.append(
                ComposableNode(
                    package="hc_motion",
                    plugin="hc_motion::MotionRouterNode",
                    name="motion_router",
                    namespace=namespace,
                    parameters=[motion_parameters],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            )
        if start_sim:
            components.append(
                ComposableNode(
                    package="hc_motion_backend_kdl",
                    plugin="hc_motion_backend_kdl::KdlIkBackendNode",
                    name="kdl_ik_backend",
                    namespace=namespace,
                    parameters=[kdl_parameters],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            )
            if robot_id == "openarmx":
                sim_package = "hc_adapter_openarmx"
                sim_plugin = "hc_adapter_openarmx::OpenArmXSimNode"
                sim_node_name = "openarmx_sim_adapter"
            elif robot_id == "x1":
                sim_package = "hc_adapter_x1"
                sim_plugin = "hc_adapter_x1::X1SimNode"
                sim_node_name = "x1_sim_adapter"
            else:
                sim_package = f"hc_adapter_{robot_id}"
                sim_plugin = f"hc_adapter_{robot_id}::{robot_id.capitalize()}SimNode"
                sim_node_name = f"{robot_id}_sim_adapter"

            components.append(
                ComposableNode(
                    package=sim_package,
                    plugin=sim_plugin,
                    name=sim_node_name,
                    namespace=namespace,
                    parameters=[sim_parameters],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            )
        if start_auto_lease:
            components.append(
                ComposableNode(
                    package="hc_teleop_core",
                    plugin="hc_teleop_core::AutoLeaseNode",
                    name="auto_control_lease",
                    namespace=namespace,
                    parameters=[auto_lease_parameters],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            )
        if start_legacy_bridge:
            components.append(
                ComposableNode(
                    package="hc_compat_bridge",
                    plugin="hc_compat_bridge::VrLegacyBridgeNode",
                    name="vr_legacy_bridge",
                    namespace=namespace,
                    parameters=[
                        {
                            "input_topic": "input/vr_frame",
                            "output_topic": "/vrdata",
                            "joint_states_input_topic": "state/joints",
                            "joint_states_output_topic": "/hc_teleop/joint_states",
                            "joint_cmd_input_topic": "control/joint_command",
                            "joint_cmd_output_topic": "/hc_teleop/joint_cmd",
                        }
                    ],
                    extra_arguments=[{"use_intra_process_comms": False}],
                )
            )
        return [
            ComposableNodeContainer(
                name="hc_teleop_container",
                namespace=namespace,
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=components,
                output="screen",
            ),
            *rviz_nodes,
        ]

    actions = list(rviz_nodes)
    if start_vr:
        actions.append(
            Node(
                package="hc_vr_gateway",
                executable="hc_vr_gateway_node",
                namespace=namespace,
                name="vr_gateway",
                parameters=[gateway_parameters],
                output="screen",
            )
        )
    if start_arbiter:
        actions.append(
            Node(
                package="hc_teleop_core",
                executable="command_arbiter_node",
                namespace=namespace,
                name="command_arbiter",
                parameters=[arbiter_parameters],
                output="screen",
            )
        )
    if start_vr_mapper:
        actions.append(
            Node(
                package="hc_teleop_core",
                executable="vr_mapper_node",
                namespace=namespace,
                name="vr_mapper",
                parameters=[mapper_parameters],
                output="screen",
            )
        )
    if start_motion_router:
        actions.append(
            Node(
                package="hc_motion",
                executable="motion_router_node",
                namespace=namespace,
                name="motion_router",
                parameters=[motion_parameters],
                output="screen",
            )
        )
    if start_sim:
        actions.append(
            Node(
                package="hc_motion_backend_kdl",
                executable="kdl_ik_backend_node",
                namespace=namespace,
                name="kdl_ik_backend",
                parameters=[kdl_parameters],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="hc_adapter_openarmx",
                executable="openarmx_sim_node",
                namespace=namespace,
                name="openarmx_sim_adapter",
                parameters=[sim_parameters],
                output="screen",
            )
        )
    if start_auto_lease:
        actions.append(
            Node(
                package="hc_teleop_core",
                executable="auto_lease_node",
                namespace=namespace,
                name="auto_control_lease",
                parameters=[auto_lease_parameters],
                output="screen",
            )
        )
    if start_legacy_bridge:
        actions.append(
            Node(
                package="hc_compat_bridge",
                executable="vr_legacy_bridge_node",
                namespace=namespace,
                name="vr_legacy_bridge",
                parameters=[
                    {
                        "input_topic": "input/vr_frame",
                        "output_topic": "/vrdata",
                        "joint_states_input_topic": "state/joints",
                        "joint_states_output_topic": "/hc_teleop/joint_states",
                        "joint_cmd_input_topic": "control/joint_command",
                        "joint_cmd_output_topic": "/hc_teleop/joint_cmd",
                    }
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="openarmx",
                description="Installed profile id or profile.yaml path",
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value="",
                description="Optional runtime robot id override",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="compact, isolated, or sim",
            ),
            DeclareLaunchArgument("start_vr", default_value="true"),
            DeclareLaunchArgument("start_arbiter", default_value="true"),
            DeclareLaunchArgument("start_vr_mapper", default_value="true"),
            DeclareLaunchArgument("start_motion_router", default_value="true"),
            DeclareLaunchArgument(
                "start_sim",
                default_value="",
                description="Start simulation adapter and KDL backend (default true in sim mode)",
            ),
            DeclareLaunchArgument(
                "start_auto_lease",
                default_value="",
                description="Automatically acquire lease on VR input (default true in sim mode)",
            ),
            DeclareLaunchArgument(
                "start_legacy_bridge",
                default_value="false",
                description="Publish legacy /vrdata JSON for the old adapter",
            ),
            DeclareLaunchArgument(
                "shadow",
                default_value="",
                description="Publish final output below shadow/ (default false in sim, true otherwise)",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 and robot_state_publisher for 3D simulation visualization",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
