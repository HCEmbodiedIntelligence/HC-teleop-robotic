#!/usr/bin/env python3
"""Unified teleoperation and simulation launcher for HC-teleop-robotic."""

from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_resource(value: str, profile_dir: Path, label: str) -> Path:
    if value.startswith("package://"):
        package_path = value[len("package://"):]
        package, _, relative = package_path.partition("/")
        package_share = Path(get_package_share_directory(package)).resolve()
        resolved = (package_share / relative).resolve()
    elif Path(value).is_absolute():
        resolved = Path(value).resolve()
    else:
        resolved = (profile_dir / value).resolve()

    if not resolved.is_file():
        raise RuntimeError(f"Resource '{label}' does not exist: {resolved}")
    return resolved


def _setup(context):
    profile_name = LaunchConfiguration("profile").perform(context).strip().lower()
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    rviz_sim = LaunchConfiguration("rviz_sim").perform(context).strip().lower()
    headless = LaunchConfiguration("headless").perform(context).strip().lower()
    start_driver = LaunchConfiguration("start_driver").perform(context).strip().lower() in {"1", "true", "yes", "on"}
    start_motion = LaunchConfiguration("start_motion").perform(context).strip().lower() in {"1", "true", "yes", "on"}
    start_teleop = LaunchConfiguration("start_teleop").perform(context).strip().lower() in {"1", "true", "yes", "on"}
    start_rsp = LaunchConfiguration("start_rsp").perform(context).strip().lower() in {"1", "true", "yes", "on"}

    # Resolve package and profile directory
    known_packages = {
        "openarmx": "hc_robot_openarmx",
        "openarmx_v10": "hc_robot_openarmx",
        "x1": "hc_robot_x1",
    }
    robot_package = known_packages.get(profile_name, f"hc_robot_{profile_name}")
    try:
        package_share = Path(get_package_share_directory(robot_package)).resolve()
    except PackageNotFoundError:
        # Fallback to in-tree src if not yet installed in share
        script_dir = Path(__file__).resolve().parent.parent
        in_tree = script_dir / "src" / robot_package
        if in_tree.is_dir():
            package_share = in_tree
        else:
            raise RuntimeError(f"Could not locate package '{robot_package}' for profile '{profile_name}'")

    profile_dir = package_share / "config" / "humanoid_stack" / profile_name
    manifest_path = profile_dir / "profile.yaml"
    if not manifest_path.is_file():
        # Check if profile dir is under default name
        candidates = list((package_share / "config" / "humanoid_stack").glob("*/profile.yaml"))
        if candidates:
            manifest_path = candidates[0]
            profile_dir = manifest_path.parent
        else:
            raise RuntimeError(f"Profile configuration not found for '{profile_name}' at: {manifest_path}")

    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)

    resources = manifest.get("resources", {})
    resolved_paths = {
        key: _resolve_resource(val, profile_dir, key)
        for key, val in resources.items()
    }

    actions = []

    # 1. Driver Runtime Node (Mock in sim mode)
    if start_driver:
        driver_params_path = resolved_paths.get("driver_params")
        actions.append(
            Node(
                package="humanoid_driver_runtime",
                executable="humanoid_driver_runtime_node",
                name="humanoid_driver_runtime",
                output="screen",
                parameters=[str(driver_params_path)],
            )
        )

    # 2. Motion Server Node (RoboManip kinematics, limits, emergency stop)
    if start_motion:
        motion_params_path = resolved_paths.get("motion_params")
        actions.append(
            Node(
                package="humanoid_motion_server",
                executable="humanoid_motion_control_node",
                name="humanoid_motion_control",
                output="screen",
                parameters=[
                    str(motion_params_path),
                    {
                        "channel_config_file": str(resolved_paths["channel_config"]),
                        "sdk_config_file": str(resolved_paths["sdk_config"]),
                        "tool_config_file": str(resolved_paths["tool_config"]),
                        "urdf_file": str(resolved_paths["urdf"]),
                    },
                ],
            )
        )

    # 3. Teleop Receiver Node (PICO VR frontend & emergency stop)
    if start_teleop and "teleop_config" in resolved_paths:
        actions.append(
            Node(
                package="hc_teleop_recv",
                executable="hc_teleop_recv_node",
                name="hc_teleop_recv",
                output="screen",
                parameters=[{"config_file": str(resolved_paths["teleop_config"])}],
            )
        )

    # 4. Robot State Publisher (URDF -> /robot_description & TF transforms)
    urdf_path = resolved_paths.get("urdf")
    if start_rsp and urdf_path and urdf_path.is_file():
        urdf_text = urdf_path.read_text(encoding="utf-8")
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": urdf_text}],
                remappings=[("joint_states", "/hc_teleop/joint_states")],
            )
        )

    # 5. RViz Visualization
    show_rviz = (rviz_sim in {"1", "true", "yes", "on"}) and (headless not in {"1", "true", "yes", "on"})
    if show_rviz:
        rviz_config = package_share / "config" / "teleop.rviz"
        if not rviz_config.is_file():
            rviz_config = profile_dir / "teleop.rviz"
        if rviz_config.is_file():
            actions.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz_sim",
                    arguments=["-d", str(rviz_config)],
                    output="screen",
                )
            )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="openarmx", description="Robot profile (openarmx, x1)"),
        DeclareLaunchArgument("mode", default_value="sim", description="sim, real, or shadow"),
        DeclareLaunchArgument("rviz_sim", default_value="true", description="Launch RViz with visualization"),
        DeclareLaunchArgument("headless", default_value="false", description="Run without GUI"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("start_motion", default_value="true"),
        DeclareLaunchArgument("start_teleop", default_value="true"),
        DeclareLaunchArgument("start_rsp", default_value="true"),
        OpaqueFunction(function=_setup),
    ])

