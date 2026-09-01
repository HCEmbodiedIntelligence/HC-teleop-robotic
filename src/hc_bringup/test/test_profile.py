from pathlib import Path

import pytest
import yaml

from hc_bringup.profile import ProfileError, load_profile, resolve_resource, validate_profile


def minimal_profile():
    return {
        "schema": "hc-teleop-profile/v1",
        "robot_id": "test_robot",
        "display_name": "Test Robot",
        "resources": {"urdf": "urdf/robot.urdf"},
        "components": [
            {
                "id": "left_arm",
                "kind": "arm",
                "adapter": {"package": "test_adapter", "plugin": "TestArm"},
                "joint_names": ["joint_1"],
                "frames": {"base": "base", "tip": "tool"},
                "command_modes": ["servo_p"],
            }
        ],
    }


def test_accepts_zero_one_or_two_arms():
    value = minimal_profile()
    assert len(validate_profile(value)["components"]) == 1
    value["components"].append(
        {
            "id": "right_arm",
            "kind": "arm",
            "adapter": {"package": "test_adapter", "plugin": "TestArm"},
            "joint_names": ["joint_2"],
            "frames": {"base": "base", "tip": "tool_2"},
        }
    )
    assert len(validate_profile(value)["components"]) == 2


def test_rejects_third_arm_and_invalid_attachment():
    value = minimal_profile()
    for number in (2, 3):
        value["components"].append(
            {
                "id": f"arm_{number}",
                "kind": "arm",
                "adapter": {"package": "test_adapter", "plugin": "TestArm"},
                "joint_names": [f"joint_{number}"],
                "frames": {"base": "base", "tip": f"tool_{number}"},
            }
        )
    with pytest.raises(ProfileError, match="at most two"):
        validate_profile(value)

    value = minimal_profile()
    value["components"].append(
        {
            "id": "hand",
            "kind": "dexterous_hand",
            "attached_to": "missing_arm",
            "adapter": {"package": "hand_adapter", "plugin": "Hand"},
            "joint_names": ["finger"],
        }
    )
    with pytest.raises(ProfileError, match="existing arm"):
        validate_profile(value)


def test_rejects_absolute_topics_and_resource_escape(tmp_path: Path):
    value = minimal_profile()
    value["components"].append(
        {
            "id": "head_camera",
            "kind": "camera",
            "adapter": {"package": "camera_adapter", "executable": "camera"},
            "frame": "camera_link",
            "image_topic": "/absolute/image",
        }
    )
    with pytest.raises(ProfileError, match="must be relative"):
        validate_profile(value)
    with pytest.raises(ProfileError, match="escapes"):
        resolve_resource("../secret", tmp_path)
    with pytest.raises(ProfileError, match="package:// or profile-relative"):
        resolve_resource("/tmp/robot.urdf", tmp_path)


def test_loads_yaml_and_resolves_package_resource(tmp_path: Path):
    value = minimal_profile()
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(value), encoding="utf-8")
    profile = load_profile(profile_path)
    assert profile.robot_id == "test_robot"

    package = tmp_path / "package"
    target = package / "urdf" / "robot.urdf"
    target.parent.mkdir(parents=True)
    target.write_text("<robot/>", encoding="utf-8")
    resolved = resolve_resource(
        "package://test_description/urdf/robot.urdf",
        tmp_path,
        package_lookup=lambda name: str(package) if name == "test_description" else "",
    )
    assert resolved == target


def test_validates_vr_bindings_and_axis_mapping():
    value = minimal_profile()
    value["teleop"] = {
        "bindings": [{"group": "left_arm", "controller": "left"}],
        "axis_mapping": [[0, 0, -1], [-1, 0, 0], [0, 1, 0]],
        "position_scale": 0.8,
    }
    assert validate_profile(value)["teleop"]["bindings"][0]["group"] == "left_arm"
    value["teleop"]["bindings"][0]["group"] = "missing"
    with pytest.raises(ProfileError, match="not an enabled arm"):
        validate_profile(value)

    value = minimal_profile()
    value["teleop"] = {"axis_mapping": [[1, 0], [0, 1]]}
    with pytest.raises(ProfileError, match="3x3"):
        validate_profile(value)

    value = minimal_profile()
    value["teleop"] = {
        "bindings": [
            {
                "group": "left_arm",
                "controller": "left",
                "axis_mapping": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            }
        ]
    }
    assert validate_profile(value)["teleop"]["bindings"][0]["axis_mapping"][0] == [1, 0, 0]
    value["teleop"]["bindings"][0]["axis_mapping"] = [[1, 0], [0, 1]]
    with pytest.raises(ProfileError, match=r"bindings\[0\]\.axis_mapping.*3x3"):
        validate_profile(value)


def test_rejects_non_positive_diagnostics_threshold():
    value = minimal_profile()
    value["diagnostics"] = {"ik_latency_warn_ms": 0.0}
    with pytest.raises(ProfileError, match="diagnostics.ik_latency_warn_ms"):
        validate_profile(value)


def test_validates_optional_robo_manip_backend_contract():
    value = minimal_profile()
    value["motion"] = {"backend_package": "hc_motion_backend_robo_manip"}
    with pytest.raises(ProfileError, match="resources.robo_manip_sdk"):
        validate_profile(value)

    value["resources"]["robo_manip_sdk"] = "motion/robo_manip.yaml"
    value["motion"]["robo_manip_joint_max_velocity_rad_s"] = 0.6
    assert validate_profile(value)["motion"]["backend_package"] == (
        "hc_motion_backend_robo_manip"
    )

    value["motion"]["robo_manip_joint_max_velocity_rad_s"] = 0.0
    with pytest.raises(ProfileError, match="motion.robo_manip_joint_max_velocity_rad_s"):
        validate_profile(value)


def test_validates_robo_manip_fault_tolerance_and_ik_tuning():
    value = minimal_profile()
    value["motion"] = {
        "robo_manip_tick_failure_reset_count": 3,
        "robo_manip_ik_orientation_tolerance_rad": 0.015,
        "robo_manip_ik_enable_regularization_task": True,
        "robo_manip_ik_regularization_task_weight": 0.0005,
        "robo_manip_ik_enable_joint_task": True,
        "robo_manip_ik_joint_task_weight": 0.001,
    }
    assert validate_profile(value)["motion"][
        "robo_manip_tick_failure_reset_count"
    ] == 3

    value["motion"]["robo_manip_tick_failure_reset_count"] = 1
    with pytest.raises(ProfileError, match="integer >= 2"):
        validate_profile(value)


def test_validates_arm_target_filter():
    value = minimal_profile()
    value["components"][0]["target_filter"] = {
        "workspace_min_m": [-0.1, -0.5, -0.4],
        "workspace_max_m": [0.7, 0.5, 0.7],
        "min_radius_m": 0.1,
        "max_radius_m": 0.75,
        "max_position_step_m": 0.03,
        "max_orientation_step_rad": 0.15,
    }
    validated = validate_profile(value)
    assert validated["components"][0]["target_filter"]["enabled"] is True

    value["components"][0]["target_filter"]["workspace_max_m"][0] = -0.2
    with pytest.raises(ProfileError, match="workspace bounds"):
        validate_profile(value)


def test_validates_simulation_only_robo_manip_limits():
    value = minimal_profile()
    value["simulation"] = {
        "robo_manip_limits": {
            "joint_max_velocity_rad_s": 1.5,
            "cartesian_max_linear_velocity_m_s": 0.5,
        }
    }
    validated = validate_profile(value)
    assert validated["simulation"]["robo_manip_limits"][
        "joint_max_velocity_rad_s"
    ] == 1.5

    value["simulation"]["robo_manip_limits"]["joint_max_velocity_rad_s"] = 0.0
    with pytest.raises(ProfileError, match="simulation.robo_manip_limits"):
        validate_profile(value)

    value["simulation"]["robo_manip_limits"] = {"unknown_limit": 1.0}
    with pytest.raises(ProfileError, match="unknown simulation.robo_manip_limits"):
        validate_profile(value)
