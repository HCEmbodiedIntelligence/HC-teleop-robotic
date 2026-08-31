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
