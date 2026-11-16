import math
from pathlib import Path
import time

import numpy as np
import pytest

from hc_platform.motion.planner import Planner, MotionError


@pytest.fixture
def motion_config(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="xy">
<link name="base"/><link name="x_link"/><link name="tool"/>
<joint name="x" type="prismatic"><parent link="base"/><child link="x_link"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint>
<joint name="y" type="prismatic"><parent link="x_link"/><child link="tool"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint>
</robot>""")
    return {
        "robot_id": "xy",
        "control_backend": "motion",
        "config_hash": "fixture-v1",
        "urdf": str(urdf),
        "package_root": str(tmp_path),
        "joint_names": ["x", "y"],
        "lower": [-1, -1],
        "upper": [1, 1],
        "velocity": [1, 1],
        "acceleration": [2, 2],
        "home": [0, 0],
        "groups": {"right_arm": ["x", "y"]},
        "allowed_groups": ["right_arm"],
        "arms": {
            "right": {"base_link": "base", "ee_link": "tool", "joint_names": ["x", "y"]}
        },
        "motion": {
            "kinematics_backend": "python",
            "collision_enabled": False,
            "stream_hz": 50,
        },
        "verification": {"simulation": "not_verified", "hardware": "not_verified"},
    }


def test_joint_quintic_limits_and_units(motion_config):
    planner = Planner(motion_config)
    try:
        description = planner.describe()
        assert description["groups"]["right_arm"]["joints"][0]["unit"] == "m"
        result = planner.plan(
            "MoveJ",
            {
                "group": "right_arm",
                "joint_names": ["x", "y"],
                "positions": [0.2, -0.1],
                "speed_scale": 0.5,
            },
            {"x": 0, "y": 0},
        )
        values = np.array(result["positions"])
        dt = result["times"][1]
        assert np.allclose(values[0], [0, 0]) and np.allclose(values[-1], [0.2, -0.1])
        assert np.abs(np.diff(values, axis=0) / dt).max() <= 0.5 + 1e-6
        assert np.abs(np.diff(values, n=2, axis=0) / (dt * dt)).max() <= 1 + 1e-3
        assert not result["collision_checked"]
    finally:
        planner.close()


def test_cartesian_line_and_pose_point_to_point(motion_config):
    planner = Planner(motion_config)
    try:
        goal = [0.025, 0.05, 0, 0, 0, 0, -1]
        result = planner.plan(
            "MoveL",
            {
                "group": "right_arm",
                "pose": goal,
                "frame_id": "base",
                "speed_scale": 1,
                "linear_velocity": 0.1,
            },
            {"x": 0, "y": 0},
        )
        for x, y in result["positions"]:
            assert abs(y - 2 * x) < 0.002
        assert np.linalg.norm(np.array(result["positions"][-1]) - [0.025, 0.05]) < 0.002
        assert result["cartesian"]["base_frame"] == "base"
        point = planner.plan(
            "MoveP",
            {"group": "right_arm", "pose": goal, "speed_scale": 0.5},
            {"x": 0, "y": 0},
        )
        assert "cartesian" not in point
        assert np.linalg.norm(np.array(point["positions"][-1]) - [0.025, 0.05]) < 0.002
    finally:
        planner.close()


@pytest.mark.parametrize(
    "change,code",
    [
        ({"joint_names": ["x", "x"], "positions": [0, 0]}, "INVALID_GROUP_JOINTS"),
        ({"joint_names": ["x"], "positions": [2]}, "JOINT_LIMIT"),
        ({"joint_names": ["x"], "positions": [float("nan")]}, "NONFINITE_NUMBER"),
        ({"joint_names": ["bad"], "positions": [0]}, "INVALID_GROUP_JOINTS"),
    ],
)
def test_invalid_commands_are_rejected(motion_config, change, code):
    planner = Planner(motion_config)
    try:
        with pytest.raises(MotionError) as caught:
            planner.plan("MoveJ", dict(group="right_arm", **change), {"x": 0, "y": 0})
        assert caught.value.code == code
    finally:
        planner.close()


def test_servo_deadline_and_unknown_frame(motion_config):
    planner = Planner(motion_config)
    try:
        with pytest.raises(MotionError) as caught:
            planner.servo(
                "ServoJ",
                {
                    "group": "right_arm",
                    "joint_names": ["x"],
                    "positions": [0.1],
                    "deadline_mono_ns": time.monotonic_ns() - 1,
                },
                {"x": 0, "y": 0},
            )
        assert caught.value.code == "PLANNING_DEADLINE"
        with pytest.raises(MotionError) as caught:
            planner.servo(
                "ServoP",
                {
                    "group": "right_arm",
                    "frame_id": "unknown",
                    "pose": [0, 0, 0, 0, 0, 0, 1],
                },
                {"x": 0, "y": 0},
            )
        assert caught.value.code == "UNKNOWN_FRAME"
    finally:
        planner.close()


def test_motion_rejects_collision_between_valid_endpoints(tmp_path):
    pytest.importorskip("pybullet")
    path = tmp_path / "collision.urdf"
    path.write_text(
        """<robot name="collision"><link name="base"/><link name="arm"><collision><origin xyz="1 0 0"/><geometry><sphere radius=".1"/></geometry></collision></link><link name="obstacle"><collision><geometry><sphere radius=".1"/></geometry></collision></link><joint name="joint" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-3.2" upper="3.2" velocity="1" effort="10"/></joint><joint name="fixture" type="fixed"><parent link="base"/><child link="obstacle"/><origin xyz="0 1 0"/></joint></robot>"""
    )
    config = {
        "urdf": str(path),
        "package_root": str(tmp_path),
        "joint_names": ["joint"],
        "lower": [-3.2],
        "upper": [3.2],
        "velocity": [1],
        "acceleration": [2],
        "groups": {"right_arm": ["joint"]},
        "allowed_groups": ["right_arm"],
        "arms": {
            "right": {"joint_names": ["joint"], "base_link": "base", "ee_link": "arm"}
        },
        "motion": {"collision_enabled": True},
        "teleop": {
            "control": {
                "collision_min_distance": 0.01,
                "collision_ignore_graph_distance": 0,
            }
        },
    }
    planner = Planner(config)
    try:
        with pytest.raises(MotionError) as caught:
            planner.plan(
                "MoveJ",
                {
                    "group": "right_arm",
                    "joint_names": ["joint"],
                    "positions": [math.pi],
                    "speed_scale": 1,
                },
                {"joint": 0},
            )
        assert caught.value.code == "COLLISION_REJECTED"
        assert "COLLISION" in str(caught.value)
    finally:
        planner.close()


def test_raw_collision_validation_has_no_backend_bypass(tmp_path):
    pytest.importorskip("pybullet")
    path = tmp_path / "raw.urdf"
    path.write_text(
        """<robot name="raw"><link name="base"/><link name="arm"><collision><origin xyz="1 0 0"/><geometry><sphere radius=".1"/></geometry></collision></link><link name="obstacle"><collision><geometry><sphere radius=".1"/></geometry></collision></link><joint name="joint" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-3.2" upper="3.2" velocity="1" effort="10"/></joint><joint name="fixture" type="fixed"><parent link="base"/><child link="obstacle"/><origin xyz="0 1 0"/></joint></robot>"""
    )
    config = {
        "control_backend": "v23",
        "urdf": str(path),
        "package_root": str(tmp_path),
        "joint_names": ["joint"],
        "lower": [-3.2],
        "upper": [3.2],
        "velocity": [1],
        "acceleration": [2],
        "groups": {"right_arm": ["joint"]},
        "allowed_groups": ["right_arm"],
        "arms": {
            "right": {"joint_names": ["joint"], "base_link": "base", "ee_link": "arm"}
        },
        "teleop": {
            "control": {
                "collision_avoidance_enabled": True,
                "collision_ignore_graph_distance": 0,
                "collision_path_step_rad": 0.1,
            }
        },
        "motion": {"collision_enabled": False},
    }
    planner = Planner(config)
    try:
        assert (
            planner.collision_enabled
        )  # motion override cannot disable global protection
        with pytest.raises(MotionError) as caught:
            planner.validate_external(
                {
                    "joint_names": ["joint"],
                    "positions": [math.pi],
                    "deadline_mono_ns": time.monotonic_ns() + 1_000_000_000,
                },
                {"joint": 0},
            )
        assert caught.value.code == "COLLISION_REJECTED"
    finally:
        planner.close()


def test_home_path_uses_configured_dynamics(motion_config):
    planner = Planner(motion_config)
    try:
        motion_config["home_velocity"] = 0.1
        planner.config["home_velocity"] = 0.1
        result = planner.validate_home(
            {
                "joint_names": ["x", "y"],
                "positions": [0.05, 0.2],
                "deadline_mono_ns": time.monotonic_ns() + 2_000_000_000,
            },
            {"x": 0, "y": 0},
        )
        assert result["validated"] and result["samples"] > 10
        assert result["nominal_duration_s"] >= 2
    finally:
        planner.close()
