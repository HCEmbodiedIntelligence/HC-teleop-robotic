from pathlib import Path
import math
import time
import numpy as np
import pytest

from hc_platform.adapters.model import UrdfModel, matrix_quaternion, rotation
from hc_platform.adapters.collision import CollisionChecker
from hc_platform.adapters.mapping import TeleopSolver
from hc_platform.configuration import compile_package


def test_collision_checks_intermediate_path_not_only_endpoints(tmp_path):
    pytest.importorskip("pybullet")
    path = tmp_path / "collision.urdf"
    path.write_text(
        """<robot name="collision"><link name="base"/><link name="arm"><collision><origin xyz="1 0 0"/><geometry><sphere radius=".1"/></geometry></collision></link><link name="obstacle"><collision><geometry><sphere radius=".1"/></geometry></collision></link><joint name="joint" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-3.2" upper="3.2" velocity="1" effort="10"/></joint><joint name="fixture" type="fixed"><parent link="base"/><child link="obstacle"/><origin xyz="0 1 0"/></joint></robot>"""
    )
    model = UrdfModel(path, tmp_path)
    config = {
        "joint_names": ["joint"],
        "arms": {"right": {"joint_names": ["joint"]}},
        "teleop": {
            "control": {
                "collision_min_distance": 0.01,
                "collision_ignore_graph_distance": 0,
                "collision_path_step_rad": 0.1,
            }
        },
    }
    checker = CollisionChecker(model, config)
    try:
        assert checker.check_path(
            {"joint": 0}, {"joint": 0.2}, time.monotonic_ns() + 1_000_000_000
        ).safe
        blocked = checker.check_path(
            {"joint": 0}, {"joint": math.pi}, time.monotonic_ns() + 1_000_000_000
        )
        assert (
            not blocked.safe
            and blocked.reason == "COLLISION_BLOCKED"
            and blocked.samples > 1
        )
        assert not checker.check_path(
            {}, {"joint": 0}, time.monotonic_ns() + 1_000_000_000
        ).safe
        assert (
            checker.check_path(
                {"joint": 0}, {"joint": 0.2}, time.monotonic_ns() - 1
            ).reason
            == "COLLISION_DEADLINE"
        )
    finally:
        checker.close()


def test_neck_enabled_without_configuration_fails_closed():
    repo = Path(__file__).resolve().parents[2]
    config = compile_package(repo / "adapters/robots/x1")
    config["teleop"]["body"]["neck_tracking_enabled"] = True
    with pytest.raises(ValueError, match="neck tracking"):
        TeleopSolver(config)


def test_optional_neck_respects_limits_and_body_ownership():
    repo = Path(__file__).resolve().parents[2]
    config = compile_package(repo / "adapters/robots/x1")
    body = config["teleop"]["body"]
    body.update(
        neck_tracking_enabled=True,
        neck_joint_names=["head_1", "head_2"],
        neck_yaw_joint="head_1",
        neck_pitch_joint="head_2",
        neck_max_yaw=0.25,
    )
    names = config["joint_names"] + ["head_1", "head_2"]
    q = config["home"] + [0.0, 0.0]
    solver = TeleopSolver(config)
    rotation_q = matrix_quaternion(rotation([0, 1, 0], 0.5)).tolist()
    request = {
        "boot_id": "neck-test",
        "epoch": 1,
        "lease_id": 1,
        "reference_epoch": 1,
        "seq": 1,
        "joint_names": names,
        "positions": q,
        "reference_positions_left": q,
        "reference_positions_right": q,
        "active_arms": {"left": False, "right": False},
        "allow_body": True,
        "head_pose": [0, 1, 0] + rotation_q,
        "head_reference_pose": [0, 1, 0, 0, 0, 0, 1],
        "deadline_mono_ns": time.monotonic_ns() + 500_000_000,
    }
    try:
        result = solver.solve(request)
        assert result is not None
        out = dict(zip(result["joint_names"], result["positions"]))
        assert abs(out["head_1"]) == pytest.approx(0.25)
        assert abs(out["head_2"]) <= 0.8
        request["allow_body"] = False
        assert solver.solve(request) is None
    finally:
        solver.close()
