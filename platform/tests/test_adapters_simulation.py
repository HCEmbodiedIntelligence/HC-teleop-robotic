from pathlib import Path
import time
import pytest

from hc_platform.adapters.simulation import SimulationAdapter


def test_pybullet_executes_only_fenced_fresh_named_commands(tmp_path):
    pytest.importorskip("pybullet")
    path = tmp_path / "robot.urdf"
    path.write_text(
        """<robot name="one"><link name="base"/><link name="arm"><inertial><mass value="1"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="100"/></joint></robot>"""
    )
    config = {
        "urdf_path": str(path),
        "package_root": str(tmp_path),
        "joint_names": ["j"],
        "home": [0.0],
        "groups": {"right_arm": ["j"]},
        "allowed_groups": ["right_arm"],
    }
    feedback = []
    sim = SimulationAdapter(config, feedback.append)
    sim.start()
    try:
        sim.fence("test", 1)
        sim.submit(
            {
                "boot_id": "test",
                "epoch": 1,
                "seq": 1,
                "mode": "position",
                "joint_names": ["j"],
                "positions": [0.4],
                "deadline_mono_ns": time.monotonic_ns() + 1_000_000_000,
            }
        )
        for _ in range(60):
            sim.step()
        assert feedback[-1]["positions"][0] > 0.25
        sim.fence("test", 2)
        held = sim.targets["j"]
        sim.submit(
            {
                "boot_id": "test",
                "epoch": 1,
                "seq": 2,
                "mode": "position",
                "joint_names": ["j"],
                "positions": [-0.9],
                "deadline_mono_ns": time.monotonic_ns() + 1_000_000_000,
            }
        )
        sim.step()
        assert sim.gate.reason == "STALE_FENCE" and sim.targets["j"] == pytest.approx(
            held, abs=0.01
        )
    finally:
        sim.close()


def test_repeated_stop_heartbeats_do_not_chase_gravity():
    pytest.importorskip("pybullet")
    from hc_platform.configuration import compile_package

    repo = Path(__file__).resolve().parents[2]
    config = compile_package(repo / "adapters/robots/x1")
    feedback = []
    sim = SimulationAdapter(config, feedback.append)
    sim.start()
    try:
        sim.fence("hold-test", 1)
        held = dict(sim.targets)
        for seq in range(1, 241):
            sim.submit(
                {
                    "boot_id": "hold-test",
                    "epoch": 1,
                    "seq": seq,
                    "mode": "stop",
                    "joint_names": [],
                    "positions": [],
                    "deadline_mono_ns": time.monotonic_ns() + 100_000_000,
                }
            )
            sim.step()
        assert sim.targets == held
        first = dict(zip(feedback[-1]["joint_names"], feedback[-1]["positions"]))
        for seq in range(241, 481):
            sim.submit(
                {
                    "boot_id": "hold-test",
                    "epoch": 1,
                    "seq": seq,
                    "mode": "stop",
                    "joint_names": [],
                    "positions": [],
                    "deadline_mono_ns": time.monotonic_ns() + 100_000_000,
                }
            )
            sim.step()
        second = dict(zip(feedback[-1]["joint_names"], feedback[-1]["positions"]))
        assert max(abs(second[name] - first[name]) for name in first) < 0.005
    finally:
        sim.close()


def test_openarmx_and_realman_gripper_interpolation_and_mimics():
    pytest.importorskip("pybullet")
    from hc_platform.configuration import compile_package

    repo = Path(__file__).resolve().parents[2]
    cases = [
        (
            "openarmx",
            "openarmx_left_finger_joint1",
            "openarmx_left_finger_joint2",
            0.044,
            0.0,
            0.022,
        ),
        (
            "HC_REALMAN",
            "Z_hand_narrow1_joint",
            "Z_hand_narrow2_joint",
            -0.50,
            0.0,
            -0.45,
        ),
    ]
    for robot_id, primary, mimic, open_val, closed_val, half_mimic in cases:
        config = compile_package(repo / "adapters/robots" / robot_id, robot_id)
        feedback = []
        sim = SimulationAdapter(config, feedback.append)
        sim.start()
        try:
            sim.fence("grip-test", 1)
            assert sim.targets[primary] == pytest.approx(open_val)
            sim.submit(
                {
                    "boot_id": "grip-test",
                    "epoch": 1,
                    "seq": 1,
                    "mode": "position",
                    "joint_names": config["joint_names"],
                    "positions": config["home"],
                    "grippers": [0.5, 1.0],
                    "grippers_valid": [True, True],
                    "deadline_mono_ns": time.monotonic_ns() + 1_000_000_000,
                }
            )
            sim.step()
            assert sim.targets[primary] == pytest.approx(
                (open_val + closed_val) * 0.5
            )
            assert sim.targets[mimic] == pytest.approx(half_mimic)
        finally:
            sim.close()
