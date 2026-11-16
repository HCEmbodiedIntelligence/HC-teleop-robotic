from pathlib import Path
import time
import numpy as np
import pytest

from hc_platform.adapters.backend import OutputGate, X1Topics, map_gripper
from hc_platform.adapters.model import (
    UrdfModel,
    matrix_quaternion,
    quaternion_matrix,
    relative_target,
    rotation,
)


ARM = """<robot name="test"><link name="base"/><link name="a"/><link name="b"/><link name="tip"/>
<joint name="j1" type="revolute"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="20"/></joint>
<joint name="j2" type="revolute"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="20"/></joint>
<joint name="tool" type="fixed"><parent link="b"/><child link="tip"/><origin xyz="1 0 0"/></joint></robot>"""


@pytest.fixture
def model(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text(ARM)
    return UrdfModel(path, tmp_path)


def test_fk_and_analytic_jacobian(model):
    q = {"j1": 0.2, "j2": -0.3}
    fk = model.fk(q, "tip")
    assert fk[:3, 3] == pytest.approx(
        [np.cos(0.2) + np.cos(-0.1), np.sin(0.2) + np.sin(-0.1), 0]
    )
    jac = model.jacobian(q, "tip", ["j1", "j2"])
    for index, name in enumerate(("j1", "j2")):
        changed = dict(q)
        changed[name] += 1e-6
        numeric = (model.fk(changed, "tip")[:3, 3] - fk[:3, 3]) / 1e-6
        assert jac[:3, index] == pytest.approx(numeric, abs=2e-6)


def test_ik_returns_named_converged_solution(model):
    target = model.fk({"j1": 0.25, "j2": -0.35}, "tip")
    result = model.ik(
        target,
        {"j1": 0.0, "j2": 0.0},
        ["j1", "j2"],
        "tip",
        timeout_ms=200,
        max_iterations=80,
    )
    assert result.converged
    assert result.positions["j1"] == pytest.approx(0.25, abs=0.02)
    assert result.positions["j2"] == pytest.approx(-0.35, abs=0.03)


def test_unreachable_target_is_not_reported_as_success(model):
    target = np.eye(4)
    target[:3, 3] = [10, 10, 0]
    result = model.ik(
        target,
        {"j1": 0.0, "j2": 0.0},
        ["j1", "j2"],
        "tip",
        max_iterations=2,
        timeout_ms=100,
    )
    assert not result.converged and result.reason == "ITERATION_LIMIT"


def test_urdf_entities_missing_meshes_and_cycles_rejected(tmp_path):
    path = tmp_path / "robot.urdf"
    for text in (
        '<!DOCTYPE robot SYSTEM "file:///etc/passwd">' + ARM,
        ARM.replace(
            'name="base"/>',
            'name="base"><visual><geometry><mesh filename="../../no.stl"/></geometry></visual></link>',
        ),
        ARM.replace('<parent link="base"/>', '<parent link="tip"/>'),
    ):
        path.write_text(text)
        with pytest.raises(ValueError):
            UrdfModel(path, tmp_path)


def test_handedness_mapping_uses_conjugation():
    mapping = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]])
    assert np.linalg.det(mapping) == -1
    pose = [1, 2, 3, 0, 0, 0, 1]
    target = relative_target(
        np.eye(4), pose, [0, 0, 0, 0, 0, 0, 1], mapping, max_displacement=100
    )
    assert target[:3, 3] == pytest.approx([3, -1, 2])
    assert np.linalg.det(target[:3, :3]) == pytest.approx(1)


def test_rotation_quaternion_roundtrip_near_pi():
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 2, 3]):
        original = rotation(axis, np.pi - 1e-8)
        assert quaternion_matrix(matrix_quaternion(original)) == pytest.approx(
            original, abs=1e-8
        )


def test_real_x1_model_loads_names_without_index_assumptions():
    repo = Path(__file__).resolve().parents[2]
    model = UrdfModel(repo / "adapters/robots/x1/urdf/hc_tj_robot.urdf")
    assert len(model.links) == 49 and len(model.joints) == 48
    assert "Joint7_R" in model.chain("io_teleop_ee_link_right", "zhi_Link")
    assert np.isfinite(model.fk({}, "io_teleop_ee_link_right", "zhi_Link")).all()


def test_output_fencing_deadline_and_authorization():
    gate = OutputGate(["j1"])
    gate.fence("boot", 3)
    command = {
        "boot_id": "boot",
        "epoch": 3,
        "seq": 1,
        "deadline_mono_ns": 100,
        "mode": "position",
        "joint_names": ["j1"],
        "positions": [0.1],
    }
    assert gate.validate(command, 50)
    assert not gate.validate(command, 51) and gate.reason == "STALE_SEQUENCE"
    assert not gate.validate(dict(command, seq=2, epoch=2), 52)
    assert (
        not gate.validate(dict(command, seq=2), 101) and gate.reason == "EXPIRED_OUTPUT"
    )
    assert (
        not gate.validate(dict(command, seq=2, joint_names=["j2"]), 60)
        and gate.reason == "GROUP_NOT_ALLOWED"
    )
    assert not gate.validate(dict(command, seq=2, positions=[float("nan")]), 60)


def test_topics_never_publish_both_contracts():
    assert X1Topics.for_contract("legacy-model-topic").command == "/action/joint_states"
    assert (
        X1Topics.for_contract("internal-model-v2").command
        == "/internal/robot/joint_commands"
    )
    with pytest.raises(ValueError):
        X1Topics.for_contract("auto")


def test_gripper_calibration_interpolation():
    names, values = map_gripper(
        {
            "finger_joint_names": ["a", "b"],
            "finger_open": [0.0, 1.0],
            "finger_closed": [2.0, 3.0],
        },
        0.5,
    )
    assert names == ["a", "b"] and values == [1.0, 2.0]
    with pytest.raises(ValueError):
        map_gripper(
            {"finger_joint_names": ["a"], "finger_open": [], "finger_closed": [2.0]},
            0.5,
        )


def test_native_matches_reference_fk_jacobian_and_ik():
    repo = Path(__file__).resolve().parents[2]
    path = repo / "adapters/robots/x1/urdf/hc_tj_robot.urdf"
    if not (repo / "platform/kinematics/build/libhc_kinematics.so").exists():
        pytest.skip("build native kinematics before parity test")
    native = UrdfModel(path, backend="native")
    reference = UrdfModel(path, backend="python")
    group = ["Joint%d_R" % i for i in range(1, 8)]
    random = np.random.default_rng(123)
    for _ in range(8):
        q = {name: float(random.uniform(-0.3, 0.3)) for name in group}
        assert native.fk(q, "io_teleop_ee_link_right", "zhi_Link") == pytest.approx(
            reference.fk(q, "io_teleop_ee_link_right", "zhi_Link"), abs=1e-10
        )
        assert native.jacobian(
            q, "io_teleop_ee_link_right", group, "zhi_Link"
        ) == pytest.approx(
            reference.jacobian(q, "io_teleop_ee_link_right", group, "zhi_Link"),
            abs=1e-10,
        )
    seed = dict(
        zip(
            group, [-1.396263, -0.523599, 1.745329, -1.919862, -0.261799, 0.174533, 0.0]
        )
    )
    target = native.fk(seed, "io_teleop_ee_link_right", "zhi_Link")
    target[2, 3] += 0.02
    a = native.ik(
        target, seed, group, "io_teleop_ee_link_right", "zhi_Link", timeout_ms=200
    )
    b = reference.ik(
        target, seed, group, "io_teleop_ee_link_right", "zhi_Link", timeout_ms=200
    )
    assert a.converged and b.converged
    assert list(a.positions.values()) == pytest.approx(
        list(b.positions.values()), abs=1e-7
    )


def test_all_bundled_robots_load_models_and_validate_urdf_invariants(tmp_path):
    from hc_platform.configuration import compile_package

    repo = Path(__file__).resolve().parents[2]
    for robot_id, min_joints in (("x1", 17), ("openarmx", 14), ("HC_REALMAN", 12)):
        config = compile_package(repo / "adapters/robots" / robot_id, robot_id)
        model = UrdfModel(config["urdf_path"], config["package_root"])
        assert len(config["joint_names"]) >= min_joints
        for side, arm in config["arms"].items():
            fk = model.fk({}, arm["ee_link"], arm["base_link"])
            assert np.isfinite(fk).all(), (robot_id, side)
    (tmp_path / "m.stl").write_text("")
    bad = tmp_path / "bad.urdf"
    for mutated in (
        ARM.replace(
            '<link name="base"/>',
            '<link name="base"><visual><geometry><mesh filename="m.stl" scale="1 0 1"/></geometry></visual></link>',
        ),
        ARM.replace('<axis xyz="0 0 1"/>', '<axis xyz="0 0 0"/>'),
        ARM.replace('velocity="1"', 'velocity="-1"'),
    ):
        bad.write_text(mutated)
        with pytest.raises(ValueError):
            UrdfModel(bad, tmp_path)
