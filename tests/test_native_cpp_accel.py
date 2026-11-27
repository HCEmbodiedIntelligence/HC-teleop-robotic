"""Numerical parity, ROS 2 E2E, and zero-dependency tests for C++17 accelerations."""
from __future__ import annotations

import json
import os
import random
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

PROJECT = Path(__file__).resolve().parents[1]
V23_SRC = PROJECT / "adapters" / "v23" / "src"
V23_CPP_BIN = PROJECT / "adapters" / "v23" / "cpp" / "build" / "controller_v2_3_ros2_cpp"

if str(V23_SRC) not in sys.path:
    sys.path.insert(0, str(V23_SRC))


def test_v23_cpp_core_matches_python_across_profiles():
    import _v23_ik_cpp as v23_cpp
    from controller_v2_3 import ControllerV23, TargetTransform
    from solve_ik import _solve_box_lsq

    # 1. Active-set box LSQ parity
    rng = np.random.default_rng(42)
    for _ in range(10):
        A = rng.normal(size=(20, 14))
        b = rng.normal(size=(20,))
        lo = -rng.uniform(0.1, 1.0, size=(14,))
        hi = rng.uniform(0.1, 1.0, size=(14,))
        v_py = _solve_box_lsq(A, b, 5e-3, lo, hi)
        v_cpp = v23_cpp.solve_box_lsq(A, b, 5e-3, lo, hi)
        np.testing.assert_allclose(v_cpp, v_py, atol=1e-10, rtol=1e-10)

    # 2. Multi-step retarget IK parity on X1 (with acceleration limits + posture)
    #    and OpenArmX (without acceleration limits)
    x1_init = {
        "Joint1_R": -1.396, "Joint2_R": -0.52, "Joint3_R": 1.74,
        "Joint4_R": -1.91, "Joint5_R": -0.26, "Joint6_R": 0.17, "Joint7_R": 0.0,
        "Joint1_L": 1.396, "Joint2_L": -0.52, "Joint3_L": -1.74,
        "Joint4_L": -1.91, "Joint5_L": 0.26, "Joint6_L": 0.17, "Joint7_L": 0.0,
    }
    for rel_cfg, init_map in (
        ("adapters/robots/x1/controller_v23.yml", x1_init),
        ("adapters/robots/openarmx/controller_v23.yml", None),
    ):
        cfg_path = PROJECT / rel_cfg
        c_py = ControllerV23(cfg_path)
        c_cpp = v23_cpp.ControllerV23Cpp(str(cfg_path))
        assert list(c_cpp.free_joint_names) == c_py.free_joint_names

        names = c_py.free_joint_names
        if init_map is not None:
            q0 = [init_map[n] for n in names]
        else:
            q0 = [0.0] * len(names)
        c_py.update_joint_state(names, q0)
        c_cpp.update_joint_state(names, q0)

        # Verify relative pose & relative Jacobian parity
        targets_py = {}
        targets_cpp = {}
        for task in c_py.pose_tasks:
            pose_py = c_py.interface.relative_pose(task.root, task.frame, c_py.q)
            t_cpp, r_cpp = c_cpp.relative_pose(task.root, task.frame)
            np.testing.assert_allclose(t_cpp, pose_py.translation, atol=1e-12)
            np.testing.assert_allclose(r_cpp, pose_py.rotation, atol=1e-12)

            j_py = c_py.interface.relative_jacobian(task.root, task.frame, c_py.q)
            j_cpp = c_cpp.relative_jacobian(task.root, task.frame)
            np.testing.assert_allclose(j_cpp, j_py, atol=1e-11)

            delta = np.array([0.04, -0.02, 0.03])
            targets_py[(task.root, task.frame)] = TargetTransform(
                pose_py.translation + delta, pose_py.rotation.copy()
            )
            targets_cpp[(task.root, task.frame)] = v23_cpp.TargetTransform(
                pose_py.translation + delta, pose_py.rotation.copy()
            )

        c_py.update_tf_targets(targets_py)
        c_cpp.update_tf_targets(targets_cpp)

        # Verify 100 steps of closed-loop IK match to < 1e-9 rad
        for _ in range(100):
            n_py, p_py = c_py.step()
            n_cpp, p_cpp = c_cpp.step()
            assert list(n_cpp) == n_py
            np.testing.assert_allclose(p_cpp, p_py, atol=1e-9, rtol=1e-9)
            c_py.update_joint_state(n_py, p_py)
            c_cpp.update_joint_state(n_cpp, p_cpp)

        # Verify reset_to_feedback parity
        c_py.reset_to_feedback()
        c_cpp.reset_to_feedback()
        np.testing.assert_allclose(c_cpp.command_q, c_py.command_q, atol=1e-12)


def test_rtp_fec_cpp_parity_with_python():
    from middleware.core.media import rtp_fec

    assert rtp_fec.USING_NATIVE_EXT is True
    rng = random.Random(1234)
    ssrc = 0x11223344
    packets = []
    for i in range(10):
        seq = (65532 + i) & 0xFFFF
        payload_len = rng.randint(50, 1180)
        ext = bytes.fromhex("bede000132aabbcc") if i % 2 == 0 else b""
        first = 0x90 if ext else 0x80
        second = 0xE0 if i == 9 else 0x60
        pkt = (
            struct.pack("!BBHII", first, second, seq, 90000 + i * 3000, ssrc)
            + ext
            + rng.randbytes(payload_len)
        )
        packets.append(pkt)

    fec_cpp = rtp_fec.encode_ulpfec(packets, mutable_extension_ids=[3])
    fec_py = rtp_fec._py_encode_ulpfec(packets, mutable_extension_ids=[3])
    assert fec_cpp == fec_py
    assert rtp_fec.protected_sequences(fec_cpp) == rtp_fec._py_protected_sequences(fec_py)

    for drop_idx in (0, 4, 9):
        subset = [p for idx, p in enumerate(packets) if idx != drop_idx]
        rec_cpp = rtp_fec.recover_ulpfec(fec_cpp, subset, ssrc, mutable_extension_ids=[3])
        rec_py = rtp_fec._py_recover_ulpfec(fec_py, subset, ssrc, mutable_extension_ids=[3])
        assert rec_cpp == rec_py
        assert rec_cpp == rtp_fec._py_zero_mutable_extensions(packets[drop_idx], [3])

    for pkt in packets:
        assert rtp_fec.rtp_layout(pkt) == rtp_fec._py_rtp_layout(pkt)
        assert rtp_fec.red_wrap(pkt, 116) == rtp_fec._py_red_wrap(pkt, 116)
        assert rtp_fec.red_unwrap(rtp_fec.red_wrap(pkt, 116)) == pkt


def test_recording_cdr_cpp_parity_and_isolation():
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from middleware.core.recording import recording_cdr

    assert recording_cdr.USING_NATIVE_EXT is True
    for count in (0, 1, 7, 16, 20):
        msg = JointState(
            name=[f"Joint{i}_R" for i in range(count)],
            position=[0.1 * i - 0.5 for i in range(count)],
        )
        msg.header.stamp.sec = 1795654056
        msg.header.stamp.nanosec = 123456789
        msg.header.frame_id = "zhi_Link"
        raw = bytes(serialize_message(msg))

        assert recording_cdr.read_joint_state(raw) == recording_cdr._py_read_joint_state(raw)
        assert recording_cdr.trim_joint_state_cdr(raw) == recording_cdr._py_trim_joint_state_cdr(raw)
        dec = recording_cdr.read_joint_state(raw)
        assert (
            recording_cdr.position_label(dec.header_prefix, dec.name, dec.position)
            == recording_cdr._py_position_label(dec.header_prefix, dec.name, dec.position)
        )

    # Confirm zero numpy / rclpy imports when loading _recording_cdr_cpp
    code = (
        "import sys; from middleware.core.recording import recording_cdr; "
        "assert recording_cdr.USING_NATIVE_EXT is True; "
        "raw = recording_cdr.serialize_joint_state(10, 20, 'base', ['j1'], [0.5]); "
        "assert recording_cdr.read_joint_state(raw).position == (0.5,); "
        "assert 'numpy' not in sys.modules and 'rclpy' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], cwd=PROJECT, check=True, timeout=10)


def test_serialize_joint_state_cpp_bit_exact_with_rclpy_and_snapshot():
    import array
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import JointState
    from middleware.core.recording import recording_cdr
    from middleware.core.recording.publication_capture import JointSnapshot, serialize_snapshot

    assert recording_cdr.USING_NATIVE_EXT is True
    cases = [
        (0, 0, "", [], [], [], []),
        (1795654056, 123456789, "zhi_Link", ["Joint1_R"], [0.25], [], []),
        (1795654056, 987654321, "base_α", ["Joint1_R", "Joint2_L", "leg_1"], [0.1, -0.2, 0.3], [0.01, 0.02, -0.03], []),
        (42, 7, "x", [f"J{i}" for i in range(19)], [0.05 * i for i in range(19)], [0.0] * 19, [1.5] * 19),
    ]
    for sec, nsec, fid, names, pos, vel, eff in cases:
        msg = JointState(name=names, position=pos, velocity=vel, effort=eff)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = fid
        expected = recording_cdr.trim_joint_state_cdr(bytes(serialize_message(msg)))

        # Test list/tuple inputs and array.array('d') buffer fast-path (ROS JointState fields)
        got_cpp = recording_cdr.serialize_joint_state(sec, nsec, fid, msg.name, msg.position, msg.velocity, msg.effort)
        got_cpp_arr = recording_cdr.serialize_joint_state(
            sec, nsec, fid, tuple(names), array.array("d", pos), tuple(vel), list(eff)
        )
        got_py = recording_cdr._py_serialize_joint_state(sec, nsec, fid, names, pos, vel, eff)
        # C++ and pure-Python produce bit-identical zero-padded CDR1 of exact trimmed length.
        assert got_cpp == got_cpp_arr == got_py
        assert len(got_cpp) == len(expected)
        assert recording_cdr.trim_joint_state_cdr(got_cpp) == got_cpp

        # Round-trip through ROS 2's own Fast-CDR deserializer (which may leave uninitialized
        # alignment padding bytes on serialize_message, so compare decoded fields rather than raw padding).
        decoded = deserialize_message(got_cpp, JointState)
        assert decoded.header.stamp.sec == sec
        assert decoded.header.stamp.nanosec == nsec
        assert decoded.header.frame_id == fid
        assert list(decoded.name) == names
        assert list(decoded.position) == pos
        assert list(decoded.velocity) == vel
        assert list(decoded.effort) == eff

        snap = JointSnapshot(sec, nsec, fid, tuple(names), tuple(pos), tuple(vel), tuple(eff), 12345)
        assert serialize_snapshot(snap) == got_cpp

    with pytest.raises(ValueError):
        recording_cdr.serialize_joint_state(0, -1, "", [], [])
    with pytest.raises(ValueError):
        recording_cdr.serialize_joint_state(2**31, 0, "", [], [])


def test_arm_teleop_math_cpp_parity_with_python():
    import math
    from adapters.core import arm_teleop_math as m

    assert m.USING_NATIVE_EXT is True
    rng = np.random.default_rng(2026)
    mappings = [
        [[0, 0, 1], [-1, 0, 0], [0, 1, 0]],
        [[0, 0, -1], [-1, 0, 0], [0, 1, 0]],
        np.eye(3),
    ]

    for _ in range(30):
        q1 = rng.normal(size=4)
        q2 = rng.normal(size=4)
        axis = rng.normal(size=3)
        angle = float(rng.uniform(-math.pi, math.pi))
        M = mappings[_ % len(mappings)]

        np.testing.assert_allclose(m.normalize_quaternion(q1), m._py_normalize_quaternion(q1), atol=1e-12)
        np.testing.assert_allclose(m.quaternion_multiply(q1, q2), m._py_quaternion_multiply(q1, q2), atol=1e-12)
        np.testing.assert_allclose(m.quaternion_inverse(q1), m._py_quaternion_inverse(q1), atol=1e-12)
        np.testing.assert_allclose(m.quaternion_from_axis_angle(axis, angle), m._py_quaternion_from_axis_angle(axis, angle), atol=1e-12)
        R = m.quaternion_to_matrix(q1)
        np.testing.assert_allclose(R, m._py_quaternion_to_matrix(q1), atol=1e-12)
        np.testing.assert_allclose(m.matrix_to_quaternion(R), m._py_matrix_to_quaternion(R), atol=1e-12)
        np.testing.assert_allclose(m.orientation_error(q1, q2), m._py_orientation_error(q1, q2), atol=1e-12)
        assert abs(m.mapped_relative_yaw(q1, q2, M) - m._py_mapped_relative_yaw(q1, q2, M)) < 1e-12
        assert abs(m.mapped_relative_pitch(q1, q2, M) - m._py_mapped_relative_pitch(q1, q2, M)) < 1e-12

        p1, p2, p3 = rng.normal(size=3), rng.normal(size=3), rng.normal(size=3)
        for orient_on in (False, True):
            cp, cq = m.relative_target(p1, q1, p2, q2, p3, q2, M, 1.1, 0.5, orient_on)
            pp, pq = m._py_relative_target(p1, q1, p2, q2, p3, q2, M, 1.1, 0.5, orient_on)
            np.testing.assert_allclose(cp, pp, atol=1e-12)
            np.testing.assert_allclose(cq, pq, atol=1e-12)

        for max_lin, max_ang in ((None, None), (0.02, 0.05)):
            cp, cq = m.stabilize_pose(p1, q1, p2, q2, 0.002, 0.01, 0.35, max_lin, max_ang)
            pp, pq = m._py_stabilize_pose(p1, q1, p2, q2, 0.002, 0.01, 0.35, max_lin, max_ang)
            np.testing.assert_allclose(cp, pp, atol=1e-12)
            np.testing.assert_allclose(cq, pq, atol=1e-12)

        t_vec = rng.uniform(-2.0, 2.0, size=7)
        prev_vec = rng.uniform(-1.0, 1.0, size=7)
        lo_vec = np.full(7, -1.5)
        hi_vec = np.full(7, 1.5)
        np.testing.assert_allclose(
            m.clamp_step(t_vec, prev_vec, lo_vec, hi_vec, 0.08),
            m._py_clamp_step(t_vec, prev_vec, lo_vec, hi_vec, 0.08),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            m.joint_limit_avoidance(prev_vec, lo_vec, hi_vec, 0.15),
            m._py_joint_limit_avoidance(prev_vec, lo_vec, hi_vec, 0.15),
            atol=1e-12,
        )

    # Scalar & velocity helpers
    assert m.binary_trigger_state(0.5, 0.5) == m._py_binary_trigger_state(0.5, 0.5)
    assert m.sticks_outward(-0.9, 0.9, 0.8) == m._py_sticks_outward(-0.9, 0.9, 0.8)
    assert m.sticks_outward(float("nan"), 0.9, 0.8) is False
    assert m.joystick_base_velocity(-0.4, 0.7, 0.1, 0.3, 0.2) == pytest.approx(
        m._py_joystick_base_velocity(-0.4, 0.7, 0.1, 0.3, 0.2)
    )
    assert m.joystick_yaw_velocity(-0.54, 0.08, 0.45, -1.0) == pytest.approx(
        m._py_joystick_yaw_velocity(-0.54, 0.08, 0.45, -1.0)
    )
    assert m.adaptive_damping(0.04, 0.03, 0.3, 0.1) == pytest.approx(
        m._py_adaptive_damping(0.04, 0.03, 0.3, 0.1)
    )
    assert m.head_yaw_orientation_velocity(0.6) == pytest.approx(m._py_head_yaw_orientation_velocity(0.6))
    offsets = [(0.15, -0.10), (0.02, 0.12), (float("nan"), 0.0)]
    assert m.hands_reach_assist_velocity(offsets) == pytest.approx(m._py_hands_reach_assist_velocity(offsets))
    assert m.rotate_velocity_to_base_frame(0.2, -0.1, 0.4) == pytest.approx(
        m._py_rotate_velocity_to_base_frame(0.2, -0.1, 0.4)
    )
    assert m.project_displacement_to_heading_frame(0.2, -0.1, 0.4) == pytest.approx(
        m._py_project_displacement_to_heading_frame(0.2, -0.1, 0.4)
    )
    assert m.blend_three_layer_base_velocity((0.1, 0.05, 0.1), (0.15, -0.05, 0.0), 0.2) == pytest.approx(
        m._py_blend_three_layer_base_velocity((0.1, 0.05, 0.1), (0.15, -0.05, 0.0), 0.2)
    )
    head_q = m.quaternion_from_axis_angle([0, 1, 0], 0.35)
    root_q = m.quaternion_from_axis_angle([0, 0, 1], 0.2)
    task_q = m.quaternion_from_axis_angle([0, 0, 1], -0.1)
    np.testing.assert_allclose(
        m.clutch_base_axis_mapping(mappings[0], head_q, root_q, task_q),
        m._py_clutch_base_axis_mapping(mappings[0], head_q, root_q, task_q),
        atol=1e-12,
    )

    # Validation errors parity
    with pytest.raises(ValueError):
        m.normalize_quaternion([0, 0, 0, 0])
    with pytest.raises(ValueError):
        m.normalize_quaternion([[1, 0], [0, 1]])
    with pytest.raises(ValueError):
        m.matrix_to_quaternion([1, 2, 3])
    with pytest.raises(ValueError):
        m.stabilize_pose([0, 0, 0], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0, 1], -0.1, 0.0, 0.5)
    with pytest.raises(ValueError):
        m.joint_limit_avoidance([0.0], [-1.0], [1.0], 0.5)


def test_collision_guard_cpp_parity_with_pybullet(monkeypatch):
    from types import SimpleNamespace
    import yaml
    from adapters.core import collision_safety

    assert collision_safety.USING_NATIVE_EXT is True
    urdf = PROJECT / "adapters" / "robots" / "x1" / "urdf" / "hc_tj_robot.urdf"
    teleop = yaml.safe_load((PROJECT / "adapters" / "robots" / "x1" / "arm_teleop.yaml").read_text())
    api = SimpleNamespace(
        urdf_path=urdf,
        teleop=teleop,
        groups={
            "right_arm": teleop["arms"]["right"]["joint_names"],
            "left_arm": teleop["arms"]["left"]["joint_names"],
            "waist": teleop["body"]["waist_joint_names"],
        },
    )

    monkeypatch.setenv("HC_NATIVE_EXT", "1")
    g_cpp = collision_safety.ModelCollisionGuard(api)
    assert g_cpp._cpp is not None

    monkeypatch.setenv("HC_NATIVE_EXT", "0")
    g_py = collision_safety.ModelCollisionGuard(api)
    assert g_py._cpp is None

    try:
        # 1. Exact protected-pair set parity (507 pairs on X1)
        clr_cpp = {(g_cpp.names[a], g_cpp.names[b]): c for (a, b), c in g_cpp.clearances.items()}
        clr_py = {(g_py.names[a], g_py.names[b]): c for (a, b), c in g_py.clearances.items()}
        assert set(clr_cpp.keys()) == set(clr_py.keys())
        assert len(clr_cpp) == 507
        for pair_key in clr_cpp:
            assert abs(clr_cpp[pair_key] - clr_py[pair_key]) < 2e-3

        # 2. Home pose is safe in both
        home = dict(g_cpp.home)
        allowed_c, cand_c, esc_c = g_cpp.check(home, {})
        allowed_p, cand_p, esc_p = g_py.check(home, {})
        assert (allowed_c, cand_c, esc_c) == (True, None, False)
        assert (allowed_p, cand_p, esc_p) == (True, None, False)

        # 3. Colliding/penetrating pose & monotonic escape parity
        rng = np.random.default_rng(7)
        arm_joints = teleop["arms"]["right"]["joint_names"] + teleop["arms"]["left"]["joint_names"]
        colliding = dict(home)
        for j in arm_joints:
            colliding[j] = float(colliding[j] + rng.uniform(-1.5, 1.5))

        # Blocking entry into collision from home
        al_c, hit_c, es_c = g_cpp.check(home, colliding)
        al_p, hit_p, es_p = g_py.check(home, colliding)
        assert al_c is False and al_p is False
        assert es_c is False and es_p is False
        assert hit_c is not None and hit_p is not None
        assert (hit_c[0], hit_c[1]) == (hit_p[0], hit_p[1])
        assert abs(hit_c[2] - hit_p[2]) < 1e-3

        # Moving halfway back toward home monotonically decreases deficit -> allowed in escape mode
        toward_home = {j: 0.9 * colliding[j] + 0.1 * home[j] for j in arm_joints}
        al_esc_c, hit_esc_c, in_col_c = g_cpp.check(colliding, toward_home)
        al_esc_p, hit_esc_p, in_col_p = g_py.check(colliding, toward_home)
        assert in_col_c is True and in_col_p is True
        assert al_esc_c == al_esc_p
        if hit_esc_c is not None and hit_esc_p is not None:
            assert abs(hit_esc_c[2] - hit_esc_p[2]) < 2e-3

        # Incomplete measured feedback raises ValueError
        with pytest.raises(ValueError):
            g_cpp.check({}, {})
    finally:
        g_cpp.close()
        g_py.close()


def test_run_generic_controller_check_detects_cpp_and_fallback():
    res_cpp = subprocess.run(
        [str(PROJECT / "run_generic_controller.sh"), "--check"],
        cwd=PROJECT,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert res_cpp.returncode == 0, res_cpp.stderr
    assert "reconstructed_v23=cpp" in res_cpp.stdout

    env = dict(os.environ, HC_V23_CPP="0", HC_CONTROLLER_PREFIX="/nonexistent")
    res_py = subprocess.run(
        [str(PROJECT / "run_generic_controller.sh"), "--check"],
        cwd=PROJECT,
        env=env,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert res_py.returncode == 0, res_py.stderr
    assert "reconstructed_v23=ready" in res_py.stdout


def test_v23_cpp_ros2_node_live_e2e_openarmx_and_x1_intervention():
    """Spin the real compiled C++17 rclcpp binary on an isolated localhost ROS domain."""
    assert V23_CPP_BIN.is_file()
    import rclpy
    from geometry_msgs.msg import Pose, PoseArray
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String

    domain = int(os.environ.get("ROS_DOMAIN_ID", "219"))
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    # --- Test A: X1 with legacy_intervention=true (permit -> feedback -> grant -> owned_poses + aux -> /internal/control/joint_targets) ---
    proc = subprocess.Popen(
        [
            str(V23_CPP_BIN),
            str(PROJECT / "adapters/robots/x1/controller_v23.yml"),
            "--ros-args",
            "--disable-external-lib-logs",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    rclpy.init()
    probe = Node(
        "cpp_v23_e2e_probe",
        cli_args=["--ros-args", "--disable-external-lib-logs"],
    )
    try:
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        acks = []
        owned_outs = []
        probe.create_subscription(String, "/status/control/ack", lambda m: acks.append(json.loads(m.data)), qos_rel)
        probe.create_subscription(JointState, "/internal/control/joint_targets", lambda m: owned_outs.append(m), qos_rel)

        joint_pub = probe.create_publisher(JointState, "/observation/joint_states", qos_rel)
        permit_pub = probe.create_publisher(Header, "/control/motion/permit", qos_rel)
        owned_pose_pub = probe.create_publisher(PoseArray, "/internal/vr/owned_poses", qos_rel)
        owned_aux_pub = probe.create_publisher(JointState, "/internal/vr/aux_joint_targets", qos_rel)

        def spin_until(pred, timeout=5.0):
            deadline = time.monotonic() + timeout
            while not pred() and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.01)
            assert pred(), f"timeout; proc_poll={proc.poll()} acks={acks[-2:] if acks else []}"

        # Wait for solver heartbeat ack
        spin_until(lambda: any(a.get("participant") == "solver" for a in acks))
        solver_instance = next(a["instance"] for a in acks if a.get("participant") == "solver")

        token_key = "hc1:ctrl1:sess1:1:vr:manual1"
        x1_names = [f"Joint{i}_R" for i in range(1, 8)] + [f"Joint{i}_L" for i in range(1, 8)]
        x1_pos = [-1.396, -0.52, 1.74, -1.91, -0.26, 0.17, 0.0,
                   1.396, -0.52, -1.74, -1.91, 0.26, 0.17, 0.0]

        def publish_fb():
            msg = JointState(name=x1_names, position=x1_pos)
            msg.header.stamp = probe.get_clock().now().to_msg()
            joint_pub.publish(msg)

        # 1. Send revoke permit, then fresh feedback -> solver becomes ready and acks token
        revoke = Header()
        revoke.stamp = probe.get_clock().now().to_msg()
        revoke.frame_id = f"{token_key}#revoke#{solver_instance}#"
        permit_pub.publish(revoke)
        deadline = time.monotonic() + 4.0
        while not any(a.get("token") == token_key and a.get("phase") == "revoke" for a in acks):
            assert time.monotonic() < deadline, f"revoke ack not received: {acks[-3:]}"
            publish_fb()
            rclpy.spin_once(probe, timeout_sec=0.01)

        # 2. Send grant permit + fresh feedback -> solver becomes ready in grant phase
        grant = Header()
        grant.stamp = probe.get_clock().now().to_msg()
        grant.frame_id = f"{token_key}#grant#{solver_instance}#"
        permit_pub.publish(grant)
        deadline = time.monotonic() + 4.0
        while not any(a.get("token") == token_key and a.get("phase") == "grant" for a in acks):
            assert time.monotonic() < deadline, f"grant ack not received: {acks[-3:]}"
            publish_fb()
            rclpy.spin_once(probe, timeout_sec=0.01)

        # 3. Publish owned_poses + aux gripper targets -> verify /internal/control/joint_targets
        pa = PoseArray()
        pa.header.frame_id = f"{token_key}#generic_task_bases"
        for _ in range(2):
            p = Pose()
            p.position.x, p.position.y, p.position.z = 0.25, -0.15, 0.1
            p.orientation.w = 1.0
            pa.poses.append(p)

        aux = JointState(name=["R_ban", "L_ban"], position=[0.5, 0.25])
        aux.header.frame_id = f"{token_key}#"

        deadline = time.monotonic() + 4.0
        while not owned_outs:
            assert time.monotonic() < deadline, "no /internal/control/joint_targets output"
            now_msg = probe.get_clock().now().to_msg()
            publish_fb()
            grant.stamp = now_msg
            permit_pub.publish(grant)
            aux.header.stamp = now_msg
            owned_aux_pub.publish(aux)
            pa.header.stamp = now_msg
            owned_pose_pub.publish(pa)
            rclpy.spin_once(probe, timeout_sec=0.01)

        out = owned_outs[-1]
        assert out.header.frame_id == f"{token_key}#"
        assert list(out.name) == x1_names + ["R_ban", "L_ban"]
        assert len(out.position) == 16
        assert all(np.isfinite(out.position))
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)

    # --- Test B: OpenArmX standard (non-intervention) mode + reset handshake ---
    proc2 = subprocess.Popen(
        [
            str(V23_CPP_BIN),
            str(PROJECT / "adapters/robots/openarmx/controller_v23.yml"),
            "--ros-args",
            "--disable-external-lib-logs",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    rclpy.init()
    probe2 = Node(
        "cpp_v23_openarmx_probe",
        cli_args=["--ros-args", "--disable-external-lib-logs"],
    )
    try:
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        cmds = []
        reset_acks = []
        probe2.create_subscription(JointState, "/control/solver/joint_targets", lambda m: cmds.append(m), qos_rel)
        probe2.create_subscription(Header, "/status/solver/reset_ack", lambda m: reset_acks.append(m), qos_rel)
        joint_pub = probe2.create_publisher(JointState, "/observation/joint_states", qos_rel)
        ee_pub = probe2.create_publisher(PoseArray, "/control/solver/ee_targets", qos_rel)
        reset_pub = probe2.create_publisher(Bool, "/control/solver/reset", qos_rel)

        oa_names = [f"openarmx_right_joint{i}" for i in range(1, 8)] + [
            f"openarmx_left_joint{i}" for i in range(1, 8)
        ]
        oa_pos = [0.0] * 14
        pa = PoseArray()
        for _ in range(2):
            p = Pose()
            p.position.x, p.position.y, p.position.z = 0.25, 0.0, 0.2
            p.orientation.w = 1.0
            pa.poses.append(p)

        deadline = time.monotonic() + 5.0
        while not cmds:
            assert time.monotonic() < deadline, "no OpenArmX joint_targets from C++ node"
            now_msg = probe2.get_clock().now().to_msg()
            js = JointState(name=oa_names, position=oa_pos)
            js.header.stamp = now_msg
            joint_pub.publish(js)
            pa.header.stamp = now_msg
            ee_pub.publish(pa)
            rclpy.spin_once(probe2, timeout_sec=0.01)

        assert list(cmds[-1].name) == oa_names
        assert len(cmds[-1].position) == 14

        # Verify reset -> reset_ack handshake
        reset_pub.publish(Bool(data=True))
        deadline = time.monotonic() + 4.0
        while not reset_acks:
            assert time.monotonic() < deadline, "no reset_ack from C++ node"
            js = JointState(name=oa_names, position=oa_pos)
            js.header.stamp = probe2.get_clock().now().to_msg()
            joint_pub.publish(js)
            rclpy.spin_once(probe2, timeout_sec=0.01)
    finally:
        probe2.destroy_node()
        rclpy.shutdown()
        proc2.terminate()
        try:
            proc2.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc2.kill()
            proc2.wait(timeout=3)
