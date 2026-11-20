"""Tests for smooth start-of-replay alignment, non-blocking replay IPC, and MCAP action capture."""
from __future__ import annotations

from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")
from mcap.reader import make_reader
from mcap.writer import Writer as McapWriter
from mcap.well_known import MessageEncoding, SchemaEncoding
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import yaml

from middleware.core.control_process import CoreProcessClient, IsolatedRosBridge, RemotePublicationCapture
from middleware.core.publication_capture import PublicationCapture
from middleware.core.replay_alignment import ReplayAlignmentPlanner, step_trapezoidal_axis
from middleware.core.topic_player import TopicPlayer
from middleware.core.topic_recorder import TopicRecorder, _get_msg_def
from tests.test_legacy_control import Rig, message


ROOT = Path(__file__).resolve().parents[1]


def _wait_until(predicate, timeout: float = 8.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), "condition did not become true before timeout"


def _joint(values: dict[str, float], stamp_ns: int) -> JointState:
    msg = JointState(name=list(values), position=[float(v) for v in values.values()], velocity=[0.0] * len(values))
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    return msg


class _DummySink:
    def __init__(self) -> None:
        self.records: list[dict] = []

    def record(self, payload: dict) -> bool:
        self.records.append(payload)
        return True


def test_trapezoidal_planner_respects_bounds_and_converges():
    planner = ReplayAlignmentPlanner(
        epoch=1,
        arm_topic="/action/joint_states",
        arm_joint_names=("Joint1_R", "Joint2_R"),
        goal_positions={"Joint1_R": 0.75, "Joint2_R": -0.40},
        align_joints=("Joint1_R", "Joint2_R"),
        start_positions={"Joint1_R": 0.0, "Joint2_R": 0.0},
        max_velocity=0.5,
        max_acceleration=1.0,
        start_tolerance=0.03,
        settle_tolerance=0.05,
        max_tracking_error=0.30,
        now_mono=0.0,
    )
    assert planner.state == "aligning"
    measured = {"Joint1_R": 0.0, "Joint2_R": 0.0}
    prev_pos = dict(measured)
    prev_vel = {"Joint1_R": 0.0, "Joint2_R": 0.0}
    t = 0.0
    max_step_vel = 0.0
    for _ in range(500):
        res = planner.step(t, measured, feedback_fresh=True, replay_allowed=True)
        if res.get("publish"):
            pos = dict(zip(planner.arm_joint_names, res["arm_positions"]))
            for j in ("Joint1_R", "Joint2_R"):
                vel = (pos[j] - prev_pos[j]) / 0.01 if t > 0 else 0.0
                max_step_vel = max(max_step_vel, abs(vel))
                assert abs(vel - prev_vel[j]) <= 1.0 * 0.01 + 1e-3 or res.get("done")
                prev_vel[j] = vel
                prev_pos[j] = pos[j]
                measured[j] = 0.7 * measured[j] + 0.3 * pos[j]
            if res.get("done"):
                break
        t += 0.01
    assert planner.state == "aligned"
    assert max_step_vel <= 0.505
    assert abs(measured["Joint1_R"] - 0.75) <= 0.05
    assert abs(measured["Joint2_R"] + 0.40) <= 0.05


def test_trapezoidal_planner_pause_resume_and_safety_aborts():
    planner = ReplayAlignmentPlanner(
        epoch=1,
        arm_topic="/action/joint_states",
        arm_joint_names=("Joint1_R",),
        goal_positions={"Joint1_R": 0.60},
        align_joints=("Joint1_R",),
        start_positions={"Joint1_R": 0.0},
        max_tracking_error=0.30,
        now_mono=0.0,
    )
    res0 = planner.step(0.01, {"Joint1_R": 0.0}, feedback_fresh=True, replay_allowed=True)
    assert res0["publish"]
    held = res0["arm_positions"][0]
    planner.pause()
    res_paused = planner.step(0.02, {"Joint1_R": held}, feedback_fresh=True, replay_allowed=True)
    assert res_paused["publish"] and res_paused["arm_positions"][0] == pytest.approx(held)
    planner.resume(0.05)
    res_resumed = planner.step(0.06, {"Joint1_R": held}, feedback_fresh=True, replay_allowed=True)
    assert res_resumed["publish"] and res_resumed["arm_positions"][0] > held

    # Obstructed arm exceeding 0.30 rad tracking error aborts safely.
    aborted = planner.step(0.07, {"Joint1_R": held - 0.31}, feedback_fresh=True, replay_allowed=True)
    assert aborted["state"] == "error"
    assert "too far" in aborted["error"]

    # Stale feedback aborts immediately.
    p2 = ReplayAlignmentPlanner(
        epoch=1,
        arm_topic="/action/joint_states",
        arm_joint_names=("Joint1_R",),
        goal_positions={"Joint1_R": 0.5},
        align_joints=("Joint1_R",),
        start_positions={"Joint1_R": 0.0},
        now_mono=0.0,
    )
    stale = p2.step(0.01, {"Joint1_R": 0.0}, feedback_fresh=False, replay_allowed=True)
    assert stale["state"] == "error"
    assert "stale" in stale["error"]


def test_ros_bridge_replay_alignment_and_capture_in_process():
    rig = Rig()
    rig.control.disable()
    rig.feed()
    rig.bridge.set_command_output_enabled(True)
    bridge, boundary = rig.bridge, rig.boundary
    sink = _DummySink()
    capture = PublicationCapture(sink)
    capture.begin_generation()
    bridge._publication_capture = capture
    bridge._actor_thread_ident = threading.get_ident()
    try:
        assert bridge.begin_replay()
        goal = {n: float(boundary.positions[n]) for n in boundary.arm_fields if n in boundary.positions}
        goal["Joint1_R"] = goal["Joint1_R"] + 0.45
        goal["R_ban"] = 0.0
        goal["L_ban"] = 0.0
        target_msg = _joint(goal, time.time_ns())
        st = bridge.start_replay_alignment(serialize_message(target_msg))
        assert st["state"] == "aligning"
        assert st["max_diff_rad"] == pytest.approx(0.45, abs=1e-3)

        rig.wire.clear()
        # Simulate hardware tracking commanded alignment pose smoothly at 100 Hz.
        for _ in range(300):
            bridge.receive_hardware_ready(True)
            bridge._tick_replay_alignment(boundary.node, {}, lambda _t: JointState)
            for topic, sent in rig.wire[-1:]:
                if topic == "/action/joint_states":
                    pos_map = dict(zip(sent.name, sent.position))
                    fb = {n: float(pos_map.get(n, boundary.positions[n])) for n in boundary.feedback_order}
                    boundary.state(_joint(fb, time.time_ns()))
            if bridge.replay_alignment_status()["state"] in {"aligned", "error"}:
                break
            time.sleep(0.011)

        assert bridge.replay_alignment_status()["state"] == "aligned", bridge.replay_alignment_status()
        assert abs(boundary.positions["Joint1_R"] - goal["Joint1_R"]) <= 0.05
        # Alignment publications must also be captured onto PublicationCapture.
        capture_deadline = time.monotonic() + 2.0
        while time.monotonic() < capture_deadline and len(sink.records) < len(rig.wire):
            time.sleep(0.01)
        assert len(sink.records) >= len(rig.wire) >= 5
        assert all(rec["topic"] == "/action/joint_states" for rec in sink.records)
        assert all(rec["stamp_ns"] > 0 for rec in sink.records)

        # Replay home flow still works after end_replay().
        bridge._replay_home_publisher = MagicMock()
        bridge._replay_home_publisher.get_subscription_count.return_value = 1
        bridge.end_replay(require_reset=True)
        assert bridge.request_replay_home(output_resumed=False)
    finally:
        capture.close()


def test_start_replay_alignment_skips_wait_when_already_at_start_pose():
    rig = Rig()
    rig.control.disable()
    rig.feed()
    rig.bridge.set_command_output_enabled(True)
    bridge, boundary = rig.bridge, rig.boundary
    assert bridge.begin_replay()
    goal = {n: float(boundary.positions[n]) for n in boundary.arm_fields if n in boundary.positions}
    goal["Joint1_R"] += 0.01  # Within 0.03 rad start tolerance.
    goal["R_ban"] = 0.0
    goal["L_ban"] = 0.0
    st = bridge.start_replay_alignment(serialize_message(_joint(goal, time.time_ns())))
    assert st["state"] == "aligned"
    assert st["max_diff_rad"] <= 0.03


def test_emergency_stop_aborts_active_alignment_immediately():
    rig = Rig()
    rig.control.disable()
    rig.feed()
    rig.bridge.set_command_output_enabled(True)
    bridge, boundary = rig.bridge, rig.boundary
    assert bridge.begin_replay()
    goal = {n: float(boundary.positions[n]) for n in boundary.arms}
    goal["Joint1_R"] += 0.50
    st = bridge.start_replay_alignment(serialize_message(_joint(goal, time.time_ns())))
    assert st["state"] == "aligning"
    bridge.emergency_stop("/control/emergency_stop", "estop mid-align")
    st_after = bridge.replay_alignment_status()
    assert st_after["state"] == "error"
    rig.wire.clear()
    bridge._tick_replay_alignment(boundary.node, {}, lambda _t: JointState)
    assert not [m for t, m in rig.wire if t == "/action/joint_states"]


def test_start_replay_alignment_rejects_limit_and_collision_violations():
    rig = Rig()
    rig.control.disable()
    rig.feed()
    rig.bridge.set_command_output_enabled(True)
    bridge, boundary = rig.bridge, rig.boundary
    assert bridge.begin_replay()
    bad_limit = {n: float(boundary.positions[n]) for n in boundary.arms}
    bad_limit["Joint1_R"] = 9.0
    with pytest.raises(ValueError, match="rejected"):
        bridge.start_replay_alignment(serialize_message(_joint(bad_limit, time.time_ns())))

    boundary.collision.check.return_value = (False, ("arm", "body"), False)
    ok_vals = {n: float(boundary.positions[n]) for n in boundary.arms}
    ok_vals["Joint1_R"] += 0.2
    with pytest.raises(ValueError, match="rejected"):
        bridge.start_replay_alignment(serialize_message(_joint(ok_vals, time.time_ns())))


def test_isolated_replay_captures_action_and_aligns_smoothly_e2e(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "232")
    profile = ROOT / "adapters/robots/x1"
    teleop = yaml.safe_load((profile / "arm_teleop.yaml").read_text())
    arm_names = [n for side in ("right", "left") for n in teleop["arms"][side]["joint_names"]]
    body_names = list(teleop["body"]["waist_joint_names"])
    l_hand = list(teleop["grippers"]["left"]["finger_joint_names"])
    r_hand = list(teleop["grippers"]["right"]["finger_joint_names"])
    l_open = [float(x) for x in teleop["grippers"]["left"]["finger_open"]]
    r_open = [float(x) for x in teleop["grippers"]["right"]["finger_open"]]

    home_vals = {n: float(teleop["robot"]["initial_joints"][n]) for n in arm_names + body_names}
    cur_vals = dict(home_vals)
    replay_start_r1 = cur_vals["Joint1_R"] + 0.55

    # Build a 0.6s X1 recording starting 0.55 rad away from initial home pose.
    replay_src = tmp_path / "source_x1.mcap"
    with replay_src.open("wb") as stream:
        w = McapWriter(stream)
        w.start(profile="ros2")
        w.add_metadata("hc_teleop.session", {"profile_id": "x1"})
        w.add_metadata("hc_teleop.record_only", {"topic": "/action/joint_states"})
        sid = w.register_schema("sensor_msgs/msg/JointState", SchemaEncoding.ROS2, _get_msg_def("sensor_msgs/msg/JointState"))
        ch_pub = w.register_channel("/internal/recording/published_action", MessageEncoding.CDR, sid)
        ch_lh = w.register_channel("/action/left_hand/joint_states", MessageEncoding.CDR, sid)
        ch_rh = w.register_channel("/action/right_hand/joint_states", MessageEncoding.CDR, sid)
        t0 = 1_795_000_000_000_000_000
        events = []
        for i in range(60):
            ts = t0 + i * 10_000_000
            cmd = dict(home_vals)
            cmd["Joint1_R"] = replay_start_r1 + 0.05 * (i / 59.0)
            events.append((ts, ch_pub, bytes(serialize_message(_joint(cmd, ts)))))
            if i % 2 == 0:
                events.append((ts + 100_000, ch_lh, bytes(serialize_message(_joint(dict(zip(l_hand, l_open)), ts)))))
                events.append((ts + 200_000, ch_rh, bytes(serialize_message(_joint(dict(zip(r_hand, r_open)), ts)))))
        events.sort(key=lambda x: x[0])
        for ts, ch, raw in events:
            w.add_message(ch, ts, raw, ts)
        w.finish()

    ctx = Context()
    rclpy.init(args=[], context=ctx, domain_id=232)
    node = rclpy.create_node("test_replay_align_capture_robot", context=ctx)
    exe = SingleThreadedExecutor(context=ctx)
    exe.add_node(node)
    fb_pub = node.create_publisher(JointState, "/observation/joint_states", qos_profile_sensor_data)
    hfb_pub = node.create_publisher(JointState, "/observation/hands/joint_states", qos_profile_sensor_data)
    hr_pub = node.create_publisher(Bool, "/status/hardware/ready", 10)
    received_arms: list[tuple[float, list[str], list[float], bytes]] = []
    received_lh: list[bytes] = []
    received_rh: list[bytes] = []
    homes: list[bool] = []
    lock = threading.Lock()

    def on_arm(raw):
        msg = deserialize_message(bytes(raw), JointState)
        with lock:
            pos_map = dict(zip(msg.name, msg.position))
            for n in arm_names:
                if n in pos_map:
                    # First-order simulated actuator tracking toward command.
                    cur_vals[n] = 0.55 * cur_vals[n] + 0.45 * float(pos_map[n])
            received_arms.append((time.monotonic(), list(msg.name), list(msg.position), bytes(raw)))

    node.create_subscription(JointState, "/action/joint_states", on_arm, 100, raw=True)
    node.create_subscription(JointState, "/action/left_hand/joint_states", lambda r: received_lh.append(bytes(r)), 100, raw=True)
    node.create_subscription(JointState, "/action/right_hand/joint_states", lambda r: received_rh.append(bytes(r)), 100, raw=True)
    node.create_subscription(Bool, "/teleop/arm/home", lambda m: homes.append(bool(m.data)), 10)

    stop_sim = threading.Event()
    def sim_loop():
        nxt = 0.0
        while not stop_sim.is_set():
            now = time.monotonic()
            if now >= nxt:
                nxt = now + 0.01
                stamp = time.time_ns()
                hr_pub.publish(Bool(data=True))
                with lock:
                    snap_vals = dict(cur_vals)
                fb_pub.publish(_joint(snap_vals, stamp))
                hand_vals = dict(zip(l_hand + r_hand, l_open + r_open))
                hfb_pub.publish(_joint(hand_vals, stamp))
            exe.spin_once(timeout_sec=0.002)
    sim_thread = threading.Thread(target=sim_loop, daemon=True)
    sim_thread.start()

    cfg = {
        "domain_id": 232,
        "node_name": "test_replay_align_owner",
        "legacy_control_profile": str(profile),
        "intervention": {"automatic_recording": False, "max_arm_velocity_rad_s": 1.0, "max_target_error_rad": 0.3},
        "dataset_topics": {"enabled": False},
        "subscriptions": [],
    }
    client = CoreProcessClient(cfg)
    rec_dir = tmp_path / "rec"
    recorder = TopicRecorder(
        {"directory": "rec", "record_action_at_publication": True},
        tmp_path,
        topics=[
            {"topic": "/action/joint_states", "type": "sensor_msgs/msg/JointState"},
            {"topic": "/observation/joint_states", "type": "sensor_msgs/msg/JointState"},
            {"topic": "/status/teleop", "type": "std_msgs/msg/String"},
            {"topic": "/status/control/ownership", "type": "std_msgs/msg/String"},
            {"topic": "/action/left_hand/joint_states", "type": "sensor_msgs/msg/JointState"},
            {"topic": "/action/right_hand/joint_states", "type": "sensor_msgs/msg/JointState"},
        ],
    )
    recorder.action_label_policy = {"feedback_timeout_s": 0.25, "status_timeout_s": 0.25}
    client.capture_spec = {"resources": recorder.ipc_resources(), "finish": recorder._finish_event}
    capture = RemotePublicationCapture(client, lambda: True)
    recorder.publication_capture = capture

    class _NoopTelemetry:
        def publish_raw(self, *_a, **_kw):
            return True

    bridge = IsolatedRosBridge.__new__(IsolatedRosBridge)
    bridge._client = client
    bridge._telemetry = _NoopTelemetry()

    try:
        client.start()
        _wait_until(
            lambda: client.snapshot().get("control_ready")
            and client.snapshot().get("command_mux", {}).get("hardware_ready")
            and node.count_publishers("/teleop/arm/home") == 1
        )
        client.call("set_command_output_enabled", True)

        # Verify stale-fence non-blocking replay packet is harmlessly dropped.
        stale_fence = (client.boot, -1, -1)
        assert client.submit_replay_raw(
            "/action/joint_states",
            "sensor_msgs/msg/JointState",
            b"stale",
            expected_fence=stale_fence,
        )
        _wait_until(lambda: client.snapshot().get("control_process", {}).get("stale_queued_commands", 0) >= 1)

        recorder.start("captured_replay.mcap", {"profile_id": "x1"})
        player = TopicPlayer(tmp_path, lambda: bridge, active_profile_provider=lambda: "x1")
        seen_aligning = []
        player._state_callback = lambda ev, st: seen_aligning.append((ev, st.get("aligning")))

        t_play_start = time.monotonic()
        player.play("source_x1.mcap", speed=1.0, mode="drive")
        _wait_until(lambda: player.status()["state"] == "completed", timeout=10.0)
        total_elapsed = time.monotonic() - t_play_start

        # Alignment took finite smooth time (< 5s total for 0.55 rad align + 0.6s replay).
        assert total_elapsed < 6.0
        assert any(ev == "replay_aligning" and flag is True for ev, flag in seen_aligning)
        assert any(ev == "replay_aligned" and flag is False for ev, flag in seen_aligning)

        # Verify smooth Joint1_R step trajectory (no single-step lunge from 0 -> 0.55!).
        with lock:
            r1_seq = [
                pos[names.index("Joint1_R")]
                for _t, names, pos, _raw in received_arms
                if "Joint1_R" in names
            ]
            arm_raw_wire = {raw for _t, _n, _p, raw in received_arms}
        assert len(r1_seq) >= 65
        max_step = max(abs(b - a) for a, b in zip(r1_seq, r1_seq[1:]))
        assert max_step < 0.03, f"expected smooth step < 0.03 rad, saw {max_step}"
        assert abs(r1_seq[-1] - (replay_start_r1 + 0.05)) < 0.01

        _wait_until(lambda: capture.status()["enqueued"] >= len(r1_seq) - 5, timeout=3.0)
        rec_res = recorder.stop()
        assert not rec_res["error"], rec_res

        with recorder.path.open("rb") as stream:
            rec_msgs = [(c.topic, m.data) for _s, c, m in make_reader(stream).iter_messages()]
        pub_action_msgs = [raw for topic, raw in rec_msgs if topic == "/internal/recording/published_action"]
        action_msgs = [raw for topic, raw in rec_msgs if topic == "/action/joint_states"]
        assert len(pub_action_msgs) >= 60
        assert len(action_msgs) == len(pub_action_msgs)
        assert all(raw in arm_raw_wire for raw in pub_action_msgs)
        assert all(raw in arm_raw_wire for raw in action_msgs)

        # Return-to-home flow after alignment + replay still works (and requires reset first).
        player.return_home(lambda resumed: bridge.request_replay_home(
            output_resumed=resumed, expected_fence=client.authorization_fence()
        ))
        assert player.status()["requires_reset"] is False
        _wait_until(lambda: len(homes) == 1 and homes[0] is True)
    finally:
        stop_sim.set()
        sim_thread.join(timeout=2.0)
        recorder.stop()
        capture.close()
        client.stop()
        recorder.shutdown()
        exe.shutdown()
        node.destroy_node()
        ctx.try_shutdown()


def test_submit_replay_raw_queue_full_does_not_fail_control_process():
    cfg = {
        "domain_id": 232,
        "node_name": "test_full_queue_nonfatal",
        "legacy_control_profile": str(ROOT / "adapters/robots/x1"),
        "intervention": {"automatic_recording": False},
        "dataset_topics": {"enabled": False},
        "subscriptions": [],
    }
    client = CoreProcessClient(cfg)
    client.process = SimpleNamespace(is_alive=lambda: True)
    client.authorization_fence = lambda: (client.boot, 0, 0)
    full_q = queue.Queue(maxsize=1)
    full_q.put_nowait({"sentinel": True})
    client.channels["replay"] = full_q
    assert client.submit_replay_raw("/action/joint_states", "sensor_msgs/msg/JointState", b"x") is False
    assert not client._failure
