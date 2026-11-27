"""Regression tests for peer heartbeat timeout decoupling, CPU pinning, and idle AUTO FK skip."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import time
import types
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from middleware.core.config import ConfigError, ConfigStore, validate_config
from middleware.core.control.control_ownership import ControlOwnership, ControlToken
from middleware.core.control.ros_bridge import RosBridge
from adapters.core.intervention import InterventionTeleop

ROOT = Path(__file__).resolve().parents[1]


def test_peer_heartbeat_timeout_decoupled_from_manual_input_timeout():
    now = [10_000_000_000]
    control = ControlOwnership(
        clock=lambda: now[0],
        ros_clock=lambda: now[0],
        recording_enabled=False,
        required_peers=("solver", "teleop"),
        input_timeout_sec=0.3,
        peer_heartbeat_timeout_sec=1.0,
    )
    assert control.input_timeout_ns == 300_000_000
    assert control.peer_heartbeat_timeout_ns == 1_000_000_000

    for peer in control.required_peers:
        control.peer(peer, peer)
    control.enable()
    control.input(
        device_instance="vr",
        sequence=1,
        valid=True,
        gripped=False,
        stamp_ns=now[0],
    )
    control.start_auto(recording_ready=False, control_ready=True)
    for phase in ("revoke", "grant"):
        for peer in control.required_peers:
            control.acknowledge(control.token, phase, peer, peer, True)
        control.advance(control_ready=True, manual_ready=False)
    assert control.state == "AUTO"

    # 400 ms jitter exceeds 300 ms manual input timeout, but stays within 1.0 s peer heartbeat.
    now[0] += 400_000_000
    control.watchdog()
    assert control.state == "AUTO"
    assert not control.input_valid

    # Exceeding 1.0 s still faults with limit_ms == 1000.0.
    now[0] += 601_000_000
    control.watchdog()
    assert control.state == "FAULT"
    assert "heartbeat lost" in control.reason
    event = next(e for e in control.events if e["kind"] == "heartbeat_timeout")
    assert event["limit_ms"] == 1000.0
    assert event["gap_ms"] > 1000.0


def test_manual_input_timeout_remains_300ms_even_when_peer_heartbeat_is_1s():
    now = [10_000_000_000]
    control = ControlOwnership(
        clock=lambda: now[0],
        ros_clock=lambda: now[0],
        recording_enabled=False,
        required_peers=("solver", "teleop"),
        input_timeout_sec=0.3,
        peer_heartbeat_timeout_sec=1.0,
    )
    for peer in control.required_peers:
        control.peer(peer, peer)
    control.enable()
    control.set_dagger(True)
    control.input(
        device_instance="vr",
        sequence=1,
        valid=True,
        gripped=False,
        stamp_ns=now[0],
    )
    control.start_auto(recording_ready=False, control_ready=True)
    for phase in ("revoke", "grant"):
        for peer in control.required_peers:
            control.acknowledge(control.token, phase, peer, peer, True)
        control.advance(control_ready=True, manual_ready=False)
    control.input(
        device_instance="vr",
        sequence=2,
        valid=True,
        gripped=True,
        stamp_ns=now[0],
    )
    for phase in ("revoke", "grant"):
        for peer in control.required_peers:
            control.acknowledge(control.token, phase, peer, peer, True)
        control.advance(control_ready=True, manual_ready=True)
    assert control.state == "MANUAL"

    now[0] += 301_000_000
    control.watchdog()
    assert control.state == "FAULT"
    assert control.reason == "teleop input timeout"


def test_config_loads_and_validates_peer_heartbeat_timeout():
    settings = ConfigStore(ROOT / "middleware/config.yaml").load()["ros"]["intervention"]
    assert settings["peer_heartbeat_timeout_sec"] == 1.0

    for bad in (0.1, 0.29, 5.1, -1.0, float("nan"), True, "1.0"):
        with pytest.raises(ConfigError, match="peer_heartbeat_timeout_sec"):
            validate_config({"ros": {"intervention": {"peer_heartbeat_timeout_sec": bad}}})

    ok = validate_config({"ros": {"intervention": {"peer_heartbeat_timeout_sec": 0.8}}})
    assert ok["ros"]["intervention"]["peer_heartbeat_timeout_sec"] == 0.8

    bridge = RosBridge(
        {
            "legacy_control_profile": str(ROOT / "adapters/robots/x1"),
            "intervention": {"peer_heartbeat_timeout_sec": 1.0},
        },
        lambda *a: None,
    )
    assert bridge.ownership.peer_heartbeat_timeout_ns == 1_000_000_000
    assert bridge.ownership.input_timeout_ns == 300_000_000


def test_ownership_wire_accepts_delayed_ack_within_peer_heartbeat_window():
    pytest.importorskip("std_msgs.msg")
    from std_msgs.msg import Header

    bridge = RosBridge(
        {
            "legacy_control_profile": str(ROOT / "adapters/robots/x1"),
            "intervention": {"peer_heartbeat_timeout_sec": 1.0},
        },
        lambda *a: None,
    )
    node = Mock()
    callbacks = {}
    node.create_subscription.side_effect = (
        lambda cls, topic, cb, *a, **kw: callbacks.setdefault(topic, cb)
    )
    node.create_timer.return_value = Mock()
    node.create_publisher.return_value = Mock()
    now_ns = time.time_ns()
    node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(
        nanoseconds=now_ns, to_msg=lambda: Header().stamp
    )
    bridge._ownership_callback_group = None
    bridge._command_callback_group = None
    bridge._telemetry_callback_group = None
    bridge._setup_ownership_ros(node, {}, Mock(), Mock(), [])

    # 500 ms delayed ACK is accepted when peer_heartbeat_timeout_sec == 1.0.
    callbacks["/status/control/ack"](
        SimpleNamespace(
            data=json.dumps(
                dict(
                    participant="teleop",
                    instance="inst-1",
                    stamp_ns=now_ns - 500_000_000,
                )
            )
        )
    )
    assert bridge.ownership.peers.get("teleop") == "inst-1"

    # > 1.0 s delayed ACK is still rejected.
    callbacks["/status/control/ack"](
        SimpleNamespace(
            data=json.dumps(
                dict(
                    participant="solver",
                    instance="inst-2",
                    stamp_ns=now_ns - 1_200_000_000,
                )
            )
        )
    )
    assert "solver" not in bridge.ownership.peers


def test_intervention_teleop_heartbeat_rate_and_auto_permit_tolerance(monkeypatch):
    monkeypatch.setitem(
        sys.modules, "std_msgs.msg", types.SimpleNamespace(Header=object, String=SimpleNamespace)
    )
    monkeypatch.setitem(
        sys.modules, "sensor_msgs.msg", types.SimpleNamespace(JointState=object)
    )
    monkeypatch.setitem(
        sys.modules, "geometry_msgs.msg", types.SimpleNamespace(PoseArray=object)
    )
    monkeypatch.setitem(
        sys.modules, "rclpy.qos", types.SimpleNamespace(qos_profile_sensor_data=1)
    )
    timers = []
    fake_node = SimpleNamespace(
        control={},
        backend="v23",
        config={
            "grippers": {
                "right": {"finger_topic": "/action/right_hand/joint_states"},
                "left": {"finger_topic": "/action/left_hand/joint_states"},
            }
        },
        create_publisher=lambda *a, **kw: Mock(),
        create_subscription=lambda *a, **kw: Mock(),
        create_timer=lambda period, cb: timers.append((period, cb)),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_500_000_000)
        ),
    )
    receiver = InterventionTeleop(fake_node)
    assert timers and timers[0][0] == 0.05

    # In AUTO (motion grant, teleop not granted), 0.4 s permit delay does NOT fault.
    receiver.active = True
    receiver.granted = False
    receiver.phase = "grant"
    receiver.token = ControlToken("controller", "session", 1, "motion")
    receiver.permit_time = 1.0
    receiver.ack_error = ""
    with patch("adapters.core.intervention.time.monotonic", return_value=1.4):
        receiver.heartbeat()
    ack = json.loads(receiver.ack_pub.publish.call_args.args[0].data)
    assert ack["success"] is True and ack["reason"] == ""

    # > 1.0 s permit loss in AUTO still faults.
    with patch("adapters.core.intervention.time.monotonic", return_value=2.1):
        receiver.heartbeat()
    ack = json.loads(receiver.ack_pub.publish.call_args.args[0].data)
    assert ack["success"] is False and ack["reason"] == "control permit timeout"


def test_control_tick_skips_idle_auto_pybullet_fk_when_unobserved():
    pytest.importorskip("sensor_msgs.msg")
    import adapters.core.arm_teleop_node as teleop

    sub_counts = {"actual": 0, "record": 0, "head": 0}
    node = SimpleNamespace(
        control_tick_metrics=None,
        _control_tick_sample=None,
        _monotonic=lambda: 1.0,
        joint_state_stamp=1.0,
        control={"joint_state_timeout": 0.5},
        _command_seed=Mock(return_value={}),
        _sync_model=Mock(),
        _publish_actual_poses=Mock(),
        solver_rearm=SimpleNamespace(blocked=False),
        hardware_ready=True,
        intervention=SimpleNamespace(active=True, granted=False, tick=Mock()),
        actual_pub=SimpleNamespace(get_subscription_count=lambda: sub_counts["actual"]),
        record_actual_pub=SimpleNamespace(
            get_subscription_count=lambda: sub_counts["record"]
        ),
        head_visualization_pub=SimpleNamespace(
            get_subscription_count=lambda: sub_counts["head"]
        ),
        _publish_status=Mock(),
    )

    # Idle AUTO + 0 subscribers on FK topics -> skip _sync_model and _publish_actual_poses.
    teleop.RobotArmTeleopNode._control_tick(node)
    assert node._sync_model.call_count == 0
    assert node._publish_actual_poses.call_count == 0
    assert node.intervention.tick.call_count == 1

    # As soon as a recording subscriber connects, sync & FK resume immediately.
    sub_counts["record"] = 1
    teleop.RobotArmTeleopNode._control_tick(node)
    assert node._sync_model.call_count == 1
    assert node._publish_actual_poses.call_count == 1

    # When manual VR takeover is granted (even with 0 FK subscribers), sync & FK run.
    sub_counts["record"] = 0
    node.intervention.granted = True
    teleop.RobotArmTeleopNode._control_tick(node)
    assert node._sync_model.call_count == 2
    assert node._publish_actual_poses.call_count == 2


def test_start_sh_resolve_teleop_cpu_syntax_and_overrides():
    script = ROOT / "adapters/start.sh"
    res = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr

    snippet = (
        f"source <(sed -n '/^_resolve_teleop_cpu()/,/^}}/p' {script})\n"
        'HC_TELEOP_CPU=off _resolve_teleop_cpu; echo "|"\n'
        'HC_TELEOP_CPU=1 _resolve_teleop_cpu; echo "|"\n'
    )
    out = subprocess.run(
        ["bash", "-c", snippet], capture_output=True, text=True, check=True
    ).stdout
    assert out == "|\n1|\n"

