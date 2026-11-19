"""Isolated ROS end-to-end test replacing v23 with humanoid_motion_server in full teleop."""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


def test_full_motion_teleop_clutch_and_dagger_e2e(tmp_path, monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    import yaml

    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from middleware.core.config import validate_config
    from middleware.core.ros_bridge import RosBridge

    root = Path(__file__).resolve().parents[1]
    profile = root / "adapters/robots/x1"
    cfg = yaml.safe_load((profile / "arm_teleop.yaml").read_text())
    assert cfg["control"]["backend"] == "motion"

    monkeypatch.setenv("ROS_DOMAIN_ID", "204")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "roslogs"))

    original_init = Node.__init__

    def patched_init(node, *args, **kwargs):
        kwargs["cli_args"] = [
            *(kwargs.get("cli_args") or []),
            "--ros-args",
            "--disable-external-lib-logs",
        ]
        original_init(node, *args, **kwargs)

    monkeypatch.setattr(Node, "__init__", patched_init)

    rclpy.init()
    probe = Node("motion_teleop_e2e_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(probe)

    motion_log = (tmp_path / "motion.log").open("w+")
    bridge_log = (tmp_path / "bridge.log").open("w+")
    motion_proc = solver_bridge_proc = teleop = bridge = None

    try:
        # Ensure isolated ROS domain has no stray nodes.
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        assert probe.get_node_names() == ["motion_teleop_e2e_probe"]

        env = os.environ.copy()
        env["HC_ARM_TELEOP_CONFIG"] = str(profile / "arm_teleop.yaml")
        env["PYTHONPATH"] = f"{root}:{root / '.deps'}:{env.get('PYTHONPATH', '')}"

        motion_proc = subprocess.Popen(
            [str(root / "tools/run_motion.sh"), "--profile", str(profile)],
            cwd=root,
            env=env,
            stdout=motion_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        solver_bridge_proc = subprocess.Popen(
            [
                "/usr/bin/python3",
                "-u",
                str(root / "adapters/nodes/motion_teleop_bridge.py"),
                "--config",
                str(profile / "arm_teleop.yaml"),
            ],
            cwd=root,
            env=env,
            stdout=bridge_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        values = {n: float(q) for n, q in cfg["robot"]["initial_joints"].items()}
        for j in ("leg_1", "leg_2", "zhi"):
            values.setdefault(j, 0.0)
        hand_values = {
            n: float(q)
            for side in ("right", "left")
            for n, q in zip(
                cfg["grippers"][side]["finger_joint_names"],
                cfg["grippers"][side]["finger_open"],
            )
        }
        arm_names = [n for side in ("right", "left") for n in cfg["arms"][side]["joint_names"]]

        outputs = []
        fk = {}
        ee_actual = []
        teleop_statuses = []
        solver_targets = []
        send_program = False

        def on_cmd(msg, channel):
            outputs.append((time.monotonic(), channel, copy.deepcopy(msg), bridge.ownership.state if bridge else "OFF"))
            target_dict = values if channel == "arms" else hand_values
            target_dict.update(
                (n, float(q)) for n, q in zip(msg.name, msg.position) if math.isfinite(q)
            )

        probe.create_subscription(JointState, "/action/joint_states", lambda m: on_cmd(m, "arms"), 10)
        for side in ("right", "left"):
            probe.create_subscription(
                JointState, f"/action/{side}_hand/joint_states", lambda m, s=side: on_cmd(m, s), 10
            )
            probe.create_subscription(
                PoseStamped,
                f"/observation/motion/{side}_arm/ee_pose",
                lambda m, s=side: fk.update({s: m}),
                qos_profile_sensor_data,
            )
        probe.create_subscription(PoseArray, "/observation/ee_poses", ee_actual.append, 10)
        probe.create_subscription(String, "/status/teleop", lambda m: teleop_statuses.append(json.loads(m.data)), 10)
        probe.create_subscription(JointState, "/control/solver/joint_targets", solver_targets.append, 10)

        state_pub = probe.create_publisher(JointState, "/observation/joint_states", 10)
        hands_pub = probe.create_publisher(JointState, "/observation/hands/joint_states", 10)
        ready_pub = probe.create_publisher(Bool, "/status/hardware/ready", 10)
        vr_pub = probe.create_publisher(String, "/vrdata", 10)
        program_pub = probe.create_publisher(JointState, "/hc_teleop/joint_cmd", 10)
        program_hands = {
            s: probe.create_publisher(JointState, f"/hc_teleop/joint_cmd_finger_{s}", qos_profile_sensor_data)
            for s in ("right", "left")
        }

        frame = {
            "protocol_version": 3,
            "sequence": 0,
            "vr_timestamp": 0.0,
            "tracking": {"head": True, "left": True, "right": True},
            "poses": {
                "head": {"position": [0.0, 1.5, 0.0], "quaternion": [0.0, 0.0, 0.0, 1.0]},
                "left": {"position": [-0.2, 1.2, -0.3], "quaternion": [0.0, 0.0, 0.0, 1.0]},
                "right": {"position": [0.2, 1.2, -0.3], "quaternion": [0.0, 0.0, 0.0, 1.0]},
            },
            "inputs": {
                "left": {"grip": 0.0, "trigger": 0.0, "held_mask": 0, "pressed_mask": 0},
                "right": {"grip": 0.0, "trigger": 0.0, "held_mask": 0, "pressed_mask": 0},
            },
        }

        def feed():
            stamp = probe.get_clock().now().to_msg()
            s = JointState(name=list(values), position=list(values.values()), velocity=[0.0] * len(values))
            s.header.stamp = stamp
            state_pub.publish(s)
            h = JointState(name=list(hand_values), position=list(hand_values.values()))
            h.header.stamp = stamp
            hands_pub.publish(h)
            ready_pub.publish(Bool(data=True))
            frame["sequence"] += 1
            frame["vr_timestamp"] = time.monotonic()
            frame["hc_received_ns"] = time.time_ns()
            vr_pub.publish(String(data=json.dumps(frame)))
            if send_program:
                pm = JointState(name=arm_names, position=[values[n] for n in arm_names])
                pm.header.stamp = stamp
                program_pub.publish(pm)
                for side, pub in program_hands.items():
                    fn = cfg["grippers"][side]["finger_joint_names"]
                    fmsg = JointState(name=fn, position=[hand_values[n] for n in fn])
                    fmsg.header.stamp = stamp
                    pub.publish(fmsg)

        probe.create_timer(0.02, feed)

        def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.004)

        def until(cond, timeout=20.0, desc="condition"):
            deadline = time.monotonic() + timeout
            while not cond() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.004)
            assert cond(), f"Timed out waiting for {desc}; motion_log={motion_log.read() if False else ''}"

        ros_cfg = validate_config({
            "ros": {
                "domain_id": 204,
                "legacy_control_profile": str(profile),
                "command_mux": {"enabled": True, "source": "vr", "hardware_ready_timeout_sec": 2.0},
                "intervention": {"legacy_topics": True, "automatic_recording": False},
            },
            "robot_profiles": {"root": str(root / "adapters/robots"), "active": "x1"},
        })["ros"]
        bridge = RosBridge(ros_cfg, lambda *_: None)
        bridge.start()
        teleop = RobotArmTeleopNode(str(profile / "arm_teleop.yaml"))
        executor.add_node(teleop)

        until(
            lambda: len(fk) == 2
            and len(ee_actual) > 2
            and teleop_statuses
            and teleop_statuses[-1]["backend"] == "motion"
            and teleop_statuses[-1]["generic_controller"]["solver_fresh"]
            and teleop_statuses[-1]["generic_controller"]["command_fresh"]
            and bridge.control_ready(),
            timeout=25.0,
            desc="motion stack ready",
        )
        ready_rejected = bridge.status()["command_mux"]["rejected"]

        # 1. Ordinary VR teleop: move both hands forward (+Z VR -> +X robot).
        ee0 = copy.deepcopy(ee_actual[-1])
        frame["inputs"]["right"]["grip"] = 1.0
        until(lambda: teleop_statuses[-1]["mode"] == "arms_grippers", timeout=8.0, desc="right grip engage")
        for _ in range(35):
            frame["poses"]["right"]["position"][2] += 0.0025
            frame["poses"]["left"]["position"][2] += 0.0025
            pump(0.02)
        pump(1.2)
        dx_r = ee_actual[-1].poses[0].position.x - ee0.poses[0].position.x
        dx_l = ee_actual[-1].poses[1].position.x - ee0.poses[1].position.x
        assert dx_r > 0.035 and dx_l > 0.035, (dx_r, dx_l)

        # 2. Release right Grip -> clutch hold freezes arm output.
        frame["inputs"]["right"]["grip"] = 0.0
        until(
            lambda: any(m.header.frame_id == "solver_clutch_hold" for m in solver_targets[-15:]),
            timeout=5.0,
            desc="clutch hold",
        )
        ee_held = copy.deepcopy(ee_actual[-1])
        for _ in range(15):
            frame["poses"]["right"]["position"][2] += 0.004
            pump(0.02)
        assert abs(ee_actual[-1].poses[0].position.x - ee_held.poses[0].position.x) < 0.005

        # 3. Re-press right Grip -> clutch resume resets from feedback & tracks again.
        frame["inputs"]["right"]["grip"] = 1.0
        until(lambda: teleop_statuses[-1]["mode"] == "arms_grippers", timeout=8.0, desc="clutch resume")
        ee_before_resume = copy.deepcopy(ee_actual[-1])
        for _ in range(30):
            frame["poses"]["right"]["position"][2] += 0.0025
            pump(0.02)
        pump(1.0)
        assert ee_actual[-1].poses[0].position.x - ee_before_resume.poses[0].position.x > 0.025
        pre_dagger_mux = bridge.status()["command_mux"]
        assert pre_dagger_mux["rejected"] == ready_rejected, (
            ready_rejected,
            pre_dagger_mux["rejected"],
            pre_dagger_mux["rejection_reasons"],
            bridge._boundary.last_rejection,
        )

        # 4. DAgger ControlOwnership: AUTO -> MANUAL -> home -> AUTO -> disable.
        frame["inputs"]["right"]["grip"] = 0.0
        pump(0.25)
        send_program = True
        own = bridge.ownership
        until(lambda: own.required_peers <= own.peers.keys(), timeout=8.0, desc="ownership peers")
        own.enable()
        until(lambda: own.input_valid and own.input_armed and bridge.control_ready(), timeout=6.0, desc="ownership input armed")
        own.start_auto(recording_ready=False, control_ready=bridge.control_ready())
        until(lambda: own.state == "AUTO", timeout=6.0, desc="ownership AUTO")

        frame["inputs"]["right"]["grip"] = 1.0
        until(lambda: own.state == "MANUAL", timeout=6.0, desc="ownership MANUAL")
        ee_manual_0 = copy.deepcopy(ee_actual[-1])
        for _ in range(25):
            frame["poses"]["right"]["position"][2] += 0.0025
            pump(0.02)
        pump(0.8)
        assert ee_actual[-1].poses[0].position.x - ee_manual_0.poses[0].position.x > 0.02

        before_home = len(outputs)
        assert bridge.request_manual_home()
        until(
            lambda: any(m.header.frame_id == "home" for _, ch, m, _ in outputs[before_home:] if ch == "arms"),
            timeout=6.0,
            desc="DAgger manual home started",
        )
        until(lambda: not teleop.homing, timeout=12.0, desc="DAgger manual home complete")
        for n in cfg["control"].get("home_joint_names") or arm_names:
            assert abs(values[n] - cfg["robot"]["initial_joints"][n]) <= cfg["control"].get("home_tolerance", 0.05) + 0.01

        frame["inputs"]["right"]["grip"] = 0.0
        until(lambda: own.state == "AUTO", timeout=6.0, desc="ownership AUTO after release")
        own.disable()
        until(lambda: teleop.control_source == "vr" and not teleop.intervention.active, timeout=6.0, desc="ownership disable")

        mux = bridge.status()["command_mux"]
        assert mux["forwarded"] > 500
        assert bridge._boundary.last_rejection == "", bridge._boundary.last_rejection
        assert not any(
            reason.endswith(("invalid_target", "boundary_rejected", "commit_rejected"))
            for reason in mux["rejection_reasons"]
        ), mux["rejection_reasons"]
    finally:
        if bridge:
            bridge.ownership.disable()
            bridge.stop()
        if teleop:
            executor.remove_node(teleop)
            teleop.destroy_node()
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown()
        for proc in (solver_bridge_proc, motion_proc):
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        motion_log.close()
        bridge_log.close()
