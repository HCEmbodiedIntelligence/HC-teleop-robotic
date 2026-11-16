import asyncio
import copy
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from hc_platform.adapters.ipc import CoreConnection
from hc_platform.data import DataClient, DataServer, DataService
from hc_platform.ipc import CoreClient
from hc_platform.replay import (
    Point,
    ReplayClock,
    ReplayError,
    ReplayManager,
    Trajectory,
    load_trajectory,
    validate_collisions,
    validate_contract,
)
from hc_platform.supervisor import Supervisor


def configuration():
    return {
        "robot_id": "x1",
        "config_hash": "replay-fixture-v1",
        "command_interface_version": "hc-control-ipc/v1",
        "joint_names": ["joint"],
        "joint_units": ["rad"],
        "lower": [-2],
        "upper": [2],
        "velocity": [1.0],
        "acceleration": [10.0],
        "home": [0.0],
        "groups": {"right_arm": ["joint"]},
        "allowed_groups": ["right_arm"],
        "control_hz": 100,
        "feedback_timeout_ms": 500,
        "command_timeout_ms": 200,
        "lease_timeout_ms": 1000,
    }


def manifest(config):
    return {
        "session": {
            "spec": {
                "robot_id": config["robot_id"],
                "config_hash": config["config_hash"],
                "robot_config": copy.deepcopy(config),
            }
        }
    }


def test_driver_contract_requires_units_version_and_exact_config():
    config = configuration()
    value = manifest(config)
    validate_contract(value, config)
    value["session"]["spec"]["robot_config"].pop("joint_units")
    with pytest.raises(ReplayError, match="REPLAY_JOINT_UNITS_REQUIRED"):
        validate_contract(value, config)
    value = manifest(config)
    value["session"]["spec"]["config_hash"] = "different"
    with pytest.raises(ReplayError, match="REPLAY_CONFIG_MISMATCH"):
        validate_contract(value, config)


def test_speed_acceleration_and_loop_are_prevalidated():
    config = configuration()
    points = [
        Point(index * 0.01, (0.01 * math.sin(math.pi * index / 100) ** 2,))
        for index in range(101)
    ]
    trajectory = Trajectory(["joint"], points, ["right_arm"], config)
    trajectory.validate(1, True)
    trajectory.validate(2, True)
    fast = Trajectory(
        ["joint"], [Point(0, (0.0,)), Point(0.01, (1.0,))], ["right_arm"], config
    )
    with pytest.raises(ReplayError, match="REPLAY_VELOCITY_LIMIT"):
        fast.validate(1)
    bad_loop = Trajectory(
        ["joint"],
        [Point(i * 0.01, (i * 0.00001,)) for i in range(101)],
        ["right_arm"],
        config,
    )
    with pytest.raises(ReplayError, match="REPLAY_LOOP"):
        bad_loop.validate(1, True)
    with pytest.raises(ReplayError) as error:
        trajectory.check_alignment(
            {"joint_names": ["joint"], "positions": [1.0], "feedback_age_ns": 0}
        )
    assert error.value.code == "REPLAY_START_POSE_MISMATCH"
    with pytest.raises(ReplayError, match="REPLAY_FEEDBACK_STALE"):
        trajectory.check_alignment(
            {
                "joint_names": ["joint"],
                "positions": [0.0],
                "feedback_age_ns": 1_000_000_000,
            }
        )


def test_clock_pause_and_speed_preserve_media_position():
    clock = ReplayClock(1)
    clock.rebase(2)
    clock.set_speed(2)
    assert clock.elapsed() == 2
    clock.resume()
    clock.pause()
    position = clock.elapsed()
    clock.set_speed(0.5)
    assert clock.elapsed() == position


def test_collision_preflight_requires_full_feedback_and_checks_every_segment(
    monkeypatch,
):
    config = configuration()
    config["teleop"] = {"control": {"collision_avoidance_enabled": True}}
    trajectory = Trajectory(
        ["joint"],
        [Point(0, (0.0,)), Point(0.01, (0.0001,)), Point(0.02, (0.0002,))],
        ["right_arm"],
        config,
    )
    with pytest.raises(ReplayError, match="COLLISION_FEEDBACK_INCOMPLETE"):
        validate_collisions(trajectory, {"joint_names": [], "positions": []}, config)
    calls = []

    class Checker:
        def __init__(self, *_):
            pass

        def check_path(self, current, target, deadline):
            calls.append((dict(current), dict(target)))
            return SimpleNamespace(
                safe=len(calls) < 3, reason="COLLISION_BLOCKED", samples=1
            )

        def close(self):
            pass

    monkeypatch.setattr("hc_platform.adapters.model.UrdfModel", lambda *_: object())
    monkeypatch.setattr("hc_platform.adapters.collision.CollisionChecker", Checker)
    with pytest.raises(ReplayError, match="COLLISION_BLOCKED"):
        validate_collisions(
            trajectory,
            {"joint_names": ["joint"], "positions": [0.0], "feedback_age_ns": 0},
            config,
        )
    assert len(calls) == 3
    assert calls[1][0]["joint"] == 0.0
    assert calls[2][0]["joint"] == 0.0001


def test_stop_cancels_pending_start_before_artifact_read_finishes(tmp_path):
    async def scenario():
        entered, release = asyncio.Event(), asyncio.Event()
        calls = []

        class API:
            async def request(self, op, **kwargs):
                calls.append(op)
                return {
                    "state": {
                        "boot_id": "boot",
                        "epoch": 1 if op == "status" else 2,
                        "mode": "idle",
                        "safety": "ready",
                        "transition_pending": False,
                    }
                }

        class Data:
            async def status(self):
                return {"active_session_id": None}

            async def get_dataset(self, *_):
                entered.set()
                await release.wait()
                return {"state": "READY", "format": "mcap", "manifest": {"assets": []}}

        class SupervisorStub:
            children = {}

            async def stop(self, *_args, **_kwargs):
                pass

            async def start(self, *_args, **_kwargs):
                pytest.fail("Canceled preparation must never spawn a worker")

        runtime = SimpleNamespace(
            run_dir=tmp_path,
            core=API(),
            data=Data(),
            supervisor=SupervisorStub(),
            config=configuration(),
        )
        manager = ReplayManager(runtime)
        pending = asyncio.create_task(manager.start("asset", mode="drive"))
        await entered.wait()
        assert (await manager.status())["phase"] == "PREPARING"
        await asyncio.wait_for(manager.stop(), 0.5)
        assert "stop" in calls
        release.set()
        with pytest.raises(ReplayError) as error:
            await pending
        assert error.value.code == "REPLAY_START_CANCELED"
        assert (await manager.status())["phase"] == "STOPPED"

    asyncio.run(scenario())


async def make_recording(data, config, duration=2.0):
    session = await data.start_session(
        manifest(config)["session"]["spec"], "trajectory"
    )
    for index in range(int(duration * 100) + 1):
        t = index * 0.01
        await data.append_sample(
            session["id"],
            "/platform/control/commands",
            {
                "type": "output",
                "stage": "core_gate",
                "boot_id": "recorded-old-boot",
                "config_hash": config["config_hash"],
                "epoch": 999,
                "lease_id": 999,
                "reference_epoch": 999,
                "seq": index + 1,
                "issued_mono_ns": 1_000_000_000 + index * 10_000_000,
                "deadline_mono_ns": 1_100_000_000 + index * 10_000_000,
                "mode": "position",
                "joint_names": ["joint"],
                "positions": [0.01 * math.sin(math.pi * t / duration) ** 2],
                "grippers": [0.1, 0.2],
                "grippers_valid": [False, True],
                "base_velocity": [0, 0, 0],
            },
        )
    final = await data.stop_session(session["id"])
    return await data.get_dataset(final["artifact_id"])


def test_recorded_gate_loads_named_plan_and_partial_gripper_mask(tmp_path):
    async def scenario():
        config = configuration()
        config["enable_grippers"] = True
        data = await DataService(tmp_path).open()
        try:
            artifact = await make_recording(data, config, 1)
            path = Path(artifact["root"]) / "capture.mcap"
            trajectory = load_trajectory(
                path,
                artifact["manifest"],
                config,
                {"speed": 1, "include_grippers": True},
            )
            assert trajectory.names == ["joint"]
            assert trajectory.points[0].grippers_valid == (False, True)
            assert trajectory.duration == 1
        finally:
            await data.close()

    asyncio.run(scenario())


def test_drive_worker_uses_fresh_fencing_pause_resume_and_stops(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    binary = repo / "platform/control/build/hc-control"
    if not binary.exists():
        pytest.skip("Build the C++ control core first")

    async def scenario():
        run_dir = tmp_path / "run"
        run_dir.mkdir(mode=0o700)
        config = configuration()
        config_path = run_dir / "config.json"
        config_path.write_text(json.dumps(config))
        supervisor = Supervisor(repo)
        service = await DataService(tmp_path / "data").open()
        server = await DataServer(service, run_dir / "data.sock").start()
        data = DataClient(run_dir / "data.sock")
        api = CoreClient(run_dir / "control.sock")
        feedback_task = backend = manager = None
        position, outputs = [0.0], []
        try:
            child = await supervisor.start(
                "control",
                [
                    str(binary),
                    "--socket",
                    str(run_dir / "control.sock"),
                    "--config",
                    str(config_path),
                ],
            )
            await supervisor.wait_path(run_dir / "control.sock", child)

            async def on_backend(event):
                if event.get("type") == "fence":
                    await backend.send(
                        {
                            "op": "fence_ack",
                            "boot_id": event["boot_id"],
                            "epoch": event["epoch"],
                            "stopped": True,
                        }
                    )
                elif event.get("type") == "output":
                    outputs.append(event)
                    if event.get("positions"):
                        position[:] = event["positions"]

            backend = CoreConnection(
                run_dir / "control.sock",
                "backend",
                on_backend,
                {"fence_ack": True, "physical_stop_ack": False},
            )
            await backend.start()

            async def feedback():
                sequence = 0
                while True:
                    sequence += 1
                    await backend.send(
                        {
                            "op": "feedback",
                            "seq": sequence,
                            "joint_names": ["joint"],
                            "positions": position[:],
                            "sample_mono_ns": __import__("time").monotonic_ns(),
                            "ready": True,
                        }
                    )
                    await asyncio.sleep(0.01)

            feedback_task = asyncio.create_task(feedback())
            await asyncio.sleep(0.05)
            await api.request("recover")
            for _ in range(100):
                state = (await api.request("status"))["state"]
                if state["safety"] == "ready" and not state["transition_pending"]:
                    break
                await asyncio.sleep(0.01)
            artifact = await make_recording(service, config)
            runtime = SimpleNamespace(
                repo=repo,
                run_dir=run_dir,
                config=config,
                supervisor=supervisor,
                data=data,
                core=api,
                environment=lambda: os.environ.copy(),
            )
            manager = ReplayManager(runtime)
            started = await manager.start(artifact["id"], mode="drive")
            assert started["phase"] == "RUNNING", (started, list(supervisor.logs))
            await asyncio.sleep(0.1)
            paused = await manager.pause()
            assert paused["phase"] == "PAUSED"
            media_time = paused["elapsed_s"]
            await asyncio.sleep(0.1)
            assert abs((await manager.status())["elapsed_s"] - media_time) < 0.02
            changed = await manager.set_speed(2)
            assert changed["speed"] == 2
            resumed = await manager.resume()
            assert resumed["phase"] == "RUNNING"
            for _ in range(300):
                status = await manager.status()
                if status["phase"] in {"COMPLETED", "FAILED"}:
                    break
                await asyncio.sleep(0.01)
            assert status["phase"] == "COMPLETED", (status, list(supervisor.logs))
            if manager.monitor:
                await manager.monitor
            state = (await api.request("status"))["state"]
            assert state["safety"] == "stopped"
            assert all(output["boot_id"] != "recorded-old-boot" for output in outputs)
            assert any(
                output["positions"] and output["positions"][0] > 0.001
                for output in outputs
            )
            assert (await service.get_dataset(artifact["id"]))["references"] == []
        finally:
            if manager:
                await manager.close()
            if feedback_task:
                feedback_task.cancel()
                await asyncio.gather(feedback_task, return_exceptions=True)
            if backend:
                await backend.close()
            await api.close()
            await data.close()
            await server.close()
            await service.close()
            await supervisor.close()

    asyncio.run(scenario())
