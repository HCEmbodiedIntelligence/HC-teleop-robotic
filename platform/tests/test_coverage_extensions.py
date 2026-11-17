"""Regression tests for multi-robot, capture-plan, preflight, VR events, and API errors."""

import asyncio
import copy
import json
from pathlib import Path
import time
from types import SimpleNamespace as NS

from fastapi.testclient import TestClient
import httpx
import pytest

from hc_platform.api import create_app
from hc_platform.capture_plan import validate_capture_plan
from hc_platform.configuration import ConfigurationError, PackageRegistry
from hc_platform.media import create_media_app
from hc_platform.replay import ReplayManager
from hc_platform.runtime import Runtime
from hc_platform.topics import TopicRegistry

REPO = Path(__file__).resolve().parents[2]


class _FakeCore:
    def __init__(self):
        self.state = {
            "boot_id": "test-boot",
            "epoch": 1,
            "mode": "idle",
            "safety": "ready",
            "owner": "none",
            "issued_mono_ns": time.monotonic_ns(),
        }

    async def request(self, op, **_values):
        if op == "stop":
            self.state.update(epoch=self.state["epoch"] + 1, safety="stopped")
        elif op == "recover":
            self.state.update(safety="ready")
        self.state["issued_mono_ns"] = time.monotonic_ns()
        return {"ok": True, "state": copy.deepcopy(self.state)}

    async def close(self):
        pass


class _FakeAux:
    busy = False

    async def status(self):
        return {"phase": "idle"}

    async def stop(self):
        return {"stopped": True}

    async def close(self):
        pass


async def _make_runtime(tmp_path):
    runtime = Runtime(REPO, tmp_path)
    runtime.core = _FakeCore()
    runtime.config = {"config_hash": "test", "groups": {}, "joint_names": []}
    runtime.topics = TopicRegistry(tmp_path / "topics.json")
    app = create_app(runtime)
    runtime.motion = _FakeAux()
    runtime.replay = _FakeAux()
    return runtime, app


async def _release_clients(runtime):
    await runtime.media.aclose()
    for client in (runtime.data, runtime.audit_data, runtime.status_data):
        await client.close()


def test_capture_plan_respects_disabled_cameras_empty_names_and_local_lerobot():
    plan = {
        "source_format": "MCAP",
        "effective_adapter": {
            "feature_sources": {
                "observation.state": "observation.state",
                "action": "action",
                "observation.images.head": "observation.images.head",
            },
            "configuration": {
                "cameras": [
                    {
                        "rgb": "observation.images.head",
                        "rgb_topic": "/cameras/head/color/image_raw",
                    }
                ]
            },
        },
        "output": {
            "frequency_hz": 30,
            "features": {
                "observation.state": {"dtype": "float32", "names": ["j1"]},
                "action": {"dtype": "float32", "names": ["j1"]},
                "observation.images.head": {"dtype": "video", "role": "rgb"},
            },
        },
    }
    config = {
        "joint_names": ["j1"],
        "joint_units": ["rad"],
        "groups": {"right_arm": ["j1"]},
        "allowed_groups": ["right_arm"],
        "camera": {
            "streams": [
                {
                    "id": "head",
                    "topic": "/cameras/head/color/image_raw",
                    "enabled": False,
                }
            ]
        },
    }
    selected = [
        {"topic": "/cameras/head/color/image_raw", "type": "sensor_msgs/msg/Image"}
    ]
    with pytest.raises(ConfigurationError):
        validate_capture_plan(
            plan,
            config,
            selected,
            {"warnings": []},
            {"local_format": "lerobot", "dataset_fps": 30},
            {"fresh_joint_names": ["j1"]},
        )

    config["camera"]["streams"][0]["enabled"] = True
    options = validate_capture_plan(
        plan,
        config,
        selected,
        {"warnings": []},
        {"local_format": "lerobot", "dataset_fps": 30},
        {"fresh_joint_names": ["j1"]},
    )
    assert options["fps"] == 30 and options["joint_names"] == ["j1"]

    bad_plan = copy.deepcopy(plan)
    bad_plan["output"]["features"]["observation.state"]["names"] = []
    with pytest.raises(ConfigurationError):
        validate_capture_plan(
            bad_plan,
            config,
            selected,
            {"warnings": []},
            {"local_format": "lerobot", "dataset_fps": 30},
            {"fresh_joint_names": ["j1"]},
        )


def test_preflight_blocks_auto_upload_without_task_and_bad_adhoc_topic(tmp_path):
    async def run():
        runtime, _ = await _make_runtime(tmp_path)

        class Media:
            async def get(self, path, params=None, **_kwargs):
                if path == "/validate-types":
                    return httpx.Response(
                        200,
                        json={"results": {"missing/msg/Bad": "ModuleNotFoundError"}},
                    )
                return httpx.Response(
                    200,
                    json={"ros_available": True, "topics": [], "topic_metrics": {}},
                )

            async def aclose(self):
                pass

        await runtime.media.aclose()
        runtime.media = Media()
        runtime.status_data.status = lambda: asyncio.sleep(
            0, result={"active_session_id": None}
        )
        try:
            res = await runtime.recording_preflight(
                {
                    "auto_upload": True,
                    "topics": [
                        {
                            "topic": "/adhoc",
                            "type": "missing/msg/Bad",
                            "outputs": ["record"],
                            "required": False,
                        }
                    ],
                }
            )
            reasons = {item["reason"] for item in res["blockers"]}
            assert not res["ready"]
            assert "SUBSCRIPTION_UNAVAILABLE" in reasons
            assert "PLATFORM_TASK_REQUIRED" in reasons
        finally:
            await _release_clients(runtime)

    asyncio.run(run())


def test_package_delete_tombstone_collision_and_media_publish_400(tmp_path):
    registry = PackageRegistry(REPO / "adapters/robots", tmp_path / "state")
    archive = tmp_path / "x1.zip"
    registry.export_zip("x1", archive)
    registry.import_zip(archive, "custom-del")
    # Pre-existing stale tombstone must not break subsequent delete().
    (registry.root / ".deleted-custom-del").mkdir()
    registry.delete("custom-del")
    assert not (registry.root / "custom-del").exists()

    media_app = create_media_app({"camera": {"streams": []}, "topics": []})
    with TestClient(media_app) as mclient:
        bad = mclient.post(
            "/publish",
            json={"topic": "/external/t", "type": "bad/msg/Type", "message": {}},
        )
        assert bad.status_code in (400, 503)
        assert "detail" in bad.json()


def test_vr_replay_and_mark_notifications_and_preview_404(tmp_path):
    async def run():
        runtime, app = await _make_runtime(tmp_path)
        events = []
        runtime.notify_vr = lambda kind, payload: events.append((kind, payload))
        runtime.active_recording = "rec-1"
        await runtime.audit_data.close()
        runtime.audit_data = NS(
            append_sample=lambda **kw: asyncio.sleep(0, result={"ok": True, **kw}),
            close=lambda: asyncio.sleep(0),
        )

        class MediaStub:
            async def get(self, _path, **_kw):
                return httpx.Response(200, json={"streams": []})

            async def aclose(self):
                pass

        await runtime.media.aclose()
        runtime.media = MediaStub()

        replay = ReplayManager(runtime)

        async def fake_command(op, **params):
            return {
                "id": "r1",
                "phase": "PAUSED" if op == "pause" else "RUNNING",
                "mode": "drive",
                "artifact_id": "a1",
            }

        replay._command = fake_command
        replay.local_state = {
            "id": "r1",
            "phase": "RUNNING",
            "mode": "drive",
            "artifact_id": "a1",
        }
        replay.status_path.write_text(json.dumps(replay.local_state))
        runtime.replay = replay
        try:
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                preview = await client.get("/api/cameras/missing/preview")
                assert preview.status_code == 404
                marked = await client.post(
                    "/api/recordings/mark", json={"label": "clip-1"}
                )
                assert marked.status_code == 200

            await replay.pause()
            await replay.resume()
            stopped = await replay.stop()
            assert stopped["phase"] == "STOPPED"

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                op = await client.post(
                    "/api/control/operations",
                    json={
                        "op": "recover",
                        "request_id": "recov-1",
                        "expected_boot_id": runtime.core.state["boot_id"],
                        "expected_epoch": runtime.core.state["epoch"],
                    },
                )
                assert op.status_code == 202

            kinds = [kind for kind, _ in events]
            assert "recording_marked" in kinds
            assert "replay_status" in kinds
            assert "replay_started" in kinds
            assert "replay_stopped" in kinds
            assert "replay_reset" in kinds
            assert (await replay.status())["phase"] == "IDLE"
        finally:
            await _release_clients(runtime)

    asyncio.run(run())


def test_x1_adapter_halt_holds_latched_position_without_chasing_sag():
    from hc_platform.adapters.x1 import X1RosAdapter

    config = {
        "joint_names": ["Joint1_R", "Joint2_R"],
        "groups": {"right_arm": ["Joint1_R", "Joint2_R"]},
        "allowed_groups": ["right_arm"],
        "hardware": {"allowed_groups": ["right_arm"]},
    }
    adapter = X1RosAdapter(config, lambda _fb: None)
    published = []

    class FakeClock:
        def now(self):
            return NS(to_msg=lambda: NS(sec=0, nanosec=0))

    class FakeJointState:
        def __init__(self):
            self.header = NS(stamp=None)
            self.name = []
            self.position = []

    adapter.node = NS(get_clock=lambda: FakeClock())
    adapter._JointState = FakeJointState
    adapter.joint_pub = NS(publish=lambda msg: published.append(dict(zip(msg.name, msg.position))))
    adapter.hand_pubs = {}
    adapter.base_pub = None
    adapter.ready = True
    adapter.ready_at = time.monotonic()

    def push_fb(p1, p2):
        now = time.monotonic()
        adapter.feedback = {"Joint1_R": p1, "Joint2_R": p2}
        adapter.feedback_at = {"Joint1_R": now, "Joint2_R": now}

    adapter.gate.fence("b", 1)
    push_fb(0.5, 0.6)
    now_ns = time.monotonic_ns()
    adapter.submit(
        {
            "boot_id": "b",
            "epoch": 1,
            "lease_id": 0,
            "reference_epoch": 1,
            "seq": 1,
            "mode": "stop",
            "joint_names": ["Joint1_R", "Joint2_R"],
            "positions": [0.5, 0.6],
            "deadline_mono_ns": now_ns + 200_000_000,
        }
    )
    adapter._tick()
    assert published[-1] == {"Joint1_R": 0.5, "Joint2_R": 0.6}

    push_fb(0.42, 0.51)
    adapter.submit(
        {
            "boot_id": "b",
            "epoch": 1,
            "lease_id": 0,
            "reference_epoch": 1,
            "seq": 2,
            "mode": "stop",
            "joint_names": ["Joint1_R", "Joint2_R"],
            "positions": [0.42, 0.51],
            "deadline_mono_ns": time.monotonic_ns() + 200_000_000,
        }
    )
    adapter._tick()
    assert published[-1] == {"Joint1_R": 0.5, "Joint2_R": 0.6}

    adapter.submit(
        {
            "boot_id": "b",
            "epoch": 1,
            "lease_id": 1,
            "reference_epoch": 1,
            "seq": 3,
            "mode": "position",
            "joint_names": ["Joint1_R", "Joint2_R"],
            "positions": [0.8, 0.9],
            "deadline_mono_ns": time.monotonic_ns() + 200_000_000,
        }
    )
    adapter._tick()
    assert published[-1] == {"Joint1_R": 0.8, "Joint2_R": 0.9}

    push_fb(0.71, 0.79)
    adapter.submit(
        {
            "boot_id": "b",
            "epoch": 1,
            "lease_id": 0,
            "reference_epoch": 1,
            "seq": 4,
            "mode": "stop",
            "joint_names": ["Joint1_R", "Joint2_R"],
            "positions": [0.71, 0.79],
            "deadline_mono_ns": time.monotonic_ns() + 200_000_000,
        }
    )
    adapter._tick()
    assert published[-1] == {"Joint1_R": 0.8, "Joint2_R": 0.9}

    adapter.ready = False
    count_before = len(published)
    adapter._halt()
    assert len(published) == count_before

    adapter.ready = True
    adapter.ready_at = time.monotonic()
    push_fb(0.3, 0.4)
    adapter._halt()
    assert published[-1] == {"Joint1_R": 0.3, "Joint2_R": 0.4}


def test_vr_control_recover_teleop_lifecycle_and_shutdown_suppression(tmp_path):
    class TeleopCore:
        def __init__(self):
            self.calls = []
            self.state = {
                "boot_id": "test-boot",
                "epoch": 1,
                "safety": "stopped",
                "mode": "idle",
                "paused": False,
                "transition_pending": False,
                "owner": "none",
                "issued_mono_ns": time.monotonic_ns(),
            }

        async def request(self, op, **values):
            self.calls.append((op, dict(values)))
            if op == "recover":
                if self.state["safety"] not in ("stopped", "fault"):
                    raise RuntimeError("RECOVERY_NOT_ALLOWED")
                self.state.update(epoch=self.state["epoch"] + 1, safety="ready", mode="idle", paused=False)
            elif op == "session_start":
                assert values["expected_epoch"] == self.state["epoch"]
                self.state.update(epoch=self.state["epoch"] + 1, mode=values["mode"], paused=False, owner="vr")
            elif op == "resume":
                assert values["expected_epoch"] == self.state["epoch"]
                self.state.update(epoch=self.state["epoch"] + 1, paused=False, owner="vr")
            elif op == "stop":
                self.state.update(epoch=self.state["epoch"] + 1, safety="stopped", mode="idle", paused=False)
            self.state["issued_mono_ns"] = time.monotonic_ns()
            return {"ok": True, "state": copy.deepcopy(self.state)}

        async def close(self):
            pass

    async def run():
        import socket

        runtime = Runtime(REPO, tmp_path, mode="teleop")
        runtime.core = TeleopCore()
        runtime.replay = _FakeAux()
        runtime.config = {"config_hash": "test", "groups": {}, "joint_names": []}
        runtime.topics = TopicRegistry(tmp_path / "topics.json")
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock_path = tmp_path / "api-events.sock"
        sock.bind(str(sock_path))
        runtime.event_socket = sock
        vr_errors = []
        runtime.notify_vr = lambda kind, payload: vr_errors.append((kind, payload))
        task = asyncio.create_task(runtime._vr_events())

        sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            def send_recover():
                sender.sendto(
                    json.dumps(
                        {
                            "kind": "control_recover",
                            "boot_id": runtime.core.state["boot_id"],
                            "epoch": runtime.core.state["epoch"],
                            "sample_mono_ns": time.monotonic_ns(),
                        }
                    ).encode(),
                    str(sock_path),
                )

            send_recover()
            for _ in range(50):
                if runtime.core.state["mode"] == "teleop":
                    break
                await asyncio.sleep(0.01)
            assert [op for op, _ in runtime.core.calls if op != "status"] == [
                "recover",
                "session_start",
            ]
            assert runtime.core.state["safety"] == "ready"
            assert runtime.core.state["mode"] == "teleop"

            # Pressing A again while already active in teleop is idempotent (no RECOVERY_NOT_ALLOWED).
            send_recover()
            await asyncio.sleep(0.05)
            assert [op for op, _ in runtime.core.calls if op != "status"] == [
                "recover",
                "session_start",
            ]
            assert not any(entry["level"] == "ERROR" for entry in runtime.supervisor.logs)

            # Pressing A while paused resumes teleop.
            runtime.core.state["paused"] = True
            runtime.core.state["epoch"] += 1
            send_recover()
            for _ in range(50):
                if not runtime.core.state["paused"]:
                    break
                await asyncio.sleep(0.01)
            assert [op for op, _ in runtime.core.calls if op != "status"] == [
                "recover",
                "session_start",
                "resume",
            ]

            # Events arriving after shutdown_requested must not spam ERROR logs.
            runtime.shutdown_requested = True
            send_recover()
            await asyncio.sleep(0.05)
            assert not any(entry["level"] == "ERROR" for entry in runtime.supervisor.logs)
            assert not vr_errors
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            sender.close()
            sock.close()
            await _release_clients(runtime)

    asyncio.run(run())


def test_capture_teardown_shuts_down_context_before_joining_executor():
    from hc_platform.capture import Capture

    capture = Capture("/unused", "sess", [])
    capture.start_ros = lambda: None
    capture.stopping = True
    events = []

    class FakeContext:
        def try_shutdown(self):
            events.append("context_shutdown")

    class FakeExecutor:
        def shutdown(self):
            events.append("executor_shutdown")

    class FakeNode:
        def destroy_node(self):
            events.append("node_destroy")

    class FakeThread:
        def join(self, timeout=None):
            events.append(("thread_join", timeout))

        def is_alive(self):
            return False

    capture.context = FakeContext()
    capture.executor = FakeExecutor()
    capture.node = FakeNode()
    capture.thread = FakeThread()

    asyncio.run(capture.run())
    assert events == [
        "context_shutdown",
        ("thread_join", 2),
        "executor_shutdown",
        "node_destroy",
    ]


def test_web_console_avoids_unguarded_crypto_random_uuid():
    web_src = REPO / "platform/web/src"
    for source in web_src.glob("*.ts*"):
        text = source.read_text()
        if "randomUUID" in text:
            assert "createRequestId" in text
            # Only the fallback guard itself may touch crypto.randomUUID.
            assert text.count("randomUUID()") == 1
            assert text.count("typeof c.randomUUID") == 1
    assets = list((REPO / "platform/web/dist/assets").glob("index-*.js"))
    if assets:
        bundle = assets[0].read_text()
        assert bundle.count("randomUUID()") <= 1
        if "randomUUID()" in bundle:
            assert 'randomUUID=="function"' in bundle or "randomUUID==='function'" in bundle
