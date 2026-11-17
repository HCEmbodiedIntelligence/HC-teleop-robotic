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


def test_legacy_ws_and_unknown_ws_do_not_crash_staticfiles(tmp_path):
    async def setup():
        return await _make_runtime(tmp_path)

    runtime, app = asyncio.run(setup())
    try:
        client = TestClient(app)
        with client.websocket_connect("/ws") as ws:
            hello = ws.receive_json()
            assert hello["kind"] == "connected"
            ws.send_json({"kind": "ping"})
            pong = ws.receive_json()
            assert pong["kind"] == "pong"
        with pytest.raises(Exception):
            with client.websocket_connect("/ws-unknown") as bad_ws:
                bad_ws.receive_text()
    finally:
        asyncio.run(_release_clients(runtime))


def test_teleop_runtime_reenters_teleop_after_home_complete(tmp_path):
    class HomeTeleopCore:
        def __init__(self):
            self.calls = []
            self.state = {
                "boot_id": "test-boot",
                "epoch": 4,
                "safety": "ready",
                "mode": "idle",
                "paused": False,
                "transition_pending": False,
                "owner": "none",
                "issued_mono_ns": time.monotonic_ns(),
            }

        async def request(self, op, **values):
            self.calls.append((op, dict(values)))
            if op == "session_start":
                self.state.update(
                    epoch=self.state["epoch"] + 1,
                    mode=values["mode"],
                    owner="vr",
                )
            return {"ok": True, "state": copy.deepcopy(self.state)}

        async def close(self):
            pass

    async def run():
        runtime = Runtime(REPO, tmp_path, mode="teleop")
        await runtime.core.close()
        runtime.core = HomeTeleopCore()
        runtime.replay = _FakeAux()
        runtime.config = {"config_hash": "test", "groups": {}, "joint_names": []}
        runtime.topics = TopicRegistry(tmp_path / "topics.json")
        runtime.audit.events.put_nowait(
            {
                "type": "ownership_transition",
                "reason": "HOME_COMPLETE",
                "boot_id": "test-boot",
                "epoch": 4,
                "mono_ns": time.monotonic_ns(),
            }
        )
        task = asyncio.create_task(runtime._control_events())
        try:
            for _ in range(50):
                if runtime.core.state["mode"] == "teleop":
                    break
                await asyncio.sleep(0.01)
            assert [op for op, _ in runtime.core.calls if op != "status"] == [
                "session_start"
            ]
            assert runtime.core.state["mode"] == "teleop"
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            if runtime._home_resume_task:
                runtime._home_resume_task.cancel()
                await asyncio.gather(runtime._home_resume_task, return_exceptions=True)
            await _release_clients(runtime)

    asyncio.run(run())


def test_sim_camera_renderer_produces_jpeg_depth_and_intrinsics():
    pytest.importorskip("pybullet")
    from PIL import Image
    import io
    from hc_platform.adapters.model import UrdfModel
    from hc_platform.adapters.sim_camera import SimCameraRenderer
    from hc_platform.configuration import compile_package

    cfg = compile_package(REPO / "adapters/robots/x1")
    cfg.setdefault("sim", {})["camera_fps"] = 30.0
    model = UrdfModel(cfg["urdf_path"], cfg["package_root"])
    renderer = SimCameraRenderer(cfg, model, cfg.get("topics", []))
    assert renderer.available
    assert any(t == "/observation/cameras/head/color/compressed" for t, _ in renderer.required_topics())
    assert any(t == "/observation/cameras/head/depth/compressed" for t, _ in renderer.required_topics())
    assert renderer.info_topic == "/observation/cameras/head/camera_info"
    assert renderer.calibration_topic == "/observation/cameras/head/calibration"

    renderer.start()
    try:
        sample = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and sample is None:
            sample = renderer.pop_latest()
            if sample is None:
                time.sleep(0.02)
        assert sample is not None
        rgb_payloads, depth_payloads, primary_meta, stamp_ns, mono_ns = sample
        assert len(rgb_payloads) == 1
        topic, kind, w, h, frame_id, data = rgb_payloads[0]
        assert topic == "/observation/cameras/head/color/compressed"
        assert kind == "CompressedImage" and (w, h) == (640, 400)
        img = Image.open(io.BytesIO(data))
        assert img.size == (640, 400)
        assert len(depth_payloads) == 1
        d_topic, _, png_bytes = depth_payloads[0]
        assert d_topic == "/observation/cameras/head/depth/compressed"
        assert Image.open(io.BytesIO(png_bytes)).size == (640, 400)
        assert primary_meta[:2] == (640, 400) and primary_meta[2] > 0
        assert stamp_ns > 0 and mono_ns > 0
    finally:
        renderer.close()


def test_real_sim_camera_capture_preview_and_lerobot_export(tmp_path):
    pytest.importorskip("rclpy")
    pytest.importorskip("pybullet")
    import os
    from hc_platform.data.sample_codec import iter_samples

    converter = Path(
        os.environ.get(
            "HC_CONVERTER_PYTHON", REPO / ".venv-lerobot-platform/bin/python"
        )
    )
    if not (REPO / "platform/control/build/hc-control").exists() or not converter.exists():
        pytest.skip("Build hc-control and converter environment first")

    def free_port():
        with __import__("socket").socket(__import__("socket").AF_INET, __import__("socket").SOCK_DGRAM) as s:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]

    runtime = Runtime(REPO, tmp_path, vr_port=free_port(), discovery_port=free_port())
    with TestClient(create_app(runtime)) as client:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            st = client.get("/api/status").json()
            cams = client.get("/api/cameras").json()["streams"]
            head = next(c for c in cams if c["id"] == "head")
            if st["control"].get("backend_ready") and st["backend"].get("process_alive") and head.get("source_alive"):
                break
            time.sleep(0.05)
        cams = client.get("/api/cameras").json()["streams"]
        head = next(c for c in cams if c["id"] == "head")
        assert head["source_alive"] and head["frames_received"] > 0
        snap = client.get("/api/cameras/head/snapshot")
        assert snap.status_code == 200 and snap.headers["content-type"].startswith("image/jpeg")

        assert client.post("/api/control/operations", json={"op": "recover", "request_id": "recov"}).status_code == 202
        for _ in range(100):
            if client.get("/api/status").json()["control"]["safety"] == "ready":
                break
            time.sleep(0.02)

        rec = client.post(
            "/api/recordings",
            json={
                "format": "mcap_lerobot",
                "local_format": "mcap_lerobot",
                "filename": "sim_full.mcap",
                "dataset_fps": 10,
                "task_description": "sim camera reach",
                "request_id": "sim-e2e-rec",
            },
        )
        assert rec.status_code == 201, rec.text
        aid = rec.json()["id"]

        assert client.post(
            "/api/control/operations",
            json={"op": "session_start", "mode": "program", "groups": ["right_arm"]},
        ).status_code == 202
        while client.get("/api/status").json()["control"]["transition_pending"]:
            time.sleep(0.02)

        for seq in range(1, 35):
            cur = client.get("/api/status").json()["control"]
            names = runtime.config["groups"]["right_arm"]
            pos = dict(zip(cur["joint_names"], cur["positions"]))
            payload = {k: cur[k] for k in ("boot_id", "epoch", "lease_id", "reference_epoch")}
            payload.update(
                source="program",
                seq=seq,
                ttl_ms=150,
                joint_names=names,
                positions=[pos[n] + (0.015 if i == 0 else 0.0) for i, n in enumerate(names)],
            )
            assert client.post("/api/control/command", json=payload).status_code == 200
            time.sleep(0.02)

        stop = client.post("/api/recordings/stop")
        assert stop.status_code == 200, stop.text
        mcap_path = Path(stop.json()["root"]) / "sim_full.mcap"
        gaps = [s for s in iter_samples(mcap_path, ["/capture/gaps"])]
        assert not gaps

        for _ in range(300):
            datasets = [d for d in client.get("/api/datasets").json() if d.get("session_id") == aid]
            lerobot_ds = [d for d in datasets if d.get("format") == "lerobot" and d.get("state") == "READY"]
            if lerobot_ds:
                assert lerobot_ds[0]["summary"]["camera_count"] >= 1
                assert lerobot_ds[0]["summary"]["frame_count"] > 0
                break
            failed = [d for d in datasets if d.get("state") in ("FAILED", "DERIVATION_FAILED")]
            assert not failed, failed
            time.sleep(0.1)
        else:
            raise AssertionError("Timed out waiting for LeRobot simulation dataset conversion")


def test_audit_dropped_events_resets_after_reporting(tmp_path):
    class _AuditDataStub:
        def __init__(self):
            self.samples = []

        async def append_sample(self, *, session_id, topic, payload):
            self.samples.append((session_id, topic, dict(payload)))

        async def close(self):
            pass

    async def run():
        runtime = Runtime(REPO, tmp_path, mode="sim")
        stub = _AuditDataStub()
        await runtime.audit_data.close()
        runtime.audit_data = stub
        runtime.active_recording = "sess-1"
        runtime.audit.dropped_events = 3
        for seq in (1, 2):
            runtime.audit.events.put_nowait(
                {"type": "output", "boot_id": "b", "epoch": 1, "seq": seq, "mono_ns": seq}
            )
        task = asyncio.create_task(runtime._control_events())
        try:
            for _ in range(50):
                if len(stub.samples) >= 3:
                    break
                await asyncio.sleep(0.01)
            gap_samples = [s for s in stub.samples if s[1] == "/capture/gaps"]
            assert len(gap_samples) == 1
            assert gap_samples[0][2]["dropped"] == 3
            assert runtime.audit.dropped_events == 0
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            await _release_clients(runtime)

    asyncio.run(run())


def test_simulation_adapter_clamps_joint_limit_and_stays_ready(tmp_path):
    pytest.importorskip("pybullet")
    from hc_platform.adapters.simulation import SimulationAdapter
    from hc_platform.configuration import compile_package

    cfg = compile_package(REPO / "adapters/robots/x1")
    feedbacks = []
    adapter = SimulationAdapter(cfg, feedbacks.append)
    adapter.start()
    try:
        j6_idx = cfg["joint_names"].index("Joint6_R")
        upper_j6 = cfg["upper"][j6_idx]
        adapter.targets["Joint6_R"] = upper_j6 + 0.5
        adapter._apply_motors()
        assert adapter.targets["Joint6_R"] == upper_j6
        for _ in range(60):
            adapter.step()
        latest = feedbacks[-1]
        measured_j6 = latest["positions"][latest["joint_names"].index("Joint6_R")]
        assert measured_j6 <= upper_j6 + 0.005
    finally:
        adapter.close()


def test_data_client_and_capture_batch_append_samples(tmp_path):
    from hc_platform.capture import Capture
    from hc_platform.data import DataClient, DataServer, DataService
    from hc_platform.data.sample_codec import iter_samples

    async def run():
        sock = tmp_path / "data.sock"
        service = await DataService(tmp_path / "data").open()
        server = await DataServer(service, sock).start()
        client = DataClient(sock)
        try:
            sess = await client.start_session(
                {"robot_id": "x1", "config_hash": "test", "filename": "batch.mcap"},
                "ordinary",
            )
            capture = Capture(str(sock), sess["id"], [])
            for idx in range(5):
                capture.receive(
                    b"\x00\x01\x00\x00\x01",
                    "/status/test",
                    "std_msgs/msg/Bool",
                    "bool data\n",
                    0,
                )
            capture.start_ros = lambda: None
            capture.stopping = True
            await capture.run()
            stopped = await client.stop_session(sess["id"])
            samples = [
                s
                for s in iter_samples(Path(stopped["root"]) / "batch.mcap")
                if s["topic"] == "/status/test"
            ]
            assert len(samples) == 5
        finally:
            await client.close()
            await server.close()
            await service.close()

    asyncio.run(run())
