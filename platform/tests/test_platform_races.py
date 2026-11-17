"""Deterministic lifecycle races; no hardware or subprocesses are started."""

import asyncio
import contextlib
import copy
import json
from pathlib import Path
import socket
import time
from types import SimpleNamespace as NS

import httpx
import pytest

from hc_platform.api import create_app
from hc_platform.ipc import CoreError
from hc_platform.runtime import Runtime
from hc_platform.topics import TopicRegistry

REPO = Path(__file__).resolve().parents[2]


class FakeCore:
    def __init__(self):
        self.state = {
            "boot_id": "test-boot",
            "epoch": 1,
            "mode": "idle",
            "safety": "ready",
            "owner": "none",
            "issued_mono_ns": time.monotonic_ns(),
        }
        self.calls = []
        self.status_seen = asyncio.Event()
        self.stop_seen = asyncio.Event()

    async def request(self, op, **values):
        self.calls.append((op, values))
        if op == "status":
            self.status_seen.set()
        elif op == "stop":
            self.state.update(epoch=self.state["epoch"] + 1, safety="stopped")
            self.stop_seen.set()
        else:
            if (
                values.get("expected_boot_id", self.state["boot_id"])
                != self.state["boot_id"]
                or values.get("expected_epoch", self.state["epoch"])
                != self.state["epoch"]
            ):
                raise CoreError("EXPECTED_STATE_MISMATCH", state=self.state)
            if op == "session_start":
                self.state.update(mode=values.get("mode"), owner="program")
        self.state["issued_mono_ns"] = time.monotonic_ns()
        return {"ok": True, "state": copy.deepcopy(self.state)}

    async def close(self):
        pass


class FakeAuxiliary:
    busy = False

    async def status(self):
        return {"phase": "idle"}

    async def stop(self):
        return {"stopped": True}

    async def close(self):
        pass


async def make_runtime(tmp_path):
    runtime = Runtime(REPO, tmp_path)
    runtime.core = FakeCore()
    runtime.config = {"config_hash": "test", "groups": {}, "joint_names": []}
    runtime.topics = TopicRegistry(tmp_path / "topics.json")
    app = create_app(runtime)
    runtime.motion = FakeAuxiliary()
    runtime.replay = FakeAuxiliary()
    return runtime, app


async def release_clients(runtime):
    await runtime.media.aclose()
    for client in (runtime.data, runtime.audit_data, runtime.status_data):
        await client.close()


def test_preflight_discovered_topic_without_first_sample_is_a_blocker(tmp_path):
    async def run():
        runtime, app = await make_runtime(tmp_path)
        spec = {
            "topic": "/state",
            "type": "sensor_msgs/msg/JointState",
            "outputs": ["record"],
            "required": True,
        }

        class Media:
            async def get(self, *args, **kwargs):
                return httpx.Response(
                    200,
                    json={
                        "ros_available": True,
                        "topics": [{"topic": "/state", "types": [spec["type"]]}],
                        "topic_metrics": {"/state": {"age_ms": None}},
                    },
                )

            async def aclose(self):
                pass

        await runtime.media.aclose()
        runtime.media = Media()

        async def status():
            return {"active_session_id": None}

        runtime.status_data.status = status
        try:
            result = await runtime.recording_preflight({"topics": [spec]})
            assert not result["ready"]
            assert result["blockers"][0]["reason"] == "NO_FRESH_DATA"
        finally:
            await release_clients(runtime)

    asyncio.run(run())


def test_preflight_blocks_unavailable_optional_subscription(tmp_path):
    async def run():
        runtime, app = await make_runtime(tmp_path)
        spec = {
            "topic": "/custom",
            "type": "missing/msg/Custom",
            "outputs": ["record"],
            "required": False,
        }

        class Media:
            async def get(self, *args, **kwargs):
                return httpx.Response(
                    200,
                    json={
                        "ros_available": True,
                        "topics": [{"topic": "/custom", "types": [spec["type"]]}],
                        "topic_metrics": {
                            "/custom": {
                                "age_ms": None,
                                "error": "SUBSCRIPTION_UNAVAILABLE: missing type",
                            }
                        },
                    },
                )

            async def aclose(self):
                pass

        await runtime.media.aclose()
        runtime.media = Media()

        async def status():
            return {"active_session_id": None}

        runtime.status_data.status = status
        try:
            result = await runtime.recording_preflight({"topics": [spec]})
            assert (
                not result["ready"]
                and result["blockers"][0]["reason"] == "SUBSCRIPTION_UNAVAILABLE"
            )
        finally:
            await release_clients(runtime)

    asyncio.run(run())


def test_dataset_pagination_and_package_references_beyond_first_page(tmp_path):
    async def run():
        runtime, app = await make_runtime(tmp_path)
        calls = []

        async def list_datasets(limit=100, offset=0):
            calls.append((limit, offset))
            return (
                [{"summary": {"robot_id": "another"}} for _ in range(limit)]
                if offset == 0
                else [{"summary": {"robot_id": "referenced"}}]
            )

        runtime.data.list_datasets = list_datasets
        try:
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                assert (
                    await client.get("/api/datasets?limit=12&offset=24")
                ).status_code == 200
                assert calls[-1] == (12, 24)
                response = await client.delete("/api/robots/referenced")
                assert (
                    response.status_code == 422
                    and "referenced" in response.json()["message"]
                )
                assert calls[-2:] == [(1000, 0), (1000, 1000)]
        finally:
            await release_clients(runtime)

    asyncio.run(run())


def test_stop_preempts_locked_start_and_cancels_the_old_operation(
    tmp_path, monkeypatch
):
    monkeypatch.delenv("HC_PLATFORM_TOKEN", raising=False)

    async def run():
        runtime, app = await make_runtime(tmp_path)
        transport = httpx.ASGITransport(app=app)
        try:
            async with httpx.AsyncClient(
                transport=transport, base_url="http://test"
            ) as client:
                await runtime.mutation_lock.acquire()
                pending = asyncio.create_task(
                    client.post(
                        "/api/control/operations",
                        json={"op": "session_start", "mode": "program"},
                    )
                )
                await asyncio.wait_for(runtime.core.status_seen.wait(), 1)
                stopped = await asyncio.wait_for(
                    client.post("/api/control/operations", json={"op": "stop"}), 1
                )
                assert stopped.status_code == 202 and runtime.core.stop_seen.is_set()
                assert not pending.done()
                runtime.mutation_lock.release()
                response = await pending
                assert (
                    response.status_code == 409
                    and response.json()["code"] == "STALE_OPERATION"
                )
                assert not any(op == "session_start" for op, _ in runtime.core.calls)
        finally:
            if runtime.mutation_lock.locked():
                runtime.mutation_lock.release()
            await release_clients(runtime)

    asyncio.run(run())


def test_stop_during_auto_record_prepare_does_not_start_program(tmp_path, monkeypatch):
    monkeypatch.delenv("HC_PLATFORM_TOKEN", raising=False)

    async def run():
        runtime, app = await make_runtime(tmp_path)
        runtime.recording_policy = {
            "enabled": True,
            "mode": "corrections",
            "required": True,
        }
        preparing = asyncio.Event()
        finish = asyncio.Event()
        sealed = []

        async def start(spec, request_id):
            assert runtime.mutation_lock.locked()
            preparing.set()
            await finish.wait()
            runtime.active_recording = "own-recording"
            return {"id": "own-recording"}

        async def stop():
            sealed.append(runtime.active_recording)
            runtime.active_recording = None

        runtime.start_recording = start
        runtime.stop_recording = stop
        try:
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                pending = asyncio.create_task(
                    client.post(
                        "/api/control/operations",
                        json={"op": "session_start", "mode": "program"},
                    )
                )
                await asyncio.wait_for(preparing.wait(), 1)
                assert (
                    await client.post("/api/control/operations", json={"op": "stop"})
                ).status_code == 202
                finish.set()
                response = await pending
                assert response.status_code == 409
                assert sealed == ["own-recording"]
                assert not any(op == "session_start" for op, _ in runtime.core.calls)
        finally:
            finish.set()
            await release_clients(runtime)

    asyncio.run(run())


def test_topic_mutations_wait_for_lifecycle_and_recheck_recording(
    tmp_path, monkeypatch
):
    monkeypatch.delenv("HC_PLATFORM_TOKEN", raising=False)

    async def run():
        runtime, app = await make_runtime(tmp_path)
        spec = {
            "topic": "/state",
            "type": "sensor_msgs/msg/JointState",
            "outputs": ["record"],
        }
        try:
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                await runtime.mutation_lock.acquire()
                put = asyncio.create_task(client.put("/api/topics", json=spec))
                delete = asyncio.create_task(
                    client.delete("/api/topics", params={"topic": "/state"})
                )
                await asyncio.sleep(0.03)
                assert (
                    runtime.topics.items == [] and not put.done() and not delete.done()
                )
                runtime.active_recording = "recording-started-under-lock"
                runtime.mutation_lock.release()
                assert (await put).status_code == 409
                assert (await delete).status_code == 409
                assert not (tmp_path / "topics.json").exists()
        finally:
            if runtime.mutation_lock.locked():
                runtime.mutation_lock.release()
            await release_clients(runtime)

    asyncio.run(run())


def test_vr_record_event_is_revalidated_after_waiting_for_config_lock(tmp_path):
    async def run():
        runtime, app = await make_runtime(tmp_path)
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        receiver.setblocking(False)
        sender.setblocking(False)
        runtime.event_socket = receiver
        failures = asyncio.Event()
        started = []
        runtime.supervisor.log = lambda *args, **kwargs: failures.set()

        async def start(*args, **kwargs):
            started.append(True)

        runtime.start_recording = start
        await runtime.mutation_lock.acquire()
        consumer = asyncio.create_task(runtime._vr_events())
        try:
            event = {
                "kind": "record_toggle",
                "sample_mono_ns": time.monotonic_ns(),
                "boot_id": "test-boot",
                "epoch": 1,
            }
            await asyncio.get_running_loop().sock_sendall(
                sender, json.dumps(event).encode()
            )
            await asyncio.wait_for(runtime.core.status_seen.wait(), 1)
            runtime.core.state["epoch"] = 2
            runtime.management_generation += 1
            runtime.mutation_lock.release()
            await asyncio.wait_for(failures.wait(), 1)
            assert started == []
        finally:
            if runtime.mutation_lock.locked():
                runtime.mutation_lock.release()
            consumer.cancel()
            await asyncio.gather(consumer, return_exceptions=True)
            receiver.close()
            sender.close()
            runtime.event_socket = None
            await release_clients(runtime)

    asyncio.run(run())


def test_long_data_rpc_does_not_block_status_or_ownership_capture(tmp_path):
    async def run():
        runtime, app = await make_runtime(tmp_path)
        runtime.active_recording = "recording"
        runtime.recording_mode = "dagger_corrections"
        entered = asyncio.Event()
        release = asyncio.Event()
        intervention = asyncio.Event()

        async def handle(reader, writer):
            try:
                while raw := await reader.readline():
                    request = json.loads(raw)
                    method = request["method"]
                    if method == "prepare_download":
                        entered.set()
                        await release.wait()
                    if method == "set_intervention":
                        intervention.set()
                    value = (
                        {"active_session_id": "recording"}
                        if method == "status"
                        else {"accepted": True, "active": True}
                    )
                    writer.write(
                        (
                            json.dumps({"id": request["id"], "result": value}) + "\n"
                        ).encode()
                    )
                    await writer.drain()
            finally:
                writer.close()
                await writer.wait_closed()

        server = await asyncio.start_unix_server(
            handle, str(runtime.run_dir / "data.sock")
        )
        long_call = asyncio.create_task(
            runtime.data.call(
                "prepare_download", {"artifact_id": "example", "owner": "test"}
            )
        )
        consumer = asyncio.create_task(runtime._control_events())
        try:
            await asyncio.wait_for(entered.wait(), 1)
            await runtime.audit.events.put(
                {
                    "type": "ownership_transition",
                    "owner": "human",
                    "boot_id": "test-boot",
                    "epoch": 2,
                    "event_seq": 1,
                    "mono_ns": time.monotonic_ns(),
                }
            )
            await asyncio.wait_for(intervention.wait(), 1)
            state = await asyncio.wait_for(runtime._get_status(), 1)
            assert state["recording"]["active_session_id"] == "recording"
            assert not long_call.done()
        finally:
            release.set()
            await long_call
            consumer.cancel()
            await asyncio.gather(consumer, return_exceptions=True)
            await release_clients(runtime)
            server.close()
            await server.wait_closed()

    asyncio.run(run())


def test_shutdown_cancellation_preserves_tail_audit_and_reaps_children(tmp_path):
    async def run():
        runtime, app = await make_runtime(tmp_path)
        runtime.active_recording = "recording"
        order = []

        async def audit_tail():
            await runtime.core.stop_seen.wait()
            order.append("tail-ack")
            await asyncio.Event().wait()

        async def stop_recording():
            assert "tail-ack" in order
            order.append("seal")
            runtime.active_recording = None

        async def stop_control():
            order.append("stop-control")

        async def supervisor_close():
            order.append("children-reaped")

        runtime.tasks = [asyncio.create_task(audit_tail())]
        runtime.stop_recording = stop_recording
        runtime.stop_control = stop_control
        runtime.supervisor.close = supervisor_close
        closing = asyncio.create_task(runtime.close())
        await runtime.core.stop_seen.wait()
        closing.cancel()
        with pytest.raises(asyncio.CancelledError):
            await closing
        assert order == ["tail-ack", "seal", "stop-control", "children-reaped"]
        assert runtime.tasks[0].cancelled()

    asyncio.run(run())


def test_offer_requires_active_vr_owner_or_valid_token(tmp_path, monkeypatch):
    monkeypatch.setenv("HC_PLATFORM_TOKEN", "video-test-token")

    async def run():
        runtime, app = await make_runtime(tmp_path)

        class Media:
            async def post(self, *args, **kwargs):
                return httpx.Response(200, json={"accepted": True})

            async def aclose(self):
                pass

        await runtime.media.aclose()
        runtime.media = Media()
        runtime.read_adapter_state = lambda role: {
            "is_locked": True,
            "active_peer": "192.0.2.10",
            "process_alive": True,
            "status_stale": False,
        }
        try:
            for peer, headers, expected in [
                ("192.0.2.11", {}, 403),
                ("192.0.2.10", {}, 200),
                ("192.0.2.11", {"Authorization": "Bearer video-test-token"}, 200),
            ]:
                async with httpx.AsyncClient(
                    transport=httpx.ASGITransport(app=app, client=(peer, 1234)),
                    base_url="http://test",
                ) as client:
                    response = await client.post(
                        "/offer/head",
                        json={"sdp": "test", "type": "offer"},
                        headers=headers,
                    )
                    assert response.status_code == expected, response.text
            runtime.read_adapter_state = lambda role: {
                "is_locked": True,
                "active_peer": "192.0.2.10",
                "process_alive": True,
                "status_stale": True,
            }
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app, client=("192.0.2.10", 1234)),
                base_url="http://test",
            ) as client:
                assert (await client.post("/offer/head", json={})).status_code == 403
        finally:
            await release_clients(runtime)

    asyncio.run(run())


def test_record_stop_waits_for_consumed_audit_watermark_and_freezes_cutoff(tmp_path):
    async def run():
        runtime, app = await make_runtime(tmp_path)
        runtime.active_recording = "recording"
        recorded = []
        sealed = []
        interventions = []

        class AuditData:
            async def append_sample(self, **sample):
                recorded.append(sample)
                return {"accepted": True}

            async def set_intervention(self, **sample):
                interventions.append(sample)
                return {"active": False}

            async def stop_session(self, **sample):
                sealed.append(sample)
                return {"id": sample["session_id"]}

            async def close(self):
                pass

        runtime.audit_data = AuditData()

        async def status():
            return {"active_session_id": "recording"}

        runtime.status_data.status = status

        async def stop_child(*args, **kwargs):
            pass

        runtime.supervisor.stop = stop_child
        consumer = asyncio.create_task(runtime._control_events())
        stopping = asyncio.create_task(runtime.stop_recording())
        try:
            while runtime.recording_cutoff_ns is None:
                await asyncio.sleep(0)
            cutoff = runtime.recording_cutoff_ns
            await runtime.audit.events.put(
                {"type": "output_ack", "mono_ns": cutoff - 1, "accepted": True}
            )
            await runtime.audit.events.put(
                {"type": "output", "issued_mono_ns": cutoff + 1}
            )
            await runtime.audit.events.put(
                {"type": "state", "issued_mono_ns": cutoff + 2, "boot_id": "test-boot"}
            )
            await asyncio.wait_for(stopping, 1)
            assert len(recorded) == 1 and recorded[0]["payload"]["type"] == "output_ack"
            assert sealed == [
                {"session_id": "recording", "cutoff_monotonic_ns": cutoff}
            ]
            assert runtime.active_recording is None
        finally:
            consumer.cancel()
            await asyncio.gather(consumer, return_exceptions=True)
            await release_clients(runtime)

    asyncio.run(run())


def test_headset_projection_catalog_and_metadata_are_distinct_from_console(
    tmp_path, monkeypatch
):
    monkeypatch.delenv("HC_PLATFORM_TOKEN", raising=False)

    async def run():
        runtime, app = await make_runtime(tmp_path)
        streams = [
            {"id": name, "enabled": name != "disabled", "metadata": {"source": name}}
            for name in ("head", "disabled", "wrist", "overhead", "fourth", "fifth")
        ]
        runtime.config["camera"] = {"streams": streams}

        class Media:
            async def get(self, *args, **kwargs):
                return httpx.Response(200, json={"streams": streams})

            async def aclose(self):
                pass

        await runtime.media.aclose()
        runtime.media = Media()

        async def status():
            return {"control": {"safety": "stopped"}}

        runtime.status = status
        runtime.registry.compile = lambda robot: {
            "topics": [{"topic": "/package-default"}]
        }
        runtime.topics.items = [{"topic": "/edited-runtime"}]
        try:
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                assert len((await client.get("/api/cameras")).json()["streams"]) == 6
                initial = (await client.get("/api/camera/status")).json()
                assert [item["id"] for item in initial["streams"]] == [
                    "head",
                    "wrist",
                    "overhead",
                    "fourth",
                ]
                assert (
                    await client.put(
                        "/api/cameras/projection", json={"camera_ids": ["disabled"]}
                    )
                ).status_code == 422
                assert (
                    await client.put(
                        "/api/cameras/projection",
                        json={"camera_ids": ["wrist", "head"]},
                    )
                ).status_code == 200
                assert [
                    item["id"]
                    for item in (await client.get("/api/camera/status")).json()[
                        "streams"
                    ]
                ] == ["wrist", "head"]
                streams[2]["enabled"] = False
                fallback = (await client.get("/api/camera/status")).json()
                assert (
                    fallback["projection"]["fallback_reason"]
                    == "SAVED_PROJECTION_HAS_UNAVAILABLE_CAMERAS"
                )
                assert "wrist" not in fallback["projection"]["camera_ids"]
                assert (await client.get("/api/topics/catalog")).json()["topics"] == [
                    {"topic": "/package-default"}
                ]
                assert (await client.get("/api/observations")).json()[
                    "camera_metadata"
                ]["head"] == {"source": "head"}
        finally:
            await release_clients(runtime)

    asyncio.run(run())


def test_gripper_capability_requires_authorized_fresh_hand_feedback(
    tmp_path, monkeypatch
):
    monkeypatch.delenv("HC_PLATFORM_TOKEN", raising=False)

    async def run():
        runtime, app = await make_runtime(tmp_path)
        runtime.mode = "teleop"
        runtime.config.update(
            arms={"right": {}, "left": {}},
            enable_grippers=True,
            enable_base=False,
            allowed_groups=["right_arm"],
            teleop={"body": {}},
            verification={},
            command_interface_version="v1",
            groups={"right_arm": ["j1"], "left_arm": ["j2"]},
            hardware={"allowed_groups": ["right_arm", "right_gripper"]},
            joint_names=["j1", "j2"],
            joint_units=["rad", "rad"],
            lower=[-1, -1],
            upper=[1, 1],
            velocity=[1, 1],
            acceleration=[1, 1],
        )
        backend = {"ready": True, "gripper_ready": {"right": False, "left": True}}

        async def status():
            return {
                "control": {"safety": "ready", "available_groups": ["right_arm"]},
                "backend": backend,
            }

        async def capabilities():
            return {"capabilities": {}}

        runtime.status = status
        runtime.motion.capabilities = capabilities
        try:
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                caps = (await client.get("/api/robot/info")).json()["capabilities"]
                assert caps["arms.teleop"]["ready_groups"] == ["right_arm"]
                assert caps["grippers.open_close"]["runtime_ready"] is False
                assert caps["grippers.open_close"]["blocking_reasons"] == [
                    "GRIPPER_FEEDBACK_UNAVAILABLE"
                ]
                backend["gripper_ready"]["right"] = True
                assert (await client.get("/api/robot/info")).json()["capabilities"][
                    "grippers.open_close"
                ]["runtime_ready"]
        finally:
            await release_clients(runtime)

    asyncio.run(run())


def test_vr_events_drain_during_slow_recording_stop_and_coalesce_toggles(tmp_path):
    async def run():
        runtime, _ = await make_runtime(tmp_path)
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        receiver.setblocking(False)
        sender.setblocking(False)
        runtime.event_socket = receiver
        runtime.active_recording = "rec-1"
        notifications = []
        runtime.notify_vr = lambda kind, payload: notifications.append((kind, payload))
        stop_entered = asyncio.Event()
        allow_stop = asyncio.Event()
        stop_calls = 0

        async def status():
            return {"active_session_id": runtime.active_recording}

        async def stop_child(*args, **kwargs):
            pass

        async def stop_session(session_id, cutoff_monotonic_ns=None):
            nonlocal stop_calls
            stop_calls += 1
            stop_entered.set()
            await allow_stop.wait()
            return {"id": session_id, "state": "READY"}

        runtime.status_data.status = status
        runtime.supervisor.stop = stop_child
        runtime.audit_data.stop_session = stop_session
        runtime.audit_watermark_boot = "test-boot"
        runtime.audit_watermark_ns = time.monotonic_ns() + 10_000_000_000
        consumer = asyncio.create_task(runtime._vr_events())

        async def send_vr(kind):
            await asyncio.get_running_loop().sock_sendall(
                sender,
                json.dumps(
                    {
                        "kind": kind,
                        "sample_mono_ns": time.monotonic_ns(),
                        "boot_id": "test-boot",
                        "epoch": 1,
                    }
                ).encode(),
            )

        try:
            await send_vr("record_toggle")
            await asyncio.wait_for(stop_entered.wait(), 1)
            assert any(
                kind == "recording_status"
                and payload.get("recording") is False
                and payload.get("finalizing") is True
                for kind, payload in notifications
            )
            # Send duplicate record_toggle presses + an unrelated control event while
            # stop_session is still finalizing beyond the 500ms wire expiry window.
            for _ in range(3):
                await send_vr("record_toggle")
            await send_vr("control_pause")
            await asyncio.sleep(0.55)
            allow_stop.set()
            for _ in range(50):
                if any(op == "pause" for op, _ in runtime.core.calls) and any(
                    kind == "recording_stopped" for kind, _ in notifications
                ):
                    break
                await asyncio.sleep(0.01)
            assert stop_calls == 1
            assert runtime.active_recording is None
            assert any(op == "pause" for op, _ in runtime.core.calls)
            assert not any(
                entry["level"] == "ERROR" for entry in runtime.supervisor.logs
            )
            assert not any(kind == "recording_error" for kind, _ in notifications)
            kinds = [kind for kind, _ in notifications]
            assert kinds.index("recording_status") < kinds.index("recording_stopped")
        finally:
            allow_stop.set()
            consumer.cancel()
            await asyncio.gather(consumer, return_exceptions=True)
            receiver.close()
            sender.close()
            runtime.event_socket = None
            await release_clients(runtime)

    asyncio.run(run())


def test_vr_record_toggle_while_starting_does_not_abort_new_recording_or_expire(
    tmp_path,
):
    async def run():
        runtime, _ = await make_runtime(tmp_path)
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        receiver.setblocking(False)
        sender.setblocking(False)
        runtime.event_socket = receiver
        notifications = []
        runtime.notify_vr = lambda kind, payload: notifications.append((kind, payload))
        start_entered = asyncio.Event()
        allow_start = asyncio.Event()
        starts = []
        stops = []

        async def start_impl(spec, request_id, selected):
            start_entered.set()
            await allow_start.wait()
            runtime.active_recording = "rec-2"
            starts.append("rec-2")
            return {"id": "rec-2"}

        async def stop_impl():
            stops.append(runtime.active_recording)
            runtime.active_recording = None
            return {"id": "rec-2"}

        runtime._start_recording_impl = start_impl
        runtime._stop_recording_impl = stop_impl
        consumer = asyncio.create_task(runtime._vr_events())

        async def send_toggle():
            await asyncio.get_running_loop().sock_sendall(
                sender,
                json.dumps(
                    {
                        "kind": "record_toggle",
                        "sample_mono_ns": time.monotonic_ns(),
                        "boot_id": "test-boot",
                        "epoch": 1,
                    }
                ).encode(),
            )

        try:
            await send_toggle()
            await asyncio.wait_for(start_entered.wait(), 1)
            await send_toggle()
            await send_toggle()
            await asyncio.sleep(0.55)
            allow_start.set()
            for _ in range(50):
                if starts:
                    break
                await asyncio.sleep(0.01)
            await asyncio.sleep(0.05)
            assert starts == ["rec-2"]
            assert stops == []
            assert runtime.active_recording == "rec-2"
            assert not any(
                entry["level"] == "ERROR" for entry in runtime.supervisor.logs
            )
            assert not any(kind == "recording_error" for kind, _ in notifications)
        finally:
            allow_start.set()
            consumer.cancel()
            await asyncio.gather(consumer, return_exceptions=True)
            receiver.close()
            sender.close()
            runtime.event_socket = None
            await release_clients(runtime)

    asyncio.run(run())


def test_vr_expired_wire_event_still_rejected(tmp_path):
    async def run():
        runtime, _ = await make_runtime(tmp_path)
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        receiver.setblocking(False)
        sender.setblocking(False)
        runtime.event_socket = receiver
        logged = []
        error_event = asyncio.Event()
        started = []

        def log(subsystem, message, level="INFO"):
            logged.append((subsystem, message, level))
            if level == "ERROR":
                error_event.set()

        async def start(*args, **kwargs):
            started.append(True)

        runtime.supervisor.log = log
        runtime.start_recording = start
        consumer = asyncio.create_task(runtime._vr_events())
        try:
            event = {
                "kind": "record_toggle",
                "sample_mono_ns": time.monotonic_ns() - 600_000_000,
                "boot_id": "test-boot",
                "epoch": 1,
            }
            await asyncio.get_running_loop().sock_sendall(
                sender, json.dumps(event).encode()
            )
            await asyncio.wait_for(error_event.wait(), 1)
            assert started == []
            assert any(
                sub == "vr-events" and "Expired VR management event" in msg and lvl == "ERROR"
                for sub, msg, lvl in logged
            )
        finally:
            consumer.cancel()
            await asyncio.gather(consumer, return_exceptions=True)
            receiver.close()
            sender.close()
            runtime.event_socket = None
            await release_clients(runtime)

    asyncio.run(run())
