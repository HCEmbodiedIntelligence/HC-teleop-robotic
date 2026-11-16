import asyncio
import base64
import json
import os
import struct
import sys
import time
import uuid
import hashlib
import zipfile
from pathlib import Path

import pytest

from hc_platform.data import DataClient, DataError, DataServer, DataService
from hc_platform.data.artifacts import checksum, inspect_mcap, safe_asset
from hc_platform.data.recorder import RecorderProcess
from hc_platform.data.sample_codec import iter_samples


def run(coroutine):
    return asyncio.run(coroutine)


async def recorded(service, request="record"):
    session = await service.start_session(
        {"robot_id": "x1", "config_hash": "test"}, request
    )
    await service.append_sample(
        session["id"], "/state", {"name": ["joint_b", "joint_a"], "position": [2, 1]}
    )
    result = await service.stop_session(session["id"])
    return await service.get_dataset(result["artifact_id"])


def test_actual_mcap_cdr_and_idempotent_session(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            spec = {"robot_id": "x1"}
            session = await service.start_session(spec, "same")
            assert (await service.start_session(spec, "same"))["id"] == session["id"]
            with pytest.raises(DataError, match="IDEMPOTENCY_CONFLICT"):
                await service.start_session({"robot_id": "other"}, "same")
            text = b"hello\0"
            cdr = b"\0\1\0\0" + struct.pack("<I", len(text)) + text
            await service.append_sample(
                session["id"],
                "/raw",
                {
                    "__cdr_b64": base64.b64encode(cdr).decode(),
                    "__ros_type": "std_msgs/msg/String",
                    "__ros_schema": "string data\n",
                },
            )
            await service.append_sample(session["id"], "/json", {"value": 42})
            result = await service.stop_session(session["id"])
            artifact = await service.get_dataset(result["artifact_id"])
            assert (
                inspect_mcap(Path(artifact["root"]) / "capture.mcap")["message_count"]
                == 3
            )
            sample = [
                s
                for s in iter_samples(Path(artifact["root"]) / "capture.mcap")
                if s["topic"] == "/json"
            ][0]
            assert sample["payload"] == {"value": 42}
            assert artifact["manifest"]["task_success"] is None
        finally:
            await service.close()

    run(scenario())


def test_single_sqlite_writer_and_private_rpc(tmp_path):
    async def scenario():
        service = await DataService(tmp_path / "state").open()
        second = DataService(tmp_path / "state")
        server = await DataServer(service, tmp_path / "data.sock").start()
        client = DataClient(tmp_path / "data.sock")
        try:
            with pytest.raises(DataError, match="Another data service"):
                await second.open()
            assert (tmp_path / "data.sock").stat().st_mode & 0o777 == 0o600
            assert (await client.call("status"))["service"] == "jobs-data"
            assert await client.list_datasets() == []
            with pytest.raises(DataError):
                await client.call("close")
        finally:
            await client.close()
            await server.close()
            await second.close()
            await service.close()

    run(scenario())


def test_rpc_timeout_closes_connection_before_next_request(tmp_path):
    async def scenario():
        service = await DataService(tmp_path / "state").open()
        original_status, calls = service.status, 0

        async def slow_once():
            nonlocal calls
            calls += 1
            if calls == 1:
                await asyncio.sleep(0.05)
            return await original_status()

        service.status = slow_once
        server = await DataServer(service, tmp_path / "data.sock").start()
        client = DataClient(tmp_path / "data.sock", timeout=0.01)
        try:
            with pytest.raises(DataError) as error:
                await client.status()
            assert (
                error.value.code == "DATA_SERVICE_UNAVAILABLE" and client.writer is None
            )
            client.timeout = 1
            assert (await client.status())["state"] == "ready"
            await asyncio.sleep(0.06)
        finally:
            await client.close()
            await server.close()
            await service.close()

    run(scenario())


def test_recording_queue_overflow_is_partial_not_ready(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session({"queue_bytes": 1024}, "overflow")
            with pytest.raises(DataError) as error:
                await service.append_sample(
                    session["id"], "/large", {"text": "x" * 8192}
                )
            assert error.value.code == "RECORDING_QUEUE_OVERFLOW"
            with pytest.raises(DataError):
                await service.stop_session(session["id"])
            assert (await service.get_session(session["id"]))["state"] == "PARTIAL"
            assert await service.list_datasets() == []
            assert list((tmp_path / "staging").glob("*.partial/capture.mcap"))
        finally:
            await service.close()

    run(scenario())


def test_disk_full_writer_is_detected_without_ready_artifact():
    if not Path("/dev/full").exists():
        pytest.skip("Linux fault device required")

    async def scenario():
        recorder = RecorderProcess("/dev/full")
        try:
            await recorder.ready()
            recorder.append("/test", b"\0\1\0\0\1\0\0\0\0", 1, 1)
            with pytest.raises(DataError):
                await recorder.finish()
        finally:
            await recorder.abort()

    run(scenario())


def test_dagger_confirmed_boundaries_gate_capture(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session(
                {"mode": "dagger_corrections", "auto_record": True}, "dagger"
            )
            identity = session["id"]
            assert not (
                await service.append_sample(identity, "/state", {"value": "automatic"})
            )["accepted"]
            with pytest.raises(DataError):
                await service.set_intervention(identity, True, {"event_id": "request"})
            origin = time.monotonic_ns() - 5_000_000
            await service.set_intervention(
                identity,
                True,
                {
                    "event_id": "begin",
                    "confirmed": True,
                    "control_epoch": 5,
                    "monotonic_ns": origin,
                },
            )
            await service.append_sample(
                identity,
                "/state",
                {"value": "human"},
                sample_monotonic_ns=origin + 1_000_000,
            )
            await service.set_intervention(
                identity,
                False,
                {
                    "event_id": "end",
                    "confirmed": True,
                    "control_epoch": 6,
                    "monotonic_ns": origin + 2_000_000,
                },
            )
            assert not (
                await service.append_sample(
                    identity, "/state", {"value": "automatic_again"}
                )
            )["accepted"]
            result = await service.stop_session(identity)
            artifact = await service.get_dataset(result["artifact_id"])
            samples = list(iter_samples(Path(artifact["root"]) / "capture.mcap"))
            assert [
                s["payload"]["value"] for s in samples if s["topic"] == "/state"
            ] == ["human"]
            assert (
                len([s for s in samples if s["topic"] == "/hc/control/intervention"])
                == 2
            )
        finally:
            await service.close()

    run(scenario())


def test_dagger_delayed_and_reordered_boundaries_use_sample_clock(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session(
                {
                    "mode": "dagger_corrections",
                    "auto_record": True,
                    "reorder_window_ms": 100,
                },
                "reordered",
            )
            identity = session["id"]
            origin = time.monotonic_ns() - 10_000_000
            # Both transitions are delayed relative to capture, and end arrives
            # before start on the independent audit connection.
            await service.append_sample(
                identity,
                "/state",
                {
                    "value": "human_before_start_event",
                    "received_monotonic_ns": origin + 3_000_000,
                },
            )
            await service.append_sample(
                identity,
                "/state",
                {
                    "value": "automatic_after_end",
                    "received_monotonic_ns": origin + 8_000_000,
                },
            )
            await service.set_intervention(
                identity,
                False,
                {
                    "event_id": "end",
                    "confirmed": True,
                    "control_epoch": 6,
                    "monotonic_ns": origin + 7_000_000,
                },
            )
            await service.set_intervention(
                identity,
                True,
                {
                    "event_id": "start",
                    "confirmed": True,
                    "control_epoch": 5,
                    "monotonic_ns": origin + 2_000_000,
                },
            )
            # Old queued program data arriving while the human interval is
            # known is excluded by time and independently by command epoch.
            await service.append_sample(
                identity,
                "/state",
                {
                    "value": "late_old_automatic",
                    "received_monotonic_ns": origin + 1_000_000,
                },
            )
            await service.append_sample(
                identity,
                "/platform/control/commands",
                {
                    "epoch": 4,
                    "issued_mono_ns": origin + 4_000_000,
                    "value": "old_program_epoch",
                },
            )
            await service.append_sample(
                identity,
                "/platform/control/commands",
                {
                    "epoch": 5,
                    "issued_mono_ns": origin + 5_000_000,
                    "value": "human_command",
                },
            )
            assert not (
                await service.append_sample(
                    identity, "/unknown", {"value": "untrusted_clock"}
                )
            )["accepted"]
            result = await service.stop_session(identity)
            artifact = await service.get_dataset(result["artifact_id"])
            samples = list(iter_samples(Path(artifact["root"]) / "capture.mcap"))
            assert [
                sample["payload"]["value"]
                for sample in samples
                if "value" in sample["payload"]
            ] == ["human_before_start_event", "human_command"]
            sidecar = [
                json.loads(line)
                for line in (Path(artifact["root"]) / "capture.samples.jsonl")
                .read_text()
                .splitlines()
            ]
            assert len(sidecar) == 2 and all(
                row["human_intervention"] for row in sidecar
            )
            assert sidecar[-1]["control_epoch"] == 5
            stats = artifact["manifest"]["stats"]["reorder"]
            assert stats["selected"] == 2 and stats["excluded"] == 4
            assert stats["exclusion_reasons"]["command_epoch_outside_intervention"] == 1
            intervals = json.loads(
                (Path(artifact["root"]) / "interventions.json").read_text()
            )["intervals"]
            assert (
                len(intervals) == 1
                and intervals[0]["complete"]
                and intervals[0]["start_epoch"] == 5
            )
        finally:
            await service.close()

    run(scenario())


def test_dagger_reorder_overflow_never_publishes_ready(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session(
                {
                    "mode": "dagger_corrections",
                    "auto_record": True,
                    "queue_messages": 1,
                    "reorder_window_ms": 100,
                },
                "window-overflow",
            )
            await service.append_sample(
                session["id"],
                "/state",
                {"q": [1]},
                sample_monotonic_ns=time.monotonic_ns(),
            )
            with pytest.raises(DataError) as error:
                await service.append_sample(
                    session["id"],
                    "/state",
                    {"q": [2]},
                    sample_monotonic_ns=time.monotonic_ns(),
                )
            assert error.value.code == "DAGGER_REORDER_OVERFLOW"
            with pytest.raises(DataError):
                await service.stop_session(session["id"])
            assert await service.list_datasets() == []
        finally:
            await service.close()

    run(scenario())


def test_dagger_late_transition_fails_closed_and_stop_drains_audit(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session(
                {
                    "mode": "dagger_corrections",
                    "auto_record": True,
                    "reorder_window_ms": 10,
                },
                "too-late",
            )
            stamp = time.monotonic_ns()
            await service.append_sample(
                session["id"],
                "/state",
                {"value": "assumed_automatic"},
                sample_monotonic_ns=stamp,
            )
            await asyncio.sleep(0.06)
            with pytest.raises(DataError) as error:
                await service.set_intervention(
                    session["id"],
                    True,
                    {
                        "event_id": "late",
                        "confirmed": True,
                        "control_epoch": 5,
                        "monotonic_ns": stamp - 1,
                    },
                )
            assert error.value.code == "DAGGER_TRANSITION_TOO_LATE"
            with pytest.raises(DataError):
                await service.stop_session(session["id"])
            assert await service.list_datasets() == []
            session = await service.start_session(
                {
                    "mode": "dagger_corrections",
                    "auto_record": True,
                    "reorder_window_ms": 100,
                },
                "tail-drain",
            )
            stamp = time.monotonic_ns() - 2_000_000
            await service.append_sample(
                session["id"],
                "/state",
                {"value": "tail_human"},
                sample_monotonic_ns=stamp + 1_000_000,
            )
            stopping = asyncio.create_task(service.stop_session(session["id"]))
            await asyncio.sleep(0.01)
            await service.set_intervention(
                session["id"],
                True,
                {
                    "event_id": "tail",
                    "confirmed": True,
                    "control_epoch": 7,
                    "monotonic_ns": stamp,
                },
            )
            result = await stopping
            artifact = await service.get_dataset(result["artifact_id"])
            assert artifact["manifest"]["incomplete_intervention"]["event_id"] == "tail"
            samples = list(iter_samples(Path(artifact["root"]) / "capture.mcap"))
            assert [
                s["payload"]["value"] for s in samples if s["topic"] == "/state"
            ] == ["tail_human"]
        finally:
            await service.close()

    run(scenario())


def test_download_reference_prevents_delete_and_path_escape(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            artifact = await recorded(service)
            identity = artifact["id"]
            await service.acquire_artifact(identity, "preview")
            with pytest.raises(DataError, match="ARTIFACT_IN_USE"):
                await service.delete_dataset(identity)
            with pytest.raises(DataError):
                await service.resolve_asset(identity, "../metadata.sqlite3", "preview")
            assert (await service.resolve_asset(identity, "capture.mcap", "preview"))[
                "path"
            ].endswith("capture.mcap")
            await service.release_artifact(identity, "preview")
            download = await service.prepare_download(identity)
            assert Path(download["path"]).stat().st_size > 0
            await service.release_artifact(identity, download["owner"])
            await service.delete_dataset(identity)
            assert await service.list_datasets() == []
        finally:
            await service.close()

    run(scenario())


def test_reader_lease_survives_data_service_restart(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        artifact = await recorded(service)
        await service.acquire_artifact(artifact["id"], "independent-replay", 300)
        await service.close()
        service = await DataService(tmp_path).open()
        try:
            with pytest.raises(DataError, match="ARTIFACT_IN_USE"):
                await service.delete_dataset(artifact["id"])
            await service.release_artifact(artifact["id"], "independent-replay")
            assert (await service.delete_dataset(artifact["id"]))["deleted"]
        finally:
            await service.close()

    run(scenario())


def test_manifest_failure_keeps_source(tmp_path, monkeypatch):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session({}, "seal-failure")
            await service.append_sample(session["id"], "/state", {"q": [1]})

            def fail(*args):
                raise OSError("disk full")

            monkeypatch.setattr("hc_platform.data.service.seal_directory", fail)
            with pytest.raises(DataError):
                await service.stop_session(session["id"])
            assert await service.list_datasets() == []
            assert list((tmp_path / "staging").glob("*.partial/capture.mcap"))
        finally:
            await service.close()

    run(scenario())


def test_persistent_job_recovery_idempotency_and_cancel(tmp_path):
    async def scenario():
        service = await DataService(
            tmp_path,
            converter_command=[sys.executable, "-c", "import time;time.sleep(60)"],
        ).open()
        try:
            artifact = await recorded(service)
            job = await service.enqueue_conversion(
                artifact["id"], "convert", {"fps": 30}
            )
            assert (
                await service.enqueue_conversion(artifact["id"], "convert", {"fps": 30})
            )["id"] == job["id"]
            with pytest.raises(DataError, match="IDEMPOTENCY_CONFLICT"):
                await service.enqueue_conversion(artifact["id"], "convert", {"fps": 20})
            for _ in range(100):
                if service.processes:
                    break
                await asyncio.sleep(0.02)
            assert service.processes
        finally:
            await service.close()
        recovered = await DataService(tmp_path).open()
        try:
            assert (await recovered.get_job(job["id"]))["result"] == {}
            await recovered.cancel_job(job["id"])
            for _ in range(100):
                state = (await recovered.get_job(job["id"]))["state"]
                if state == "CANCELED":
                    break
                await asyncio.sleep(0.02)
            assert state == "CANCELED"
            assert (await recovered.get_dataset(artifact["id"]))["references"] == []
        finally:
            await recovered.close()

    run(scenario())


def test_cdr_invalid_and_symlink_rejected(tmp_path):
    (tmp_path / "link").symlink_to("/etc/passwd")
    with pytest.raises(DataError):
        safe_asset(tmp_path, "link")
    assert checksum(Path(__file__))["size_bytes"] > 0


@pytest.mark.parametrize(
    "local,upload,local_kind,upload_kind",
    [
        ("mcap", "lerobot", None, "conversion"),
        ("lerobot", "mcap", "conversion", None),
        ("mcap_mp4", "mcap", "video", None),
    ],
)
def test_local_and_upload_formats_route_independently(
    tmp_path, local, upload, local_kind, upload_kind
):
    async def scenario():
        service = await DataService(
            tmp_path,
            converter_command=[
                sys.executable,
                "-m",
                "hc_platform.data.converter_worker",
            ],
            platform_config={
                "platform_base_url": "http://127.0.0.1:9",
                "credential_env": "TEST_NO_NETWORK",
                "robot_id": "platform-x1-uuid",
            },
        ).open()
        service.runner.cancel()
        await asyncio.gather(service.runner, return_exceptions=True)
        try:
            identity = str(uuid.uuid4())
            spec = {
                "recording_id": identity,
                "local_format": local,
                "upload_format": upload,
                "auto_upload": True,
                "capture_snapshot": {"recording_id": identity},
                "upload_target": {
                    "robot_id": "platform-x1-uuid",
                    "collection_task_id": "task-a",
                    "capture_plan": {
                        "recording_id": identity,
                        "plan_revision": 1,
                        "plan_sha256": "a" * 64,
                        "snapshot_sha256": "b" * 64,
                    },
                },
            }
            session = await service.start_session(spec, "freeze-format")
            assert session["id"] == identity
            # A later connection change must not redirect this in-flight capture.
            await service.configure_platform(
                {
                    "platform_base_url": "http://127.0.0.1:10",
                    "credential_env": "OTHER_TOKEN",
                    "robot_id": "other-robot",
                }
            )
            assert (await service.start_session(spec, "freeze-format"))[
                "id"
            ] == identity
            await service.append_sample(identity, "/state", {"value": 1})
            result = await service.stop_session(identity)
            jobs = await service.list_jobs()
            derivations = [j for j in jobs if j["kind"] in {"conversion", "video"}]
            assert len(derivations) == 1
            route = derivations[0]["params"]["route"]
            assert route["required_for_local"] == (local_kind is not None)
            assert route["upload_on_success"] == (upload_kind is not None)
            uploads = [j for j in jobs if j["kind"] == "upload"]
            assert bool(uploads) == (upload == "mcap")
            binding = (
                uploads[0]["params"]["platform_config"]
                if uploads
                else route["platform_config"]
            )
            assert binding["platform_base_url"].endswith(":9")
            assert result["state"] == ("READY" if local == "mcap" else "POSTPROCESSING")
        finally:
            await service.close()

    run(scenario())


def test_recording_id_is_stable_and_snapshot_mismatch_rejected(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            with pytest.raises(DataError) as error:
                await service.start_session(
                    {
                        "recording_id": str(uuid.uuid4()),
                        "capture_snapshot": {"recording_id": str(uuid.uuid4())},
                    },
                    "bad-snapshot",
                )
            assert error.value.code == "CAPTURE_RECORDING_ID_MISMATCH"
            session = await service.start_session({}, "stable")
            assert session["id"] == str(
                uuid.uuid5(uuid.NAMESPACE_URL, "hc-recording:stable")
            )
            await service.stop_session(session["id"])
        finally:
            await service.close()

    run(scenario())


def test_filename_save_directory_are_real_and_recoverable(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            for invalid in (
                "../escape.mcap",
                "/tmp/escape.mcap",
                "hidden/file",
                "bad.mp4",
            ):
                with pytest.raises(DataError):
                    await service.start_session({"filename": invalid}, "bad-" + invalid)
            with pytest.raises(DataError):
                await service.start_session(
                    {"selected_save_dir": "/tmp/not-the-data-root"}, "bad-directory"
                )
            session = await service.start_session(
                {"filename": "cup_demo", "selected_save_dir": "cup/day1"}, "named"
            )
            await service.append_sample(session["id"], "/state", {"q": [1]})
            result = await service.stop_session(session["id"])
            artifact = await service.get_dataset(result["artifact_id"])
            assert (
                Path(artifact["root"])
                == tmp_path / "artifacts/cup/day1" / session["id"]
            )
            assert (Path(artifact["root"]) / "cup_demo.mcap").exists()
            assert not (Path(artifact["root"]) / "capture.mcap").exists()
            # Model a crash between directory rename and catalog registration.
            await service.store.run(
                lambda db: db.execute(
                    "DELETE FROM artifacts WHERE id=?", (artifact["id"],)
                )
            )
        finally:
            await service.close()
        recovered = await DataService(tmp_path).open()
        try:
            assert (await recovered.get_dataset(artifact["id"]))["state"] == "READY"
        finally:
            await recovered.close()

    run(scenario())


def test_robot_package_is_frozen_portable_and_hash_checked(tmp_path):
    async def scenario():
        source = tmp_path / "source.zip"
        with zipfile.ZipFile(source, "w") as archive:
            archive.writestr("robot.urdf", "<robot name='x1'/>")
            archive.writestr("meshes/part.stl", "test mesh")
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        service = await DataService(tmp_path / "data").open()
        try:
            with pytest.raises(DataError) as error:
                await service.start_session(
                    {
                        "robot_package_archive": str(source),
                        "robot_package_archive_sha256": "0" * 64,
                    },
                    "bad-package",
                )
            assert error.value.code == "ROBOT_PACKAGE_HASH_MISMATCH"
            assert not list((tmp_path / "data/staging").iterdir())
            session = await service.start_session(
                {
                    "robot_package_archive": str(source),
                    "robot_package_archive_sha256": digest,
                    "robot_config": {"package_hash": "package-tree-hash"},
                },
                "frozen-package",
            )
            source.unlink()
            await service.append_sample(session["id"], "/state", {"q": [1]})
            for _ in range(100):
                status = await service.status()
                if status["statistics"]["written"] >= 2:
                    break
                await asyncio.sleep(0.01)
            assert (
                status["statistics"]["accepted"] == 2
                and status["statistics"]["written"] == 2
            )
            assert (
                status["statistics"]["dropped"] == 0
                and status["statistics"]["errors"] == 0
            )
            result = await service.stop_session(session["id"])
            artifact = await service.get_dataset(result["artifact_id"])
            frozen = artifact["manifest"]["session"]["robot_package"]
            assert (
                frozen["package_hash"] == "package-tree-hash"
                and frozen["sha256"] == digest
            )
            asset = next(
                item
                for item in artifact["manifest"]["assets"]
                if item["path"] == "robot_package.zip"
            )
            assert asset["sha256"] == digest
            with zipfile.ZipFile(Path(artifact["root"]) / frozen["path"]) as archive:
                assert archive.read("meshes/part.stl") == b"test mesh"
            item = (await service.list_datasets())[0]
            assert "manifest" not in item and item["summary"]["message_count"] == 2
        finally:
            await service.close()

    run(scenario())
