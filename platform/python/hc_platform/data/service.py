"""Data-plane domain service; no control callbacks or legacy modules live here."""

import asyncio
import base64
import hashlib
import importlib.util
import json
import os
import random
import shutil
import signal
import struct
import sys
import tarfile
import time
import uuid
from pathlib import Path
from urllib.parse import parse_qs, urlsplit, urlunsplit

from .artifacts import (
    atomic_json,
    canonical,
    freeze_robot_package,
    inspect_mcap,
    safe_asset,
    seal_directory,
    sync_directory,
    validate_lerobot,
    verify_directory,
)
from .errors import DataError
from .dagger import DaggerWindow
from .recorder import RecorderProcess, available as mcap_available
from .store import MetadataStore
from .upload import PlatformAdapter, validate_config


def _row(row):
    if row is None:
        raise DataError("NOT_FOUND")
    value = dict(row)
    for key in ("spec", "manifest", "params", "result", "stats", "error"):
        if key in value and value[key] is not None:
            value[key] = json.loads(value[key])
    return value


class DataService:
    """One instance/process owns a state directory; API clients use Unix RPC.

    All public methods are async. The control process must never await them in
    its servo loop; use the independently bounded capture bridge instead.
    """

    def __init__(
        self, state_dir, *, converter_command=None, platform_config=None, max_uploads=2
    ):
        self.root = Path(state_dir).resolve()
        self.store = MetadataStore(self.root)
        converter_python = os.environ.get("HC_CONVERTER_PYTHON")
        self.converter_command = tuple(
            converter_command
            or (
                converter_python or sys.executable,
                "-m",
                "hc_platform.data.converter_worker",
            )
        )
        self.external_converter = converter_command is not None or bool(
            converter_python
        )
        self.platform_config = (
            validate_config(platform_config) if platform_config else None
        )
        self.max_uploads = max(1, min(4, int(max_uploads)))
        self.recorder = None
        self.active_session = None
        self.session_lock = asyncio.Lock()
        self.session_stopping = False
        self.recording_gate = False
        self.intervention = None
        self.dagger = self.dagger_task = self.finalize_task = None
        self.drain_until_ns = self.stop_requested_ns = 0
        self.jobs = {}
        self.processes = {}
        self.runner = None
        self.closing = False

    async def open(self):
        await self.store.open()
        for directory in ("artifacts", "staging", "downloads"):
            (self.root / directory).mkdir(exist_ok=True)
        if self.platform_config is None:
            value = await self.store.run(
                lambda db: db.execute(
                    "SELECT value FROM settings WHERE key='platform'"
                ).fetchone()
            )
            if value:
                self.platform_config = validate_config(json.loads(value[0]))
        await self._recover()
        self.runner = asyncio.create_task(self._job_runner(), name="data-job-runner")
        return self

    async def close(self):
        if self.closing:
            return
        self.closing = True
        if self.recorder is not None and self.active_session:
            try:
                await self.stop_session(self.active_session["id"])
            except Exception:
                if self.recorder:
                    await self.recorder.abort()
        if self.runner:
            self.runner.cancel()
            await asyncio.gather(self.runner, return_exceptions=True)
        for task in self.jobs.values():
            task.cancel()
        if self.jobs:
            await asyncio.gather(*list(self.jobs.values()), return_exceptions=True)
        await self.store.close()

    async def _recover(self):
        def repair(db):
            db.execute(
                "UPDATE sessions SET state='INTERRUPTED',error=? WHERE state IN ('PREPARING','ARMED','RECORDING','STOPPING','FINALIZING')",
                (
                    canonical(
                        {
                            "code": "PROCESS_RESTARTED",
                            "message": "Partial recording retained",
                        }
                    ),
                ),
            )
            db.execute(
                "UPDATE jobs SET state=CASE WHEN cancel_requested=1 THEN 'CANCELED' ELSE 'QUEUED' END,updated=? WHERE state IN ('RUNNING','CANCEL_REQUESTED')",
                (time.time(),),
            )
            # API downloads and the replay worker may outlive jobs-data. Keep
            # their unexpired leases across its restart; dead readers expire.
            db.execute(
                "DELETE FROM artifact_refs WHERE (kind!='job' AND expires<=?) OR owner IN (SELECT 'job:'||id FROM jobs WHERE state IN ('SUCCEEDED','FAILED','CANCELED'))",
                (time.time(),),
            )

        await self.store.run(repair)
        # A published manifest is the durable filesystem fact if the process
        # died between rename and the SQLite commit.
        for path in sorted((self.root / "artifacts").rglob("manifest.json")):
            try:
                manifest = json.loads(await asyncio.to_thread(path.read_text))
                if (
                    manifest.get("schema_version") != "hc-artifact/v1"
                    or manifest.get("artifact_id") != path.parent.name
                ):
                    continue
                await asyncio.to_thread(verify_directory, path.parent, manifest)
                await self._register_artifact(path.parent, manifest)
                if manifest.get("session_id") == manifest["artifact_id"]:
                    await self.store.run(
                        lambda db, m=manifest, p=str(path.parent): db.execute(
                            "UPDATE sessions SET state='READY',root=?,error=NULL WHERE id=? AND state='INTERRUPTED'",
                            (p, m["session_id"]),
                        )
                    )
            except (OSError, ValueError, KeyError, DataError):
                # Never present an unverifiable recovered artifact as ready.
                await self.store.run(
                    lambda db, p=str(path.parent): db.execute(
                        "UPDATE artifacts SET state='CORRUPT' WHERE root=?", (p,)
                    )
                )
        deleting = await self.store.run(
            lambda db: [
                dict(r)
                for r in db.execute(
                    "SELECT id,root FROM artifacts WHERE state='DELETING'"
                )
            ]
        )
        for entry in deleting:
            await asyncio.to_thread(shutil.rmtree, entry["root"], True)
            await self.store.run(
                lambda db, identity=entry["id"]: db.execute(
                    "UPDATE artifacts SET state='DELETED' WHERE id=?", (identity,)
                )
            )
        # Complete the durable outbox implied by a ready raw recording. A crash
        # after publishing bytes but before creating its jobs must not lose the
        # selected local/upload format plan.
        for artifact in await self.list_datasets(limit=100000, include_manifest=True):
            if (
                artifact["state"] == "READY"
                and artifact["format"] == "mcap"
                and artifact.get("session_id") == artifact["id"]
            ):
                try:
                    await self._route_session_outputs(artifact)
                except DataError as error:
                    await self.store.run(
                        lambda db, a=artifact, e=error: db.execute(
                            "UPDATE sessions SET error=? WHERE id=?",
                            (canonical(e.as_dict()), a["session_id"]),
                        )
                    )

    async def status(self):
        statistics = self.recorder.statistics() if self.recorder else None
        if statistics is not None and self.dagger:
            statistics["reorder"] = self.dagger.statistics()
        if (
            statistics
            and statistics["last_error"]
            and self.active_session
            and self.active_session["state"] != "PARTIAL"
        ):
            self.active_session["state"] = "PARTIAL"
            failed_session_id = self.active_session["id"]
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE sessions SET state='PARTIAL',error=? WHERE id=?",
                    (canonical({"code": statistics["last_error"]}), failed_session_id),
                )
            )
        counts = await self.store.run(
            lambda db: {
                row[0]: row[1]
                for row in db.execute("SELECT state,count(*) FROM jobs GROUP BY state")
            }
        )
        return {
            "service": "jobs-data",
            "state": "ready",
            "active_session_id": self.active_session["id"]
            if self.active_session
            else None,
            "active_session_state": self.active_session["state"]
            if self.active_session
            else None,
            "recorder_pid": self.recorder.process.pid if self.recorder else None,
            "recorder_alive": bool(self.recorder and self.recorder.process.is_alive()),
            "recording_root": str(self.root / "artifacts"),
            "statistics": statistics,
            "capabilities": {
                "mcap": mcap_available(),
                "lerobot": self.external_converter
                or importlib.util.find_spec("lerobot") is not None,
                "platform": self.platform_config is not None,
                "mcap_mp4": shutil.which("ffmpeg") is not None,
            },
            "jobs": counts,
        }

    async def start_session(self, spec, request_id):
        if not request_id or len(request_id) > 256 or not isinstance(spec, dict):
            raise DataError("SESSION_REQUEST_INVALID")
        # Round trip validates finite JSON and freezes the caller's mutable data.
        spec = json.loads(canonical(spec))
        spec["filename"] = self._filename(spec.get("filename", "capture.mcap"))
        spec["save_subdirectory"] = self._save_subdirectory(
            spec.get("save_subdirectory", spec.get("selected_save_dir", ""))
        )
        local_format = spec.get("local_format", spec.get("format", "mcap"))
        if local_format not in {"mcap", "lerobot", "mcap_lerobot", "mcap_mp4"}:
            raise DataError(
                "FORMAT_UNAVAILABLE",
                "Supported local formats: mcap, mcap_mp4, lerobot, mcap_lerobot",
            )
        mode = spec.get("mode", "ordinary")
        if mode not in {"ordinary", "dagger_corrections", "dagger_full"}:
            raise DataError("RECORDING_MODE_INVALID")
        if mode.startswith("dagger") and spec.get("auto_record") is not True:
            raise DataError("DAGGER_AUTO_RECORD_DISABLED")
        if (
            mode.startswith("dagger")
            and not 10 <= int(spec.get("reorder_window_ms", 500)) <= 10000
        ):
            raise DataError("DAGGER_REORDER_WINDOW_INVALID")
        if (
            local_format in {"lerobot", "mcap_lerobot"}
            and not self.external_converter
            and importlib.util.find_spec("lerobot") is None
        ):
            raise DataError(
                "LEROBOT_UNAVAILABLE",
                "Install the pinned LeRobot worker environment before recording this format",
            )
        if local_format == "mcap_mp4" and shutil.which("ffmpeg") is None:
            raise DataError(
                "VIDEO_ENCODER_UNAVAILABLE",
                "Install ffmpeg before recording this format",
            )
        if (
            not 1 <= int(spec.get("queue_messages", 4096)) <= 65536
            or not 1024
            <= int(spec.get("queue_bytes", 64 * 1024 * 1024))
            <= 512 * 1024 * 1024
        ):
            raise DataError("RECORDING_QUEUE_LIMIT_INVALID")
        if not mcap_available():
            raise DataError("MCAP_UNAVAILABLE")
        async with self.session_lock:
            previous = await self.store.run(
                lambda db: db.execute(
                    "SELECT * FROM sessions WHERE request_id=?", (request_id,)
                ).fetchone()
            )
            if previous:
                value = _row(previous)
                if {
                    k: v
                    for k, v in value["spec"].items()
                    if k != "_frozen_upload_config"
                } != spec:
                    raise DataError("IDEMPOTENCY_CONFLICT")
                return value
            if self.active_session:
                raise DataError(
                    "RECORDING_BUSY",
                    "Ordinary and DAgger capture share one recording lease",
                )
            try:
                identity = (
                    str(uuid.UUID(spec["recording_id"]))
                    if spec.get("recording_id")
                    else str(
                        uuid.uuid5(uuid.NAMESPACE_URL, "hc-recording:" + request_id)
                    )
                )
            except (ValueError, TypeError, AttributeError):
                raise DataError(
                    "RECORDING_ID_INVALID", "recording_id must be a UUID"
                ) from None
            if await self.store.run(
                lambda db: db.execute(
                    "SELECT 1 FROM sessions WHERE id=?", (identity,)
                ).fetchone()
            ):
                raise DataError("RECORDING_ID_CONFLICT")
            upload_format = spec.get("upload_format", "mcap")
            if upload_format not in {"mcap", "lerobot", "mcap_mp4"}:
                raise DataError("UPLOAD_FORMAT_INVALID")
            if spec.get("auto_upload") is True:
                if self.platform_config is None:
                    raise DataError("PLATFORM_NOT_CONFIGURED")
                target = spec.get("upload_target", {})
                if not target.get("robot_id") or not target.get("collection_task_id"):
                    raise DataError("UPLOAD_TARGET_REQUIRED")
                if (
                    self.platform_config.get("robot_id")
                    and target["robot_id"] != self.platform_config["robot_id"]
                ):
                    raise DataError("PLATFORM_ROBOT_MISMATCH")
                spec["_frozen_upload_config"] = json.loads(
                    canonical(self.platform_config)
                )
            snapshot = spec.get("capture_snapshot")
            if snapshot is not None and snapshot.get("recording_id") != identity:
                raise DataError("CAPTURE_RECORDING_ID_MISMATCH")
            started = time.time_ns()
            staging = self.root / "staging" / (identity + ".partial")
            staging.mkdir()
            session = {
                "id": identity,
                "request_id": request_id,
                "spec": spec,
                "state": "PREPARING",
                "root": str(staging),
                "started_ns": started,
                "clock": {
                    "monotonic_origin_ns": time.monotonic_ns(),
                    "wall_origin_ns": started,
                    "scope": "same_host_monotonic",
                },
            }
            try:
                if spec.get("robot_package_archive"):
                    session["robot_package"] = await asyncio.to_thread(
                        freeze_robot_package,
                        spec["robot_package_archive"],
                        staging / "robot_package.zip",
                        spec.get("robot_package_archive_sha256"),
                        spec.get(
                            "package_hash",
                            spec.get("robot_config", {}).get("package_hash"),
                        ),
                    )
                await asyncio.to_thread(atomic_json, staging / "session.json", session)
            except BaseException:
                await asyncio.to_thread(shutil.rmtree, staging, True)
                raise
            await self.store.run(
                lambda db: db.execute(
                    "INSERT INTO sessions(id,request_id,state,spec,root,started_ns) VALUES(?,?,?,?,?,?)",
                    (
                        identity,
                        request_id,
                        "PREPARING",
                        canonical(spec),
                        str(staging),
                        started,
                    ),
                )
            )
            try:
                recorder = RecorderProcess(
                    staging / spec["filename"],
                    max_messages=int(spec.get("queue_messages", 4096)),
                    max_bytes=int(spec.get("queue_bytes", 64 * 1024 * 1024)),
                )
                await recorder.ready()
            except Exception as error:
                await self.store.run(
                    lambda db, error=error: db.execute(
                        "UPDATE sessions SET state='FAILED',error=? WHERE id=?",
                        (
                            canonical(
                                {
                                    "code": getattr(
                                        error, "code", "RECORDER_START_FAILED"
                                    )
                                }
                            ),
                            identity,
                        ),
                    )
                )
                raise
            self.recorder, self.active_session = recorder, session
            self.finalize_task = None
            self.drain_until_ns = self.stop_requested_ns = 0
            if mode.startswith("dagger"):
                self.dagger = DaggerWindow(
                    session["clock"]["monotonic_origin_ns"],
                    delay_ns=int(spec.get("reorder_window_ms", 500)) * 1_000_000,
                    max_bytes=min(
                        64 * 1024 * 1024, int(spec.get("queue_bytes", 64 * 1024 * 1024))
                    ),
                    max_messages=int(spec.get("queue_messages", 4096)),
                    corrections=mode == "dagger_corrections",
                )
                self.dagger_task = asyncio.create_task(
                    self._dagger_flusher(), name="dagger-reorder-window"
                )
            self.recording_gate = mode != "dagger_corrections"
            self.intervention = None
            self.session_stopping = False
            session["state"] = "RECORDING" if self.recording_gate else "ARMED"
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE sessions SET state=? WHERE id=?",
                    (session["state"], identity),
                )
            )
            await self._append(
                identity,
                "/hc/recording/session",
                {"spec": spec, "session_id": identity},
                None,
                force=True,
            )
            return await self.get_session(identity)

    async def append_sample(
        self, session_id, topic, payload, source_time_ns=None, sample_monotonic_ns=None
    ):
        return await self._append(
            session_id,
            topic,
            payload,
            source_time_ns,
            sample_monotonic_ns=sample_monotonic_ns,
        )

    async def append_samples(self, session_id, samples):
        last = None
        for sample in samples:
            last = await self._append(
                session_id,
                sample["topic"],
                sample["payload"],
                sample.get("source_time_ns"),
                sample_monotonic_ns=sample.get("sample_monotonic_ns"),
            )
        return last or {"accepted": True}

    async def _append(
        self,
        session_id,
        topic,
        payload,
        source_time_ns=None,
        *,
        force=False,
        sample_monotonic_ns=None,
    ):
        if (
            not self.active_session
            or self.active_session["id"] != session_id
            or not self.recorder
        ):
            raise DataError("SESSION_NOT_ACTIVE")
        if (
            self.session_stopping
            and not force
            and not (self.dagger and time.monotonic_ns() <= self.drain_until_ns)
        ):
            raise DataError("SESSION_STOPPING")
        if not isinstance(topic, str) or not topic.startswith("/") or len(topic) > 512:
            raise DataError("TOPIC_INVALID")
        log_ns = time.time_ns()
        arrival_mono = time.monotonic_ns()
        time_source = "rpc_call" if sample_monotonic_ns is not None else "unknown"
        if isinstance(payload, dict):
            for key in (
                "issued_mono_ns",
                "mono_ns",
                "received_monotonic_ns",
                "sample_monotonic_ns",
                "monotonic_ns",
            ):
                if payload.get(key) is not None:
                    sample_monotonic_ns, time_source = payload[key], key
                    break
        if sample_monotonic_ns is not None and (
            type(sample_monotonic_ns) is not int
            or not 0 <= sample_monotonic_ns <= arrival_mono + 5_000_000
        ):
            raise DataError("SAMPLE_MONOTONIC_TIMESTAMP_INVALID")
        if self.dagger and sample_monotonic_ns is not None:
            clock = self.active_session["clock"]
            log_ns = (
                clock["wall_origin_ns"]
                + sample_monotonic_ns
                - clock["monotonic_origin_ns"]
            )
        source_ns = log_ns if source_time_ns is None else source_time_ns
        if type(source_ns) is not int or not 0 <= source_ns < 2**64:
            raise DataError("SOURCE_TIMESTAMP_INVALID")
        if isinstance(payload, dict) and "__cdr_b64" in payload:
            try:
                body = base64.b64decode(payload["__cdr_b64"], validate=True)
                ros_type, ros_schema = payload["__ros_type"], payload["__ros_schema"]
                if (
                    not isinstance(ros_type, str)
                    or not isinstance(ros_schema, str)
                    or not ros_type
                    or not ros_schema
                ):
                    raise ValueError()
                if len(body) < 4 or len(ros_type) > 512 or len(ros_schema) > 256 * 1024:
                    raise ValueError()
            except (KeyError, ValueError, TypeError):
                raise DataError("CDR_SAMPLE_INVALID") from None
        else:
            raw = canonical(
                {
                    "schema_version": "hc.sample/v1",
                    "session_id": session_id,
                    "received_monotonic_ns": arrival_mono,
                    "received_wall_time_ns": time.time_ns(),
                    "sample_monotonic_ns": sample_monotonic_ns,
                    "sample_time_source": time_source,
                    "source_time_ns": source_ns,
                    "payload": payload,
                }
            ).encode()
            body = b"\x00\x01\x00\x00" + struct.pack("<I", len(raw) + 1) + raw + b"\0"
            ros_type, ros_schema = "std_msgs/msg/String", "string data\n"
        if len(body) > 16 * 1024 * 1024:
            raise DataError("SAMPLE_TOO_LARGE")
        item = (topic, body, log_ns, source_ns, ros_type, ros_schema)
        try:
            if self.dagger and not force:
                if self.recorder.failure:
                    raise DataError(self.recorder.failure)
                if topic == "/capture/gaps":
                    raise DataError(
                        "DAGGER_CAPTURE_GAP",
                        "A capture gap prevents verified correction boundaries",
                    )
                control = (
                    payload
                    if isinstance(payload, dict)
                    and topic.startswith("/platform/control/")
                    else {}
                )
                metadata = {
                    "sample_time_source": time_source,
                    "control_epoch": control.get("epoch", control.get("control_epoch")),
                    "control_owner": control.get("owner", control.get("source")),
                    "control_boot_id": control.get("boot_id"),
                    "control_stage": control.get("stage"),
                    "control_sequence": control.get("seq"),
                }
                if (
                    metadata["control_epoch"] is not None
                    and type(metadata["control_epoch"]) is not int
                ) or (
                    metadata["control_owner"] is not None
                    and not isinstance(metadata["control_owner"], str)
                ):
                    raise DataError("DAGGER_CONTROL_METADATA_INVALID")
                buffered = self.dagger.add_sample(item, sample_monotonic_ns, metadata)
                self._drain_dagger(arrival_mono)
                return {
                    "accepted": buffered,
                    "buffered": buffered,
                    "armed": not self.recording_gate,
                }
            self.recorder.append(*item)
        except DataError as error:
            self.recorder.failure = error.code
            self.active_session["state"] = "PARTIAL"
            await self.store.run(
                lambda db, error=error: db.execute(
                    "UPDATE sessions SET state='PARTIAL',error=? WHERE id=?",
                    (canonical(error.as_dict()), session_id),
                )
            )
            raise
        return {"accepted": True, "sequence": self.recorder.accepted}

    def _drain_dagger(self, now_ns, *, final=False):
        for item, metadata in self.dagger.drain(
            now_ns, final=final, stop_ns=self.stop_requested_ns or None
        ):
            self.recorder.append(*item, metadata=metadata)

    async def _dagger_flusher(self):
        try:
            while self.dagger and self.recorder and not self.recorder.failure:
                self._drain_dagger(time.monotonic_ns())
                await asyncio.sleep(0.02)
        except DataError as error:
            if self.recorder and self.active_session:
                self.recorder.failure = error.code
                self.recorder.local_errors += 1
                self.active_session["state"] = "PARTIAL"
                identity = self.active_session["id"]
                await self.store.run(
                    lambda db, error=error: db.execute(
                        "UPDATE sessions SET state='PARTIAL',error=? WHERE id=?",
                        (canonical(error.as_dict()), identity),
                    )
                )

    async def set_intervention(self, session_id, active, event):
        async with self.session_lock:
            if not self.active_session or self.active_session["id"] != session_id:
                raise DataError("SESSION_NOT_ACTIVE")
            if self.active_session["spec"].get("mode") not in {
                "dagger_full",
                "dagger_corrections",
            }:
                raise DataError("NOT_DAGGER_SESSION")
            if (
                not isinstance(event, dict)
                or event.get("confirmed") is not True
                or not event.get("event_id")
                or not isinstance(event["event_id"], str)
                or len(event["event_id"]) > 256
                or type(event.get("control_epoch")) is not int
                or event["control_epoch"] < 0
                or type(event.get("monotonic_ns")) is not int
            ):
                raise DataError("CONFIRMED_OWNERSHIP_EVENT_REQUIRED")
            if (
                event.get("active", bool(active)) != bool(active)
                or not 0 <= event["monotonic_ns"] <= time.monotonic_ns() + 5_000_000
            ):
                raise DataError("INTERVENTION_TIMESTAMP_INVALID")
            try:
                if not self.dagger.add_event(active, event):
                    return {"duplicate": True, "active": self.intervention is not None}
            except DataError as error:
                self.recorder.failure = error.code
                self.active_session["state"] = "PARTIAL"
                await self.store.run(
                    lambda db, error=error: db.execute(
                        "UPDATE sessions SET state='PARTIAL',error=? WHERE id=?",
                        (canonical(error.as_dict()), session_id),
                    )
                )
                raise
            await self._append(
                session_id,
                "/hc/control/intervention",
                {
                    "active": bool(active),
                    **event,
                    "interval_start": self.intervention if not active else None,
                },
                None,
                force=True,
            )
            self.intervention = self.dagger.active_event(time.monotonic_ns())
            self.recording_gate = (
                self.intervention is not None
                or self.active_session["spec"].get("mode") == "dagger_full"
            )
            self.active_session["state"] = (
                "RECORDING" if self.recording_gate else "ARMED"
            )
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE sessions SET state=? WHERE id=?",
                    (self.active_session["state"], session_id),
                )
            )
            return {
                "active": self.intervention is not None,
                "control_epoch": event["control_epoch"],
                "watermark_monotonic_ns": self.dagger.watermark_ns,
            }

    async def stop_session(self, session_id, cutoff_monotonic_ns=None):
        async with self.session_lock:
            if not self.active_session or self.active_session["id"] != session_id:
                return await self.get_session(session_id)
            if self.finalize_task is None:
                self.session_stopping = True
                now_ns = time.monotonic_ns()
                if cutoff_monotonic_ns is not None and (
                    type(cutoff_monotonic_ns) is not int
                    or not self.active_session["clock"]["monotonic_origin_ns"]
                    <= cutoff_monotonic_ns
                    <= now_ns
                ):
                    self.session_stopping = False
                    raise DataError("RECORDING_CUTOFF_INVALID")
                self.stop_requested_ns = (
                    now_ns if cutoff_monotonic_ns is None else cutoff_monotonic_ns
                )
                if self.dagger and self.dagger.watermark_ns > self.stop_requested_ns:
                    self.recorder.failure = "DAGGER_CUTOFF_BEHIND_WATERMARK"
                self.drain_until_ns = now_ns + (
                    self.dagger.delay_ns if self.dagger else 0
                )
                self.finalize_task = asyncio.create_task(
                    self._finalize_session(session_id), name="recording-finalize"
                )
            task = self.finalize_task
        return await asyncio.shield(task)

    async def _finalize_session(self, session_id):
        # The lock remains free during the bounded tail drain so independent
        # capture/audit connections can deliver already-observed samples/events.
        if self.dagger:
            await asyncio.sleep(
                max(0.0, (self.drain_until_ns - time.monotonic_ns()) / 1e9)
            )
        async with self.session_lock:
            if not self.active_session or self.active_session["id"] != session_id:
                return await self.get_session(session_id)
            self.session_stopping = True
            session, recorder = self.active_session, self.recorder
            ended = time.time_ns()
            session["state"] = "FINALIZING"
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE sessions SET state='FINALIZING',ended_ns=? WHERE id=?",
                    (ended, session_id),
                )
            )
            try:
                if self.dagger_task:
                    self.dagger_task.cancel()
                    await asyncio.gather(self.dagger_task, return_exceptions=True)
                    self.dagger_task = None
                if self.dagger:
                    self._drain_dagger(time.monotonic_ns(), final=True)
                    self.intervention = self.dagger.active_event(self.stop_requested_ns)
                    await asyncio.to_thread(
                        atomic_json,
                        Path(session["root"]) / "interventions.json",
                        {
                            "schema_version": "hc-interventions/v1",
                            "events": self.dagger.events,
                            "statistics": self.dagger.statistics(),
                            "intervals": self.dagger.intervals(self.stop_requested_ns),
                            "stop_monotonic_ns": self.stop_requested_ns,
                            "clock": session["clock"],
                        },
                    )
                stats = await recorder.finish()
                if self.dagger:
                    stats["reorder"] = self.dagger.statistics()
                staging = Path(session["root"])
                spec = session["spec"]
                manifest = await asyncio.to_thread(
                    seal_directory,
                    staging,
                    {
                        "artifact_id": session_id,
                        "session_id": session_id,
                        "format": "mcap",
                        "requested_format": spec.get(
                            "local_format", spec.get("format", "mcap")
                        ),
                        "session": {**session, "ended_ns": ended},
                        "stats": stats,
                        "incomplete_intervention": self.intervention,
                        "quality": "unreviewed",
                        "task_success": None,
                    },
                )
                await asyncio.to_thread(inspect_mcap, staging / spec["filename"])
                destination = self._destination(session_id, spec)
                await asyncio.to_thread(os.replace, staging, destination)
                await asyncio.to_thread(sync_directory, destination.parent)
                artifact = await self._register_artifact(destination, manifest)
                await self.store.run(
                    lambda db: db.execute(
                        "UPDATE sessions SET state='READY',root=?,stats=? WHERE id=?",
                        (str(destination), canonical(stats), session_id),
                    )
                )
            except Exception as error:
                failure = {
                    "code": getattr(error, "code", "RECORDING_FINALIZE_FAILED"),
                    "message": str(error),
                }
                await self.store.run(
                    lambda db: db.execute(
                        "UPDATE sessions SET state='PARTIAL',error=? WHERE id=?",
                        (canonical(failure), session_id),
                    )
                )
                try:
                    await recorder.abort()
                except Exception:
                    pass
                raise DataError(
                    failure["code"],
                    "Partial source retained; no ready artifact was published",
                ) from error
            finally:
                self.active_session = None
                self.recorder = None
                self.intervention = None
                self.dagger = None
                self.drain_until_ns = 0
            await self._route_session_outputs(artifact)
            return {
                **(await self.get_session(session_id)),
                "artifact_id": artifact["id"],
            }

    async def _route_session_outputs(self, artifact):
        spec = artifact["manifest"].get("session", {}).get("spec", {})
        local = spec.get("local_format", spec.get("format", "mcap"))
        upload = (
            spec.get("upload_format", "mcap")
            if spec.get("auto_upload") is True
            else None
        )
        binding = spec.get("_frozen_upload_config")
        if upload and not binding:
            raise DataError(
                "FROZEN_UPLOAD_BINDING_MISSING",
                "Recording upload settings were not frozen; use an explicit history upload",
            )
        if local != "mcap":
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE sessions SET state='POSTPROCESSING' WHERE id=? AND state IN ('READY','INTERRUPTED')",
                    (artifact["id"],),
                )
            )
        if upload == "mcap":
            await self._enqueue(
                "upload",
                artifact["id"],
                "session-upload:" + artifact["id"],
                {"target": spec["upload_target"], "platform_config": binding},
            )
        for kind, output_format, prefix, required_local, wanted_upload in (
            (
                "conversion",
                "lerobot",
                "session-convert:",
                local in {"lerobot", "mcap_lerobot"},
                upload == "lerobot",
            ),
            (
                "video",
                "mcap_mp4",
                "session-video:",
                local == "mcap_mp4",
                upload == "mcap_mp4",
            ),
        ):
            if not required_local and not wanted_upload:
                continue
            request_id = prefix + artifact["id"]
            existing = await self.store.run(
                lambda db, r=request_id: db.execute(
                    "SELECT * FROM jobs WHERE request_id=?", (r,)
                ).fetchone()
            )
            if existing:
                existing = _row(existing)
                if (
                    existing["artifact_id"] != artifact["id"]
                    or existing["kind"] != kind
                ):
                    raise DataError("IDEMPOTENCY_CONFLICT")
                if required_local and existing["state"] in {
                    "SUCCEEDED",
                    "FAILED",
                    "CANCELED",
                }:
                    await self.store.run(
                        lambda db, e=existing: db.execute(
                            "UPDATE sessions SET state=? WHERE id=?",
                            (
                                "READY"
                                if e["state"] == "SUCCEEDED"
                                else "DERIVATION_FAILED",
                                artifact["id"],
                            ),
                        )
                    )
                continue
            options = dict(
                spec.get("conversion" if kind == "conversion" else "video", {})
            )
            if kind == "conversion":
                options = self._conversion_options(artifact, options)
            command = (
                list(self.converter_command)
                if kind == "conversion"
                else [sys.executable, "-m", "hc_platform.data.video_worker"]
            )
            await self._enqueue(
                kind,
                artifact["id"],
                request_id,
                {
                    "options": options,
                    "converter_command": command,
                    "route": {
                        "required_for_local": required_local,
                        "upload_on_success": wanted_upload,
                        "upload_target": spec.get("upload_target"),
                        "platform_config": binding,
                        "output_format": output_format,
                    },
                },
            )

    async def get_session(self, session_id):
        return _row(
            await self.store.run(
                lambda db: db.execute(
                    "SELECT * FROM sessions WHERE id=?", (session_id,)
                ).fetchone()
            )
        )

    @staticmethod
    def _filename(value):
        if (
            not isinstance(value, str)
            or not value
            or len(value.encode()) > 200
            or value.startswith(".")
            or any(c in value for c in ("/", "\\", "\0", ":"))
        ):
            raise DataError(
                "RECORDING_FILENAME_INVALID", "Use a safe basename, without directories"
            )
        if value.lower().endswith(".mcap"):
            return value[:-5] + ".mcap"
        if Path(value).suffix:
            raise DataError(
                "RECORDING_FILENAME_INVALID",
                "Recording filename must have the .mcap extension",
            )
        return value + ".mcap"

    def _save_subdirectory(self, value):
        if value in (None, ""):
            return ""
        if (
            not isinstance(value, str)
            or len(value) > 240
            or "\\" in value
            or "\0" in value
            or ":" in value
        ):
            raise DataError("RECORDING_SAVE_DIRECTORY_INVALID")
        root = self.root / "artifacts"
        candidate = Path(value)
        if candidate.is_absolute():
            try:
                value = candidate.relative_to(root).as_posix()
            except ValueError:
                raise DataError("RECORDING_SAVE_DIRECTORY_OUTSIDE_ROOT") from None
        if value in ("", "."):
            return ""
        if any(part in {"", ".", ".."} for part in value.split("/")):
            raise DataError("RECORDING_SAVE_DIRECTORY_INVALID")
        current = root
        for part in value.split("/"):
            current /= part
            if current.is_symlink():
                raise DataError("RECORDING_SAVE_DIRECTORY_SYMLINK")
        return value

    def _destination(self, identity, spec):
        folder = self._save_subdirectory(
            spec.get("save_subdirectory", spec.get("selected_save_dir", ""))
        )
        parent = self.root / "artifacts" / folder
        parent.mkdir(parents=True, exist_ok=True)
        return parent / identity

    async def list_sessions(self):
        return await self.store.run(
            lambda db: [
                _row(r)
                for r in db.execute(
                    "SELECT * FROM sessions ORDER BY started_ns DESC LIMIT 1000"
                )
            ]
        )

    async def _register_artifact(self, root, manifest):
        identity = manifest["artifact_id"]

        def register(db):
            old = db.execute(
                "SELECT * FROM artifacts WHERE id=?", (identity,)
            ).fetchone()
            if old:
                if (
                    json.loads(old["manifest"])["content_sha256"]
                    != manifest["content_sha256"]
                ):
                    raise DataError("ARTIFACT_ID_CONFLICT")
                return _row(old)
            db.execute(
                "INSERT INTO artifacts(id,session_id,state,format,root,manifest,created) VALUES(?,?,?,?,?,?,?)",
                (
                    identity,
                    manifest.get("session_id"),
                    "READY",
                    manifest["format"],
                    str(root),
                    canonical(manifest),
                    time.time(),
                ),
            )
            return _row(
                db.execute("SELECT * FROM artifacts WHERE id=?", (identity,)).fetchone()
            )

        return await self.store.run(register)

    def _dataset_summary(self, artifact):
        manifest = artifact["manifest"]
        session, stats = manifest.get("session", {}), manifest.get("stats", {})
        spec = session.get("spec", manifest.get("import_spec", {}))
        topic_names = list(stats.get("topics", {}))
        camera_count = len(manifest.get("cameras", [])) or sum(
            feature.get("dtype") == "video"
            for feature in manifest.get("features", {}).values()
        )
        duration_ns = stats.get("duration_ns")
        if duration_ns is None:
            duration_ns = max(
                0,
                (session.get("ended_ns") or manifest.get("ended_ns") or 0)
                - (session.get("started_ns") or manifest.get("started_ns") or 0),
            )
        return {
            "robot_id": spec.get("robot_id", spec.get("robot_type")),
            "size_bytes": sum(asset["size_bytes"] for asset in manifest["assets"]),
            "duration_s": duration_ns / 1e9,
            "message_count": stats.get("message_count"),
            "frame_count": stats.get("frame_count"),
            "topic_count": len(topic_names),
            "topics": topic_names,
            "camera_count": camera_count,
            "requested_format": manifest.get("requested_format", artifact["format"]),
            "task_id": spec.get("upload_target", {}).get("collection_task_id"),
            "quality": manifest.get("quality", "unreviewed"),
        }

    async def list_datasets(self, limit=100, offset=0, include_manifest=False):
        maximum = 100000 if include_manifest else 1000
        limit, offset = min(maximum, max(1, int(limit))), max(0, int(offset))
        values = await self.store.run(
            lambda db: [
                _row(r)
                for r in db.execute(
                    "SELECT * FROM artifacts WHERE state!='DELETED' ORDER BY created DESC LIMIT ? OFFSET ?",
                    (limit, offset),
                )
            ]
        )
        for value in values:
            value["summary"] = self._dataset_summary(value)
            if not include_manifest:
                value.pop("manifest")
        return values

    async def get_dataset(self, artifact_id):
        result = _row(
            await self.store.run(
                lambda db: db.execute(
                    "SELECT * FROM artifacts WHERE id=? AND state!='DELETED'",
                    (artifact_id,),
                ).fetchone()
            )
        )
        result["summary"] = self._dataset_summary(result)
        result["references"] = await self.store.run(
            lambda db: [
                dict(r)
                for r in db.execute(
                    "SELECT owner,kind,expires FROM artifact_refs WHERE artifact_id=? AND (expires IS NULL OR expires>?)",
                    (artifact_id, time.time()),
                )
            ]
        )
        result["video_assets"] = [
            item["path"]
            for item in result["manifest"]["assets"]
            if item["path"].endswith(".mp4")
        ]
        return result

    async def import_mcap(self, path, request_id, spec=None):
        # The privileged console decides whether the caller may read this path.
        source = Path(path)
        if not source.is_file() or source.is_symlink():
            raise DataError("IMPORT_PATH_INVALID")
        identity = str(uuid.uuid5(uuid.NAMESPACE_URL, "hc-import:" + request_id))
        try:
            return await self.get_dataset(identity)
        except DataError as error:
            if error.code != "NOT_FOUND":
                raise
        staging = self.root / "staging" / (identity + ".partial")
        staging.mkdir(exist_ok=True)
        await asyncio.to_thread(shutil.copyfile, source, staging / "capture.mcap")
        stats = await asyncio.to_thread(inspect_mcap, staging / "capture.mcap")
        if stats["message_count"] == 0:
            raise DataError("EMPTY_MCAP")
        manifest = await asyncio.to_thread(
            seal_directory,
            staging,
            {
                "artifact_id": identity,
                "format": "mcap",
                "stats": stats,
                "started_ns": stats["started_ns"],
                "ended_ns": stats["ended_ns"],
                "import_spec": spec or {},
                "quality": "unreviewed",
                "task_success": None,
            },
        )
        destination = self.root / "artifacts" / identity
        await asyncio.to_thread(os.replace, staging, destination)
        await asyncio.to_thread(sync_directory, destination.parent)
        return await self._register_artifact(destination, manifest)

    async def acquire_artifact(self, artifact_id, owner, ttl_seconds=300):
        if not owner or owner.startswith("job:"):
            raise DataError("REFERENCE_OWNER_INVALID")
        expires = time.time() + min(3600, max(1, int(ttl_seconds)))

        def acquire(db):
            row = db.execute(
                "SELECT * FROM artifacts WHERE id=?", (artifact_id,)
            ).fetchone()
            if not row or row["state"] != "READY":
                raise DataError("ARTIFACT_NOT_READY")
            db.execute(
                "INSERT INTO artifact_refs VALUES(?,?,?,?) ON CONFLICT(artifact_id,owner) DO UPDATE SET expires=excluded.expires",
                (artifact_id, owner, "reader", expires),
            )
            return {"artifact_id": artifact_id, "owner": owner, "expires": expires}

        return await self.store.run(acquire)

    async def release_artifact(self, artifact_id, owner):
        if owner.startswith("job:"):
            raise DataError("REFERENCE_OWNER_INVALID")
        await self.store.run(
            lambda db: db.execute(
                "DELETE FROM artifact_refs WHERE artifact_id=? AND owner=?",
                (artifact_id, owner),
            )
        )
        return {"released": True}

    async def resolve_asset(self, artifact_id, path, owner):
        artifact = await self.get_dataset(artifact_id)
        if artifact["state"] != "READY" or not any(
            r["owner"] == owner for r in artifact["references"]
        ):
            raise DataError("ARTIFACT_LEASE_REQUIRED")
        if path not in {a["path"] for a in artifact["manifest"]["assets"]}:
            raise DataError("ASSET_NOT_FOUND")
        return {
            "path": str(safe_asset(artifact["root"], path)),
            "artifact_id": artifact_id,
            "owner": owner,
        }

    async def prepare_download(self, artifact_id, owner=None):
        owner = owner or "download:" + str(uuid.uuid4())
        lease = await self.acquire_artifact(artifact_id, owner, 3600)
        artifact = await self.get_dataset(artifact_id)
        destination = self.root / "downloads" / (str(uuid.uuid4()) + ".tar")

        def archive():
            with tarfile.open(destination, "w") as stream:
                stream.add(
                    Path(artifact["root"]) / "manifest.json",
                    arcname="manifest.json",
                    recursive=False,
                )
                for asset in artifact["manifest"]["assets"]:
                    stream.add(
                        safe_asset(artifact["root"], asset["path"]),
                        arcname=asset["path"],
                        recursive=False,
                    )

        try:
            await asyncio.to_thread(archive)
        except BaseException:
            destination.unlink(missing_ok=True)
            await self.release_artifact(artifact_id, owner)
            raise
        return {
            **lease,
            "path": str(destination),
            "size_bytes": destination.stat().st_size,
        }

    async def delete_dataset(self, artifact_id):
        def tombstone(db):
            db.execute(
                "DELETE FROM artifact_refs WHERE expires IS NOT NULL AND expires<=?",
                (time.time(),),
            )
            row = db.execute(
                "SELECT * FROM artifacts WHERE id=?", (artifact_id,)
            ).fetchone()
            if row is None:
                raise DataError("NOT_FOUND")
            if row["state"] == "DELETED":
                return None
            if db.execute(
                "SELECT 1 FROM artifact_refs WHERE artifact_id=?", (artifact_id,)
            ).fetchone():
                raise DataError("ARTIFACT_IN_USE")
            db.execute(
                "UPDATE artifacts SET state='DELETING' WHERE id=?", (artifact_id,)
            )
            return row["root"]

        root = await self.store.run(tombstone)
        if root:
            await asyncio.to_thread(shutil.rmtree, root)
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE artifacts SET state='DELETED' WHERE id=?", (artifact_id,)
                )
            )
        return {"deleted": True, "artifact_id": artifact_id}

    async def enqueue_conversion(self, artifact_id, request_id, options=None):
        artifact = await self.get_dataset(artifact_id)
        return await self._enqueue(
            "conversion",
            artifact_id,
            request_id,
            {
                "options": self._conversion_options(artifact, options or {}),
                "converter_command": list(self.converter_command),
            },
        )

    @staticmethod
    def _conversion_options(artifact, supplied):
        spec = (
            artifact["manifest"]
            .get("session", {})
            .get("spec", artifact["manifest"].get("import_spec", {}))
        )
        options = {**spec.get("conversion", {}), **supplied}
        robot = spec.get("robot_config", {})
        selected_names = [
            name
            for name in robot.get("joint_names", [])
            if any(
                name in robot.get("groups", {}).get(group, [])
                for group in robot.get("allowed_groups", [])
            )
        ]
        if selected_names:
            options.setdefault("joint_names", selected_names)
        options.setdefault("state_topic", "/platform/control/state")
        options.setdefault("action_topic", "/platform/control/commands")
        options.setdefault("fps", spec.get("dataset_fps", spec.get("fps", 30)))
        options.setdefault("task", spec.get("task_description", "teleoperation"))
        options.setdefault("robot_type", spec.get("robot_id", "x1"))
        if robot.get("joint_units") and options.get("joint_names"):
            units = dict(zip(robot["joint_names"], robot["joint_units"]))
            if all(name in units for name in options["joint_names"]):
                options.setdefault(
                    "joint_units", [units[name] for name in options["joint_names"]]
                )
        present = artifact["manifest"].get("stats", {}).get("topics", {})
        cameras = {}
        for camera in robot.get("camera", {}).get("streams", []):
            topic = camera.get("topic", "")
            # Depth is not an RGB training feature. Explicit mappings remain
            # subject to the image decoder's supported-format validation.
            if "/depth/" not in topic and topic in present and camera.get("id"):
                cameras["observation.images." + camera["id"]] = topic
        if cameras:
            options.setdefault("cameras", cameras)
        if spec.get("mode") == "dagger_corrections":
            options.setdefault("only_interventions", True)
        return options

    async def enqueue_video(self, artifact_id, request_id, options=None):
        return await self._enqueue(
            "video",
            artifact_id,
            request_id,
            {
                "options": options or {},
                "converter_command": [
                    sys.executable,
                    "-m",
                    "hc_platform.data.video_worker",
                ],
            },
        )

    async def enqueue_upload(self, artifact_id, request_id, target=None):
        if self.platform_config is None:
            raise DataError("PLATFORM_NOT_CONFIGURED")
        target = target or {}
        if not target.get("robot_id") or not target.get("collection_task_id"):
            raise DataError("UPLOAD_TARGET_REQUIRED")
        return await self._enqueue(
            "upload",
            artifact_id,
            request_id,
            {"target": target, "platform_config": self.platform_config},
        )

    async def _enqueue(self, kind, artifact_id, request_id, params):
        if not request_id or len(request_id) > 256:
            raise DataError("JOB_REQUEST_INVALID")
        params = json.loads(canonical(params))

        def enqueue(db):
            old = db.execute(
                "SELECT * FROM jobs WHERE request_id=?", (request_id,)
            ).fetchone()
            if old:
                value = _row(old)
                if (
                    value["kind"] != kind
                    or value["artifact_id"] != artifact_id
                    or value["params"] != params
                ):
                    raise DataError("IDEMPOTENCY_CONFLICT")
                return value
            asset = db.execute(
                "SELECT state FROM artifacts WHERE id=?", (artifact_id,)
            ).fetchone()
            if not asset or asset[0] != "READY":
                raise DataError("ARTIFACT_NOT_READY")
            identity, now = str(uuid.uuid4()), time.time()
            db.execute(
                "INSERT INTO jobs(id,kind,artifact_id,request_id,state,params,created,updated) VALUES(?,?,?,?,?,?,?,?)",
                (
                    identity,
                    kind,
                    artifact_id,
                    request_id,
                    "QUEUED",
                    canonical(params),
                    now,
                    now,
                ),
            )
            db.execute(
                "INSERT INTO artifact_refs VALUES(?,?,?,NULL)",
                (artifact_id, "job:" + identity, "job"),
            )
            db.execute(
                "INSERT INTO job_events(job_id,timestamp,state,detail) VALUES(?,?,?,?)",
                (identity, now, "QUEUED", "{}"),
            )
            return _row(
                db.execute("SELECT * FROM jobs WHERE id=?", (identity,)).fetchone()
            )

        return await self.store.run(enqueue)

    async def get_job(self, job_id):
        return _row(
            await self.store.run(
                lambda db: db.execute(
                    "SELECT * FROM jobs WHERE id=?", (job_id,)
                ).fetchone()
            )
        )

    async def list_jobs(self):
        return await self.store.run(
            lambda db: [
                _row(r)
                for r in db.execute(
                    "SELECT * FROM jobs ORDER BY created DESC LIMIT 1000"
                )
            ]
        )

    async def job_events(self, job_id):
        return await self.store.run(
            lambda db: [
                dict(r)
                for r in db.execute(
                    "SELECT * FROM job_events WHERE job_id=? ORDER BY seq", (job_id,)
                )
            ]
        )

    async def cancel_job(self, job_id):
        def cancel(db):
            row = db.execute("SELECT * FROM jobs WHERE id=?", (job_id,)).fetchone()
            if not row:
                raise DataError("NOT_FOUND")
            if row["state"] in {"SUCCEEDED", "FAILED", "CANCELED"}:
                return _row(row)
            state = "CANCEL_REQUESTED" if row["state"] == "RUNNING" else "CANCELED"
            db.execute(
                "UPDATE jobs SET state=?,cancel_requested=1,updated=? WHERE id=?",
                (state, time.time(), job_id),
            )
            if state == "CANCELED":
                db.execute(
                    "DELETE FROM artifact_refs WHERE owner=?", ("job:" + job_id,)
                )
            return _row(
                db.execute("SELECT * FROM jobs WHERE id=?", (job_id,)).fetchone()
            )

        result = await self.store.run(cancel)
        process = self.processes.get(job_id)
        if process and process.returncode is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        return result

    async def retry_job(self, job_id):
        def retry(db):
            job = _row(
                db.execute("SELECT * FROM jobs WHERE id=?", (job_id,)).fetchone()
            )
            if job["state"] not in {"FAILED", "RETRY_WAIT", "CANCELED"}:
                raise DataError("JOB_NOT_RETRYABLE")
            asset = db.execute(
                "SELECT state FROM artifacts WHERE id=?", (job["artifact_id"],)
            ).fetchone()
            if not asset or asset[0] != "READY":
                raise DataError("ARTIFACT_NOT_READY")
            db.execute(
                "INSERT OR IGNORE INTO artifact_refs VALUES(?,?,?,NULL)",
                (job["artifact_id"], "job:" + job_id, "job"),
            )
            db.execute(
                "UPDATE jobs SET state='QUEUED',cancel_requested=0,next_run=0,error=NULL,updated=? WHERE id=?",
                (time.time(), job_id),
            )
            return _row(
                db.execute("SELECT * FROM jobs WHERE id=?", (job_id,)).fetchone()
            )

        return await self.store.run(retry)

    async def _job_runner(self):
        while True:
            self.jobs = {
                identity: task
                for identity, task in self.jobs.items()
                if not task.done()
            }
            due = await self.store.run(
                lambda db: [
                    _row(r)
                    for r in db.execute(
                        "SELECT * FROM jobs WHERE state IN ('QUEUED','RETRY_WAIT') AND next_run<=? ORDER BY created LIMIT 32",
                        (time.time(),),
                    )
                ]
            )
            active = {"conversion": 0, "video": 0, "upload": 0}
            for identity in self.jobs:
                job = await self.get_job(identity)
                active[job["kind"]] += 1
            for job in due:
                if (
                    job["kind"] != "upload"
                    and active["conversion"] + active["video"] >= 1
                ) or (job["kind"] == "upload" and active["upload"] >= self.max_uploads):
                    continue
                active[job["kind"]] += 1
                self.jobs[job["id"]] = asyncio.create_task(
                    self._run_job(job), name="data-job:" + job["id"]
                )
            await asyncio.sleep(0.2)

    async def _run_job(self, job):
        identity = job["id"]

        async def checkpoint(result):
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE jobs SET result=?,updated=? WHERE id=?",
                    (canonical(result), time.time(), identity),
                )
            )

        async def cancelled():
            current = await self.get_job(identity)
            if current["cancel_requested"]:
                raise DataError(
                    "JOB_CANCELED",
                    "Local transfer stopped; remote committed state may require reconciliation",
                )

        try:

            def claim(db):
                return db.execute(
                    "UPDATE jobs SET state='RUNNING',attempts=attempts+1,updated=? WHERE id=? AND state IN ('QUEUED','RETRY_WAIT') AND cancel_requested=0",
                    (time.time(), identity),
                ).rowcount

            if not await self.store.run(claim):
                return
            artifact = await self.get_dataset(job["artifact_id"])
            if job["kind"] in {"conversion", "video"}:
                result = await self._convert(job, artifact, cancelled)
            else:
                adapter = PlatformAdapter(job["params"]["platform_config"])
                result = await adapter.upload(
                    artifact,
                    identity,
                    job["params"]["target"],
                    job["result"],
                    checkpoint,
                    cancelled,
                )
            await self._finish_job(identity, "SUCCEEDED", result=result)
        except asyncio.CancelledError:
            await self.store.run(
                lambda db: db.execute(
                    "UPDATE jobs SET state=CASE WHEN cancel_requested=1 THEN 'CANCELED' ELSE 'QUEUED' END,updated=? WHERE id=?",
                    (time.time(), identity),
                )
            )
            raise
        except Exception as error:
            current = await self.get_job(identity)
            if (
                current["cancel_requested"]
                or getattr(error, "code", None) == "JOB_CANCELED"
            ):
                await self._finish_job(
                    identity,
                    "CANCELED",
                    error={"code": "JOB_CANCELED", "remote_cancelled": False},
                )
            elif (
                isinstance(error, DataError)
                and error.retryable
                and current["attempts"] < 10
            ):
                delay = min(300, 2 ** min(current["attempts"], 8)) * random.uniform(
                    0.8, 1.2
                )
                await self.store.run(
                    lambda db, error=error: db.execute(
                        "UPDATE jobs SET state='RETRY_WAIT',error=?,next_run=?,updated=? WHERE id=?",
                        (
                            canonical(error.as_dict()),
                            time.time() + delay,
                            time.time(),
                            identity,
                        ),
                    )
                )
            else:
                detail = (
                    error.as_dict()
                    if isinstance(error, DataError)
                    else {"code": "JOB_FAILED", "message": type(error).__name__}
                )
                await self._finish_job(identity, "FAILED", error=detail)

    async def _finish_job(self, identity, state, *, result=None, error=None):
        def finish(db):
            if result is not None:
                db.execute(
                    "UPDATE jobs SET result=? WHERE id=?", (canonical(result), identity)
                )
            db.execute(
                "UPDATE jobs SET state=?,error=?,updated=? WHERE id=?",
                (state, canonical(error) if error else None, time.time(), identity),
            )
            db.execute("DELETE FROM artifact_refs WHERE owner=?", ("job:" + identity,))
            db.execute(
                "INSERT INTO job_events(job_id,timestamp,state,detail) VALUES(?,?,?,?)",
                (identity, time.time(), state, canonical(error or {})),
            )
            job = db.execute(
                "SELECT request_id,artifact_id,kind,params FROM jobs WHERE id=?",
                (identity,),
            ).fetchone()
            if job["kind"] in {"conversion", "video"} and json.loads(job["params"]).get(
                "route", {}
            ).get("required_for_local"):
                db.execute(
                    "UPDATE sessions SET state=? WHERE id=(SELECT session_id FROM artifacts WHERE id=?)",
                    (
                        "READY" if state == "SUCCEEDED" else "DERIVATION_FAILED",
                        job["artifact_id"],
                    ),
                )

        await self.store.run(finish)

    async def _convert(self, job, artifact, cancelled):
        if artifact["format"] != "mcap":
            raise DataError("CONVERSION_SOURCE_UNSUPPORTED")
        await cancelled()
        await asyncio.to_thread(
            verify_directory, artifact["root"], artifact["manifest"]
        )
        destination = self._destination(
            job["id"], artifact["manifest"].get("session", {}).get("spec", {})
        )
        if (destination / "manifest.json").exists():
            manifest = json.loads(
                await asyncio.to_thread((destination / "manifest.json").read_text)
            )
            await asyncio.to_thread(verify_directory, destination, manifest)
            result = await self._register_artifact(destination, manifest)
            await self._route_derived_upload(job, result)
            return {"artifact_id": result["id"]}
        staging = self.root / "staging" / (job["id"] + ".partial")
        if staging.exists():
            await asyncio.to_thread(shutil.rmtree, staging)
        staging.mkdir()
        options = staging / "options.json"
        await asyncio.to_thread(atomic_json, options, job["params"]["options"])
        mcap_paths = [
            a["path"]
            for a in artifact["manifest"]["assets"]
            if a["path"].endswith(".mcap")
        ]
        if len(mcap_paths) != 1:
            raise DataError("CONVERSION_SOURCE_AMBIGUOUS")
        output = staging / "dataset"
        command = [
            *job["params"]["converter_command"],
            "--input",
            str(safe_asset(artifact["root"], mcap_paths[0])),
            "--output",
            str(output),
            "--options",
            str(options),
        ]
        log = (staging / "worker.log").open("wb")
        process = None
        environment = {
            **os.environ,
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
        }
        if (
            job["kind"] == "conversion"
            and Path(command[0]).resolve() != Path(sys.executable).resolve()
        ):
            # Do not leak ROS or the API interpreter's compiled dependencies
            # into the pinned, possibly different-version converter Python.
            environment.pop("PYTHONHOME", None)
            environment.update(
                PYTHONPATH=str(Path(__file__).resolve().parents[2]),
                PYTHONNOUSERSITE="1",
            )
        try:
            process = await asyncio.create_subprocess_exec(
                *command,
                stdout=log,
                stderr=asyncio.subprocess.STDOUT,
                env=environment,
                start_new_session=True,
            )
            self.processes[job["id"]] = process
            try:
                await asyncio.wait_for(
                    process.wait(),
                    timeout=min(
                        86400,
                        max(
                            1,
                            float(
                                job["params"]["options"].get("timeout_seconds", 3600)
                            ),
                        ),
                    ),
                )
            except asyncio.TimeoutError:
                raise DataError("CONVERSION_TIMEOUT") from None
            await cancelled()
            if process.returncode != 0:
                error_file = staging / "conversion-error.json"
                detail = (
                    json.loads(error_file.read_text()) if error_file.exists() else {}
                )
                raise DataError(
                    detail.get("code", "CONVERTER_FAILED"),
                    "Worker failed; source and worker log retained",
                )
            if job["kind"] == "video":
                from .video_worker import validate_bundle

                info = await asyncio.to_thread(validate_bundle, output)
            else:
                try:
                    info = await asyncio.to_thread(validate_lerobot, output)
                except DataError as error:
                    if error.code != "LEROBOT_VALIDATOR_UNAVAILABLE":
                        raise
                    validator = await asyncio.create_subprocess_exec(
                        *job["params"]["converter_command"],
                        "--validate-only",
                        str(output),
                        stdout=asyncio.subprocess.PIPE,
                        stderr=log,
                        env=environment,
                        start_new_session=True,
                    )
                    try:
                        raw = await asyncio.wait_for(
                            validator.stdout.read(1024 * 1024 + 1), 60
                        )
                        await asyncio.wait_for(validator.wait(), 10)
                        if validator.returncode or len(raw) > 1024 * 1024:
                            raise DataError("LEROBOT_VALIDATOR_FAILED")
                        info = json.loads(raw)
                    finally:
                        if validator.returncode is None:
                            validator.kill()
                            await validator.wait()
            if job["kind"] == "conversion":
                from ..capture_plan import validate_lerobot_features
                from ..configuration import ConfigurationError

                try:
                    validate_lerobot_features(
                        artifact["manifest"]
                        .get("session", {})
                        .get("spec", {})
                        .get("capture_snapshot"),
                        info,
                    )
                except ConfigurationError as error:
                    raise DataError("CAPTURE_STRUCTURE_MISMATCH", str(error)) from error
            package = artifact["manifest"].get("session", {}).get("robot_package")
            if package:
                await asyncio.to_thread(
                    freeze_robot_package,
                    safe_asset(artifact["root"], package["path"]),
                    output / "robot_package.zip",
                    package["sha256"],
                    package.get("package_hash"),
                )
            manifest = await asyncio.to_thread(
                seal_directory,
                output,
                {
                    "artifact_id": job["id"],
                    "session_id": artifact.get("session_id"),
                    "format": "mcap_mp4" if job["kind"] == "video" else "lerobot",
                    "source_artifact_id": artifact["id"],
                    "source_content_sha256": artifact["manifest"]["content_sha256"],
                    "conversion_options": job["params"]["options"],
                    "session": artifact["manifest"].get("session", {}),
                    "import_spec": artifact["manifest"].get("import_spec", {}),
                    "started_ns": artifact["manifest"].get("started_ns"),
                    "ended_ns": artifact["manifest"].get("ended_ns"),
                    "stats": {
                        "frame_count": info["total_frames"],
                        "episode_count": info.get("total_episodes"),
                        "fps": info["fps"],
                    },
                    "features": info.get("features", {}),
                    "cameras": info.get("cameras", []),
                    "quality": "unreviewed",
                    "task_success": None,
                },
            )
            await cancelled()
            await asyncio.to_thread(os.replace, output, destination)
            await asyncio.to_thread(sync_directory, destination.parent)
            result = await self._register_artifact(destination, manifest)
            await self._route_derived_upload(job, result)
            return {"artifact_id": result["id"], "frame_count": info["total_frames"]}
        finally:
            if process is not None and process.returncode is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(process.wait(), 3)
                except asyncio.TimeoutError:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    await process.wait()
            self.processes.pop(job["id"], None)
            log.close()

    async def _route_derived_upload(self, job, artifact):
        route = job["params"].get("route", {})
        if route.get("upload_on_success"):
            await self._enqueue(
                "upload",
                artifact["id"],
                "derived-upload:" + job["id"],
                {
                    "target": route["upload_target"],
                    "platform_config": route["platform_config"],
                },
            )

    async def configure_platform(self, config):
        config = validate_config(config)
        await self.store.run(
            lambda db: db.execute(
                "INSERT INTO settings VALUES('platform',?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                (canonical(config),),
            )
        )
        self.platform_config = config
        return {
            "configured": True,
            "platform_base_url": config["platform_base_url"],
            "robot_id": config.get("robot_id"),
        }

    async def connect_platform(
        self, link, local_robot, request_id, allow_insecure_lan=False
    ):
        try:
            parsed = urlsplit(link.strip())
            ticket = parse_qs(parsed.fragment, strict_parsing=True)["robot-connect"][0]
            if (
                parsed.query
                or parsed.username
                or parsed.password
                or len(ticket) < 32
                or len(link) > 4096
            ):
                raise ValueError()
            base = urlunsplit(
                (parsed.scheme, parsed.netloc, parsed.path.rstrip("/"), "", "")
            )
        except (ValueError, KeyError, TypeError, AttributeError):
            raise DataError("PLATFORM_CONNECTION_LINK_INVALID") from None
        config = validate_config(
            {
                "platform_base_url": base,
                "credential_env": "HC_UNUSED_CONNECT_CREDENTIAL",
                "allow_insecure_lan": bool(allow_insecure_lan),
            }
        )
        key = (
            "connection-attempt:"
            + hashlib.sha256((base + "\0" + ticket).encode()).hexdigest()
        )
        pending = {"request_id": request_id, "local_robot": local_robot}

        def freeze(db):
            previous = db.execute(
                "SELECT value FROM settings WHERE key=?", (key,)
            ).fetchone()
            if previous:
                value = json.loads(previous[0])
                if value["local_robot"] != local_robot:
                    raise DataError("PLATFORM_LOCAL_ROBOT_CHANGED")
                return value
            db.execute("INSERT INTO settings VALUES(?,?)", (key, canonical(pending)))
            return pending

        pending = await self.store.run(freeze)
        adapter = PlatformAdapter(config)
        async with adapter.client() as client:
            response = await adapter.request(
                client,
                "POST",
                "connections:redeem",
                {**pending, "ticket": ticket},
                anonymous=True,
            )
        token = response.get("token")
        if (
            not isinstance(token, str)
            or not token.startswith("hcri_")
            or response.get("local_robot") != local_robot
            or not response.get("robot_id")
        ):
            raise DataError("PLATFORM_CONNECTION_IDENTITY_MISMATCH")
        credentials = self.root / "credentials"
        if credentials.is_symlink():
            raise DataError("CREDENTIAL_DIRECTORY_INVALID")
        credentials.mkdir(mode=0o700, exist_ok=True)
        os.chmod(credentials, 0o700)
        identity = hashlib.sha256(
            canonical(
                [base, response["robot_id"], response.get("ingest_identity_id")]
            ).encode()
        ).hexdigest()
        path = credentials / (identity + ".token")

        def save_secret():
            temporary = credentials / (str(uuid.uuid4()) + ".tmp")
            descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            try:
                with os.fdopen(descriptor, "w") as stream:
                    stream.write(token)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, path)
                sync_directory(credentials)
            finally:
                temporary.unlink(missing_ok=True)

        await asyncio.to_thread(save_secret)
        config.pop("credential_env")
        config.update(
            credential_file=str(path),
            robot_id=response["robot_id"],
            local_robot=local_robot,
            organization_id=response.get("organization_id"),
            ingest_identity_id=response.get("ingest_identity_id"),
        )
        return await self.configure_platform(config)

    async def platform_status(self):
        if self.platform_config is None:
            return {
                "connected": False,
                "configured": False,
                "reason": "PLATFORM_NOT_CONFIGURED",
            }
        public = {
            "configured": True,
            "robot_id": self.platform_config.get("robot_id"),
            "platform_base_url": self.platform_config["platform_base_url"],
            "organization_id": self.platform_config.get("organization_id"),
        }
        try:
            capabilities = await PlatformAdapter(self.platform_config).capabilities()
            return {
                **public,
                "connected": True,
                "robot_id": capabilities.get("robot_id", public["robot_id"]),
                "capabilities": capabilities,
            }
        except DataError as error:
            return {**public, "connected": False, "error": error.as_dict()}

    async def platform_tasks(self, query=None):
        if self.platform_config is None:
            raise DataError("PLATFORM_NOT_CONFIGURED")
        return await PlatformAdapter(self.platform_config).tasks(query)

    async def platform_preflight(self, task_id, source_format, local_robot):
        if self.platform_config is None:
            raise DataError("PLATFORM_NOT_CONFIGURED")
        return await PlatformAdapter(self.platform_config).preflight(
            task_id, source_format, local_robot
        )

    async def platform_prepare_recording(self, task_id, command):
        if self.platform_config is None:
            raise DataError("PLATFORM_NOT_CONFIGURED")
        if (
            not isinstance(command, dict)
            or not command.get("recording_id")
            or not command.get("expected_plan_revision")
            or not command.get("source_format")
        ):
            raise DataError("CAPTURE_PREPARE_COMMAND_INVALID")
        key = "capture-prepare:" + command["recording_id"]
        frozen = {
            "task_id": task_id,
            "command": command,
            "platform": self.platform_config,
        }

        def freeze(db):
            prior = db.execute(
                "SELECT value FROM settings WHERE key=?", (key,)
            ).fetchone()
            if prior and json.loads(prior[0]) != frozen:
                raise DataError("CAPTURE_PREPARE_CONFLICT")
            db.execute(
                "INSERT OR IGNORE INTO settings VALUES(?,?)", (key, canonical(frozen))
            )

        await self.store.run(freeze)
        return await PlatformAdapter(self.platform_config).prepare_recording(
            task_id, command
        )

    async def refresh_upload(self, job_id):
        job = await self.get_job(job_id)
        if job["kind"] != "upload" or "upload_manifest" not in job["result"]:
            raise DataError("UPLOAD_NOT_SUBMITTED")
        result = await PlatformAdapter(job["params"]["platform_config"]).refresh(
            job["result"]["upload_manifest"], job["result"]
        )
        await self.store.run(
            lambda db: db.execute(
                "UPDATE jobs SET result=?,updated=? WHERE id=?",
                (canonical(result), time.time(), job_id),
            )
        )
        return await self.get_job(job_id)
