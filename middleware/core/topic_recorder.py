from __future__ import annotations

import json
import multiprocessing as mp
import os
import queue
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from mcap.well_known import MessageEncoding, SchemaEncoding
from mcap.writer import Writer as McapWriter
from .background_process import background_environment


_PRIMITIVE_TYPES = {
    "bool", "byte", "char", "float32", "float64",
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "string", "wstring",
}

RECORDING_METADATA_NAME = "hc_teleop.session"


def _lower_recording_priority() -> None:
    """Best-effort process priority reduction for recording workers."""
    try:
        if os.name == "posix":
            os.nice(10)
        elif os.name == "nt":
            import ctypes

            below_normal_priority_class = 0x00004000
            ctypes.windll.kernel32.SetPriorityClass(
                ctypes.windll.kernel32.GetCurrentProcess(),
                below_normal_priority_class,
            )
    except Exception:
        pass


def _counter_get(counter: Any) -> int:
    with counter.get_lock():
        return int(counter.value)


def _counter_set(counter: Any, value: int) -> None:
    with counter.get_lock():
        counter.value = int(value)


def _counter_add(counter: Any, value: int = 1) -> None:
    with counter.get_lock():
        counter.value += int(value)


def enqueue_record(resources, event):
    """Bound both message count and payload bytes under the ingress barrier."""
    target, accepting, dropped, lock, accepted, queued_bytes, budget = resources
    raw = event.get('_raw')
    size = len(raw)+1024 if isinstance(raw, bytes) else len(json.dumps(event, ensure_ascii=False).encode())+1024
    with lock:
        if not accepting.is_set():
            return False
        if _counter_get(queued_bytes)+size > budget:
            _counter_add(dropped)
            return False
        try:
            target.put_nowait({**event, '_queue_bytes': size})
        except queue.Full:
            _counter_add(dropped)
            return False
        _counter_add(queued_bytes, size)
        _counter_add(accepted)
        return True


def _get_msg_def(msg_type: str) -> bytes:
    """Recursively resolve ROS 2 message definition and all nested type definitions."""
    try:
        import re
        import ament_index_python

        parts = msg_type.strip().split("/")
        if len(parts) == 3 and parts[1] == "msg":
            top_pkg, _, top_name = parts
        elif len(parts) == 2:
            top_pkg, top_name = parts
        else:
            return b""

        visited = set()

        def get_file(pkg: str, name: str) -> tuple[str, str] | None:
            try:
                share_dir = Path(ament_index_python.get_package_share_directory(pkg))
                msg_file = share_dir / "msg" / f"{name}.msg"
                if msg_file.is_file():
                    return f"{pkg}/msg/{name}", msg_file.read_text(encoding="utf-8")
            except Exception:
                pass
            return None

        def find_dependencies(text: str, current_pkg: str) -> list[tuple[str, str]]:
            deps = []
            for line in text.splitlines():
                line = line.split("#")[0].strip()
                if not line or "=" in line:
                    continue
                field_type = line.split()[0]
                base_type = re.sub(r"\[.*?\]", "", field_type)
                if base_type in _PRIMITIVE_TYPES or base_type.startswith("string<") or base_type.startswith("wstring<"):
                    continue
                type_parts = base_type.split("/")
                if len(type_parts) == 3 and type_parts[1] == "msg":
                    dep_pkg, dep_name = type_parts[0], type_parts[2]
                elif len(type_parts) == 2:
                    dep_pkg, dep_name = type_parts[0], type_parts[1]
                elif len(type_parts) == 1:
                    dep_pkg, dep_name = current_pkg, type_parts[0]
                else:
                    continue
                deps.append((dep_pkg, dep_name))
            return deps

        top_res = get_file(top_pkg, top_name)
        if not top_res:
            return b""
        _, top_text = top_res
        visited.add((top_pkg, top_name))

        queue = find_dependencies(top_text, top_pkg)
        sub_defs = []

        while queue:
            dep_pkg, dep_name = queue.pop(0)
            key = (dep_pkg, dep_name)
            if key in visited:
                continue
            visited.add(key)
            res = get_file(dep_pkg, dep_name)
            if not res:
                continue
            _, text = res
            sub_defs.append((dep_pkg, dep_name, text))
            queue.extend(find_dependencies(text, dep_pkg))

        out = [top_text.rstrip()]
        sep = "=" * 80
        for dep_pkg, dep_name, text in sub_defs:
            out.append(f"\n{sep}\nMSG: {dep_pkg}/{dep_name}\n{text.rstrip()}")

        return "\n".join(out).encode("utf-8")
    except Exception:
        return b""


def _mcap_writer_process(
    path: str,
    event_queue: Any,
    messages: Any,
    error_queue: Any,
    ready_event: Any,
    metadata: dict[str, str],
    finish_event: Any,
    accepted: Any,
    queued_bytes: Any,
    ingress_lock: Any,
) -> None:
    """Write MCAP in a process isolated from WebRTC and the dashboard."""
    _lower_recording_priority()
    schemas: dict[str, int] = {}
    channels: dict[tuple[str, str], int] = {}
    try:
        with Path(path).open("wb") as stream:
            writer = McapWriter(stream)
            writer.start(profile="ros2")
            if metadata:
                writer.add_metadata(RECORDING_METADATA_NAME, metadata)
            # Do not report a recording as active until the destination file
            # has actually been opened and the MCAP header has been written.
            ready_event.set()
            try:
                while True:
                    if finish_event.is_set() and _counter_get(messages) >= _counter_get(accepted):
                        break
                    try:
                        event = event_queue.get(timeout=.1)
                    except queue.Empty:
                        continue

                    topic = str(event.get("topic") or "/events")
                    msg_type = str(
                        event.get("msg_type") or "std_msgs/msg/String"
                    )
                    raw_data = event.get("_raw")
                    stamp_ns = int(event.get("stamp_ns") or time.time_ns())

                    if isinstance(raw_data, bytes):
                        if msg_type not in schemas:
                            schemas[msg_type] = writer.register_schema(
                                name=msg_type,
                                encoding=SchemaEncoding.ROS2,
                                data=_get_msg_def(msg_type),
                            )
                        schema_id = schemas[msg_type]
                        chan_key = (topic, msg_type)
                        if chan_key not in channels:
                            channels[chan_key] = writer.register_channel(
                                topic=topic,
                                message_encoding=MessageEncoding.CDR,
                                schema_id=schema_id,
                            )
                        writer.add_message(
                            channel_id=channels[chan_key],
                            log_time=stamp_ns,
                            data=raw_data,
                            publish_time=stamp_ns,
                        )
                    else:
                        payload = event.get("payload")
                        if payload is None:
                            payload = {k:v for k,v in event.items() if not k.startswith('_')}
                        json_bytes = json.dumps(
                            payload,
                            ensure_ascii=False,
                            separators=(",", ":"),
                        ).encode("utf-8")
                        if "teleop_event" not in schemas:
                            schemas["teleop_event"] = writer.register_schema(
                                name="teleop_event",
                                encoding=SchemaEncoding.JSONSchema,
                                data=b"",
                            )
                        schema_id = schemas["teleop_event"]
                        chan_key = (topic, "teleop_event")
                        if chan_key not in channels:
                            channels[chan_key] = writer.register_channel(
                                topic=topic,
                                message_encoding=MessageEncoding.JSON,
                                schema_id=schema_id,
                            )
                        writer.add_message(
                            channel_id=channels[chan_key],
                            log_time=stamp_ns,
                            data=json_bytes,
                            publish_time=stamp_ns,
                        )
                    provenance = event.get('_provenance')
                    if provenance:
                        key = ('/events/recording/sample_provenance', 'teleop_event')
                        if key not in channels:
                            channels[key] = writer.register_channel(topic=key[0], message_encoding=MessageEncoding.JSON,
                                schema_id=writer.register_schema(name='hc-sample-provenance/v1', encoding=SchemaEncoding.JSONSchema, data=b''))
                        writer.add_message(channel_id=channels[key], log_time=stamp_ns, publish_time=stamp_ns,
                                           data=json.dumps(provenance, allow_nan=False).encode())
                    with ingress_lock:
                        _counter_add(messages)
                        _counter_add(queued_bytes, -int(event.get('_queue_bytes', 0)))
            finally:
                writer.finish()
    except Exception as exc:
        ready_event.set()
        try:
            error_queue.put_nowait(f"{type(exc).__name__}: {exc}")
        except queue.Full:
            pass


def _reusable_writer_process(control, event_queue, messages, errors, ready, finish,
                             accepted, queued_bytes, ingress_lock, finished):
    """Prepared correction slots reuse a warm writer process between files."""
    while True:
        request = control.get()
        if request is None:
            return
        path, metadata = request
        _mcap_writer_process(path, event_queue, messages, errors, ready, metadata,
                             finish, accepted, queued_bytes, ingress_lock)
        finished.set()


def read_recording_metadata(path: Path) -> dict[str, str]:
    """Read HC Teleop session metadata; legacy/external MCAP files return {}."""
    try:
        from mcap.reader import make_reader

        with path.open("rb") as stream:
            reader = make_reader(stream)
            for record in reader.iter_metadata():
                if record.name == RECORDING_METADATA_NAME:
                    return dict(record.metadata)
    except Exception:
        pass
    return {}


def _metadata_fields(metadata: dict[str, str]) -> dict[str, Any]:
    free_joints: list[str] = []
    try:
        decoded = json.loads(metadata.get("free_joints", "[]"))
        if isinstance(decoded, list):
            free_joints = [str(value) for value in decoded]
    except (TypeError, ValueError, json.JSONDecodeError):
        pass
    return {
        "profile_id": str(metadata.get("profile_id", "")),
        "profile_display_name": str(metadata.get("profile_display_name", "")),
        "robot_name": str(metadata.get("robot_name", "")),
        "controller_config": str(metadata.get("controller_config", "")),
        "free_joints": free_joints,
    }


def _inspect_mcap_file(path: Path) -> dict[str, Any]:
    try:
        from mcap.reader import make_reader

        with path.open("rb") as f:
            reader = make_reader(f)
            session_metadata = {}
            for record in reader.iter_metadata():
                if record.name == RECORDING_METADATA_NAME:
                    session_metadata = dict(record.metadata)
                    break
            s = reader.get_summary()
            if not s or not s.statistics:
                return {
                    "duration_sec": 0.0,
                    "duration_human": "--",
                    "message_count": 0,
                    "topic_count": 0,
                    "channels": [],
                    "avg_rate_hz": 0.0,
                    **_metadata_fields(session_metadata),
                }
            start_ns = s.statistics.message_start_time
            end_ns = s.statistics.message_end_time
            duration = (end_ns - start_ns) / 1e9 if (end_ns > start_ns) else 0.0
            msg_count = s.statistics.message_count
            schemas = {s_id: sch.name for s_id, sch in s.schemas.items()}
            channels = []
            for chan_id, count in sorted(s.statistics.channel_message_counts.items()):
                chan = s.channels.get(chan_id)
                if not chan:
                    continue
                channels.append({
                    "topic": chan.topic,
                    "type": schemas.get(chan.schema_id, "unknown"),
                    "count": count,
                    "rate_hz": round((count / duration) if duration > 0 else 0.0, 1),
                })
            avg_rate = (msg_count / duration) if duration > 0 else 0.0
            return {
                "duration_sec": round(duration, 2),
                "duration_human": f"{duration:.1f}s" if duration < 60 else f"{int(duration // 60)}m {int(duration % 60)}s",
                "message_count": msg_count,
                "topic_count": len(channels),
                "channels": channels,
                "avg_rate_hz": round(avg_rate, 1),
                **_metadata_fields(session_metadata),
            }
    except Exception:
        return {
            "duration_sec": 0.0,
            "duration_human": "--",
            "message_count": 0,
            "topic_count": 0,
            "channels": [],
            "avg_rate_hz": 0.0,
            **_metadata_fields({}),
        }


from .correction_support import CorrectionSupport


class TopicRecorder(CorrectionSupport):
    """Process-isolated MCAP recorder for ROS 2 CDR messages and telemetry."""

    def __init__(self, config: dict[str, Any], config_dir: Path):
        self.config, self.config_dir = dict(config), config_dir
        directory = Path(str(config.get("directory", "runtime/topic_recordings"))).expanduser()
        if not directory.is_absolute():
            directory = config_dir / directory
        self.directory = directory.resolve()
        self.action_sources = config.get("action_sources", [])
        self.save_format = str(config.get("format", "mcap"))
        self.lerobot_fps = int(config.get("lerobot_fps", 15))
        self.lerobot_task = str(config.get("lerobot_task", "遥操作"))
        self._export_process = None
        self._export_state = "idle"
        self._export_error = None
        self._export_log = None
        self._lerobot_path = ""
        self.path: Path | None = None
        self._mp = mp.get_context("spawn")
        self._queue = self._mp.Queue(maxsize=int(config.get('queue_capacity', 8192)))
        self._ingress_lock = self._mp.RLock()
        self._accepted = self._mp.Value('Q', 0)
        self._queued_bytes = self._mp.Value('Q', 0)
        self._byte_budget = int(config.get('queue_bytes', 128*1024**2))
        self._finish_event = self._mp.Event()
        self._reusable = bool(config.get('reuse_writer_process', False))
        self._writer_control = self._mp.Queue(maxsize=1)
        self._finished_event = self._mp.Event()
        self._accepting_event = self._mp.Event()
        self._messages = self._mp.Value("Q", 0)
        self._dropped = self._mp.Value("Q", 0)
        self._error_queue = self._mp.Queue(maxsize=4)
        self._ready_event = self._mp.Event()
        self._process: Any = None
        self._lock = threading.Lock()
        self._last_error: str | None = None
        self._active_metadata: dict[str, str] = {}
        from .recording_artifacts import RecordingArtifacts
        self._artifacts = RecordingArtifacts(config, config_dir, self.directory)
        self.artifact_state_dir = self._artifacts.state_dir
        self.binding_snapshot = None
        self.upload_service = None
        self._capture_identity = None
        self.task_managed = False

    def ipc_resources(self) -> tuple:
        """Resources inherited by the raw ROS subscription process."""
        return (self._queue, self._accepting_event, self._dropped, self._ingress_lock,
                self._accepted, self._queued_bytes, self._byte_budget)

    def _drain_queue(self, target: Any) -> None:
        while True:
            try:
                target.get_nowait()
            except queue.Empty:
                return

    def start(
        self,
        filename: str = "",
        metadata: dict[str, Any] | None = None,
    ) -> str:
        if self.corrections:
            raise ValueError('correction capture owns recording; exit the intervention session first')
        self.stop()
        if self.status()["export_state"] == "exporting":
            self._detach_export()
        if self.save_format in ("lerobot", "both"):
            from .lerobot_export import ensure_export_available
            ensure_export_available()
        if self.save_format == "mcap_mp4":
            from .video_bundle import ensure_available
            ensure_available()
        self._export_state, self._export_error, self._lerobot_path = "idle", None, ""
        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = filename.strip() if filename else f"ros2_{stamp}.mcap"
        if not fname.endswith(".mcap"):
            fname += ".mcap"
        # Keep the complete source, including /vrdata, for recovery and audit.
        source_dir = self.directory / ".lerobot_sources" if self.save_format == "lerobot" else self.directory / ".bundle_sources" if self.save_format == "mcap_mp4" else self.directory
        source_dir.mkdir(parents=True, exist_ok=True)
        fname = Path(fname).name
        self.path = source_dir / fname
        if self.path.exists():
            raise FileExistsError(f"录制文件已存在: {self.path}")
        # Freeze the platform task at capture start, never at conversion/upload time.
        binding = self.binding_snapshot() if self.binding_snapshot else None
        self._capture_identity = (None if self.task_managed else
            self._artifacts.begin(self.path, self.save_format, metadata or {}, binding))
        self._active_metadata = {
            str(key): str(value)
            for key, value in (metadata or {}).items()
            if value is not None
        }
        with self._lock:
            self._drain_queue(self._queue)
            self._drain_queue(self._error_queue)
            self._ready_event.clear()
            self._finish_event.clear()
            self._finished_event.clear()
            _counter_set(self._accepted, 0)
            _counter_set(self._queued_bytes, 0)
            self._last_error = None
            _counter_set(self._messages, 0)
            _counter_set(self._dropped, 0)
            if self._reusable:
                if self._process is None or not self._process.is_alive():
                    self._process = self._mp.Process(target=_reusable_writer_process,
                        args=(self._writer_control, self._queue, self._messages, self._error_queue,
                              self._ready_event, self._finish_event, self._accepted, self._queued_bytes,
                              self._ingress_lock, self._finished_event), name='correction-writer', daemon=True)
                    self._process.start()
                self._writer_control.put_nowait((str(self.path), self._active_metadata))
            else:
                self._process = self._mp.Process(
                target=_mcap_writer_process,
                args=(
                    str(self.path),
                    self._queue,
                    self._messages,
                    self._error_queue,
                    self._ready_event,
                    self._active_metadata,
                    self._finish_event,
                    self._accepted,
                    self._queued_bytes,
                    self._ingress_lock,
                ),
                name="mcap-writer",
                daemon=True,
                )
                self._process.start()
            if not self._ready_event.wait(timeout=5.0):
                self._process.terminate()
                self._process.join(timeout=2.0)
                self._process = None
                raise TimeoutError("MCAP writer did not become ready within 5 seconds")
            if not self._process.is_alive():
                try:
                    self._last_error = self._error_queue.get_nowait()
                except queue.Empty:
                    self._last_error = "MCAP writer exited during startup"
                self._process = None
                raise RuntimeError(self._last_error)
            self._artifacts.mark_started(self._capture_identity)
            self._accepting_event.set()
        return str(self.path)

    def stop(self) -> dict[str, Any]:
        process = self._process
        if process is None:
            return self.status()
        if self._reusable and self._finished_event.is_set():
            return self.status()
        capture_ended_at = datetime.now(timezone.utc).isoformat()
        # Separate end signal; accepted counts include packets still in a
        # multiprocessing feeder. A full queue never loses data to make room
        # for a sentinel, nor can Queue.empty() finish before a feeder flushes.
        with self._ingress_lock:
            self._accepting_event.clear()
            self._finish_event.set()
        if self._reusable:
            while not self._finished_event.wait(timeout=.1):
                if not process.is_alive():
                    raise RuntimeError('MCAP slot exited before completing accepted data')
        else:
            process.join()
            self._process = None
        if process.exitcode not in (None, 0):
            self._last_error = f"MCAP writer exited with code {process.exitcode}"
        self.status()  # Drain writer errors before exporting a closed source.
        if process.exitcode not in (None, 0) and not self._last_error:
            self._last_error = f"MCAP writer exited with code {process.exitcode}"
        self._artifacts.close(self._capture_identity, self._last_error, capture_ended_at)
        self._capture_identity = None
        if self.save_format in ("lerobot", "both") and not self._last_error:
            self._start_lerobot_export()
        return self.status()

    def _detach_export(self) -> None:
        """Keep a closed capture converting while the next capture uses the IPC."""
        process, log, destination = self._export_process, self._export_log, Path(self._lerobot_path)
        self._export_process, self._export_log = None, None
        if process is None:
            return  # An independently resumed converter owns its progress file.

        def finish():
            from .export_progress import ExportProgress, progress_path
            try:
                code = process.wait()
                success = code == 0 and (destination / 'meta/info.json').is_file()
                ExportProgress(progress_path(destination)).update(
                    '转换完成' if success else '转换失败', 100 if success else 0,
                    state='completed' if success else 'error',
                    error='' if success else 'LeRobot 保存失败，原始 MCAP 已保留', force=True)
            finally:
                if log is not None: log.close()

        threading.Thread(target=finish, name='lerobot-finalize', daemon=True).start()

    def close_writer(self):
        self.stop()
        if self._reusable and self._process:
            self._writer_control.put(None, timeout=1.)
            self._process.join(timeout=5.)
            if self._process.is_alive():
                raise TimeoutError('prepared MCAP writer did not exit')
            self._process = None

    def _start_lerobot_export(self) -> None:
        assert self.path is not None
        destination = self.directory / "lerobot" / self.path.stem
        self._lerobot_path = str(destination)
        self._export_state = "exporting"
        self._export_error = None
        log_path = self.path.with_suffix(".lerobot-export.log")
        from .export_progress import ExportProgress, progress_path
        ExportProgress(progress_path(destination)).update('等待转换', 0, force=True, reset=True)
        self._export_log = log_path.open("w", encoding="utf-8")
        fps, task = self.lerobot_fps, self.lerobot_task
        from .capture_formats import lerobot_snapshot
        snapshot = lerobot_snapshot(self._active_metadata)
        if snapshot:
            from humanoid_upload.capture_plan import validate_snapshot
            snapshot = validate_snapshot(snapshot, source_format='LEROBOT_V3')
            task = json.loads(self._active_metadata['platform_task'])['task']['name']
        command = [sys.executable, "-m", "middleware.core.lerobot_export",
                   str(self.path), str(destination), "--fps", str(fps),
                   "--task", task, "--robot-type",
                   self._active_metadata.get("profile_id", "robot"), "--action-sources", json.dumps(self.action_sources)]
        try:
            # Export runs outside the ROS and video workers at lower priority.
            if sys.platform.startswith("linux"):
                command = ["nice", "-n", "10", *command]
            self._export_process = subprocess.Popen(
                command, cwd=Path(__file__).resolve().parents[2],
                stdout=self._export_log, stderr=subprocess.STDOUT,
                env=background_environment(),
            )
        except Exception as exc:
            self._export_log.close()
            self._export_log = None
            self._export_state, self._export_error = "error", str(exc)
            ExportProgress(progress_path(destination)).update('转换失败', 0, state='error', error=str(exc), force=True)

    def _poll_export(self) -> None:
        process = self._export_process
        if process is None:
            # A preserved source can be retried by the export CLI. Its progress
            # must replace the cached failure from the original managed worker.
            progress = self.export_progress()
            state = progress.get('state')
            if state == 'completed' and (Path(self._lerobot_path) / 'meta/info.json').is_file():
                self._export_state, self._export_error = 'completed', None
            elif state == 'exporting':
                self._export_state, self._export_error = 'exporting', None
            elif state == 'error':
                self._export_state, self._export_error = 'error', progress.get('error')
            return
        if process.poll() is None:
            return
        self._export_process = None
        if self._export_log is not None:
            log_path = Path(self._export_log.name)
            self._export_log.close()
            self._export_log = None
        else:
            log_path = self.path.with_suffix(".lerobot-export.log")
        if process.returncode == 0 and (Path(self._lerobot_path) / "meta" / "info.json").is_file():
            self._export_state = "completed"
            from .export_progress import ExportProgress, progress_path
            ExportProgress(progress_path(Path(self._lerobot_path))).update('转换完成', 100, state='completed', force=True)
        else:
            self._export_state = "error"
            detail = log_path.read_text(encoding="utf-8", errors="replace")[-2000:] if log_path.exists() else ""
            self._export_error = f"LeRobot 保存失败，原始 MCAP 已保留。{detail}"
            from .export_progress import ExportProgress, progress_path
            ExportProgress(progress_path(Path(self._lerobot_path))).update('转换失败', 0, state='error', error=self._export_error, force=True)

    def is_recording(self) -> bool:
        process = self._process
        return self._accepting_event.is_set() and (
            process is None or process.is_alive()
        )

    def record(self, event: dict[str, Any]) -> bool:
        return enqueue_record(self.ipc_resources(), event)

    def status(self) -> dict[str, Any]:
        self._poll_export()
        while True:
            try:
                self._last_error = self._error_queue.get_nowait()
            except queue.Empty:
                break
        process = self._process
        with self._lock:
            active = self._accepting_event.is_set() and process is not None and process.is_alive()
            return {
                "recording": active,
                "path": str(self.path) if self.path else "",
                "active_file": self.path.name if self.path and active else "",
                "corrections": self.corrections.status() if self.corrections else None,
                "messages": _counter_get(self._messages),
                "accepted": _counter_get(self._accepted),
                "queued_bytes": _counter_get(self._queued_bytes),
                "queue_byte_budget": self._byte_budget,
                "dropped": _counter_get(self._dropped),
                "writer_pid": process.pid if process is not None else None,
                "writer_priority": "low",
                "upload_handoff_errors": dict(self._artifacts.recovery_errors),
                "format": self.save_format,
                "export_state": self._export_state,
                "export_progress": self.export_progress(),
                "export_error": self._export_error,
                "lerobot_path": self._lerobot_path,
                "error": self._last_error,
            }

    def export_progress(self) -> dict:
        if not self._lerobot_path:
            return {}
        from .export_progress import read_progress, progress_path
        return read_progress(progress_path(Path(self._lerobot_path)))

    def _mark_path(self, recording_path: Path) -> Path:
        return self.directory / f".{recording_path.name}.mark.json"

    def _read_mark(self, recording_path: Path) -> dict[str, Any]:
        mark_path = self._mark_path(recording_path)
        try:
            value = json.loads(mark_path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError, TypeError):
            return {}

    def mark_current_or_latest(self, source: str = "manual") -> dict[str, Any]:
        """Mark the active recording, or the latest completed recording."""
        self.directory.mkdir(parents=True, exist_ok=True)
        active = self.path if self.path and self.is_recording() else None
        candidates = [
            path
            for path in self.directory.iterdir()
            if path.is_file() and path.suffix in {".mcap", ".jsonl"}
        ]
        target = active or (max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None)
        if target is None:
            raise FileNotFoundError("no recording is available to mark")

        marked_at = datetime.now().astimezone().isoformat(timespec="seconds")
        result = {
            "recording": bool(active),
            "filename": target.name,
            "path": str(target.resolve()),
            "marked": True,
            "marked_at": marked_at,
            "mark_source": source,
        }
        mark_path = self._mark_path(target)
        mark_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        if active is not None:
            self.record(
                {
                    "kind": "recording_marker",
                    "topic": "/teleop/recording_marker",
                    "payload": result,
                }
            )
        return result

    def list_recordings(self) -> list[dict[str, Any]]:
        self._poll_export()
        self.directory.mkdir(parents=True, exist_ok=True)
        files = []
        active_path = (
            self.path.resolve()
            if (self.path and self.is_recording())
            else None
        )

        for p in self.directory.iterdir():
            if not p.is_file() or p.name.startswith("."):
                continue
            if not (p.suffix in {".mcap", ".jsonl"}):
                continue
            try:
                stat = p.stat()
            except OSError:
                continue
            size = stat.st_size
            is_cur = active_path is not None and p.resolve() == active_path

            if size < 1024:
                human_size = f"{size} B"
            elif size < 1024 * 1024:
                human_size = f"{size / 1024:.1f} KB"
            elif size < 1024 * 1024 * 1024:
                human_size = f"{size / (1024 * 1024):.2f} MB"
            else:
                human_size = f"{size / (1024 * 1024 * 1024):.2f} GB"

            created_iso = datetime.fromtimestamp(stat.st_ctime).strftime("%Y-%m-%d %H:%M:%S")
            modified_iso = datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")

            meta = _inspect_mcap_file(p) if (p.suffix == ".mcap" and not is_cur) else {
                "duration_sec": 0.0,
                "duration_human": "正在写入…" if is_cur else "--",
                "message_count": _counter_get(self._messages) if is_cur else 0,
                "topic_count": 0,
                "channels": [],
                "avg_rate_hz": 0.0,
                **_metadata_fields(self._active_metadata if is_cur else {}),
            }
            mark = self._read_mark(p)

            files.append({
                "id": p.name,
                "filename": p.name,
                "size_bytes": size,
                "size_human": human_size,
                "created_at": created_iso,
                "modified_at": modified_iso,
                "timestamp": stat.st_mtime,
                "is_current": is_cur,
                "format": p.suffix.lstrip(".").upper(),
                "duration_sec": meta["duration_sec"],
                "duration_human": meta["duration_human"],
                "message_count": meta["message_count"],
                "topic_count": meta["topic_count"],
                "channels": meta["channels"],
                "avg_rate_hz": meta["avg_rate_hz"],
                "profile_id": meta["profile_id"],
                "profile_display_name": meta["profile_display_name"],
                "robot_name": meta["robot_name"],
                "controller_config": meta["controller_config"],
                "free_joints": meta["free_joints"],
                "marked": bool(mark.get("marked", False)),
                "marked_at": str(mark.get("marked_at", "")),
                "mark_source": str(mark.get("mark_source", "")),
            })
        from .recording_datasets import inspect_dataset
        datasets_dir = self.directory / "lerobot"
        if datasets_dir.is_dir() and not datasets_dir.is_symlink():
            for dataset in datasets_dir.iterdir():
                if dataset.is_dir() and not dataset.is_symlink() and not dataset.name.startswith("."):
                    try:
                        files.append(inspect_dataset(dataset))
                    except (OSError, ValueError, KeyError, TypeError):
                        continue  # Incomplete/invalid exports are not ready datasets.
            from .recording_datasets import pending_datasets
            files.extend(pending_datasets(datasets_dir, {item.get('id') for item in files}))
        from .recording_datasets import inspect_bundle
        bundles_dir = self.directory / "bundles"
        if bundles_dir.is_dir() and not bundles_dir.is_symlink():
            for bundle in bundles_dir.iterdir():
                if bundle.is_dir() and not bundle.is_symlink() and not bundle.name.startswith('.'):
                    try:
                        files.append(inspect_bundle(bundle))
                    except (OSError, ValueError, KeyError, TypeError):
                        continue
        files.sort(key=lambda x: x["timestamp"], reverse=True)
        return self._artifacts.annotate(files)

    def shutdown(self):
        if self.corrections: self.stop_corrections()
        try:
            return self.stop()
        finally:
            self._artifacts.shutdown()

    def delete_recording(self, filename: str) -> None:
        from .recording_datasets import resolve_dataset
        target = resolve_dataset(self.directory, filename)
        if self.is_recording() and self.path and target == self.path.resolve():
            raise ValueError("cannot delete actively recording file")
        if self._export_state == "exporting" and (
            target == Path(self._lerobot_path).resolve() or
            (self.path and target == self.path.resolve())
        ):
            raise ValueError("cannot delete source or dataset while exporting")
        with self._artifacts.deleting(target):
            self._delete_recording(filename, target)

    def _delete_recording(self, filename: str, target: Path) -> None:
        if filename.startswith(("lerobot:", "bundle:")):
            from .export_progress import read_progress, progress_path
            if read_progress(progress_path(target)).get('state') == 'exporting':
                raise ValueError("cannot delete dataset while exporting")
            from .recording_datasets import dataset_files
            dataset_files(target)  # Refuse symlink-containing trees.
            import shutil
            shutil.rmtree(target)
            return
        active_path = (
            self.path.resolve()
            if (self.path and self.is_recording())
            else None
        )
        if active_path is not None and target == active_path:
            raise ValueError(f"cannot delete actively recording file: {filename}")
        target.unlink()
        try:
            self._mark_path(target).unlink()
        except FileNotFoundError:
            pass
