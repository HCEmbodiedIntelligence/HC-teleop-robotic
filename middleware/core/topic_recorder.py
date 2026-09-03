from __future__ import annotations

import json
import multiprocessing as mp
import os
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from mcap.well_known import MessageEncoding, SchemaEncoding
from mcap.writer import Writer as McapWriter


_PRIMITIVE_TYPES = {
    "bool", "byte", "char", "float32", "float64",
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "string", "wstring",
}


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
) -> None:
    """Write MCAP in a process isolated from WebRTC and the dashboard."""
    _lower_recording_priority()
    schemas: dict[str, int] = {}
    channels: dict[tuple[str, str], int] = {}
    try:
        with Path(path).open("wb") as stream:
            writer = McapWriter(stream)
            writer.start(profile="ros2")
            # Do not report a recording as active until the destination file
            # has actually been opened and the MCAP header has been written.
            ready_event.set()
            try:
                while True:
                    event = event_queue.get()
                    if event is None:
                        break

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
                            payload = event
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
                    _counter_add(messages)
            finally:
                writer.finish()
    except Exception as exc:
        ready_event.set()
        try:
            error_queue.put_nowait(f"{type(exc).__name__}: {exc}")
        except queue.Full:
            pass


def _inspect_mcap_file(path: Path) -> dict[str, Any]:
    try:
        from mcap.reader import make_reader

        with path.open("rb") as f:
            reader = make_reader(f)
            s = reader.get_summary()
            if not s or not s.statistics:
                return {
                    "duration_sec": 0.0,
                    "duration_human": "--",
                    "message_count": 0,
                    "topic_count": 0,
                    "channels": [],
                    "avg_rate_hz": 0.0,
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
            }
    except Exception:
        return {
            "duration_sec": 0.0,
            "duration_human": "--",
            "message_count": 0,
            "topic_count": 0,
            "channels": [],
            "avg_rate_hz": 0.0,
        }


class TopicRecorder:
    """Process-isolated MCAP recorder for ROS 2 CDR messages and telemetry."""

    def __init__(self, config: dict[str, Any], config_dir: Path):
        directory = Path(str(config.get("directory", "runtime/topic_recordings"))).expanduser()
        if not directory.is_absolute():
            directory = config_dir / directory
        self.directory = directory.resolve()
        self.path: Path | None = None
        self._mp = mp.get_context("spawn")
        self._queue = self._mp.Queue(maxsize=8192)
        self._accepting_event = self._mp.Event()
        self._messages = self._mp.Value("Q", 0)
        self._dropped = self._mp.Value("Q", 0)
        self._error_queue = self._mp.Queue(maxsize=4)
        self._ready_event = self._mp.Event()
        self._process: Any = None
        self._lock = threading.Lock()
        self._last_error: str | None = None

    def ipc_resources(self) -> tuple[Any, Any, Any]:
        """Resources inherited by the raw ROS subscription process."""
        return self._queue, self._accepting_event, self._dropped

    def _drain_queue(self, target: Any) -> None:
        while True:
            try:
                target.get_nowait()
            except queue.Empty:
                return

    def start(self, filename: str = "") -> str:
        self.stop()
        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = filename.strip() if filename else f"ros2_{stamp}.mcap"
        if not fname.endswith(".mcap"):
            fname += ".mcap"
        self.path = self.directory / fname
        with self._lock:
            self._drain_queue(self._queue)
            self._drain_queue(self._error_queue)
            self._ready_event.clear()
            self._last_error = None
            _counter_set(self._messages, 0)
            _counter_set(self._dropped, 0)
            self._process = self._mp.Process(
                target=_mcap_writer_process,
                args=(
                    str(self.path),
                    self._queue,
                    self._messages,
                    self._error_queue,
                    self._ready_event,
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
            self._accepting_event.set()
        return str(self.path)

    def stop(self) -> dict[str, Any]:
        process = self._process
        if process is None:
            return self.status()
        self._accepting_event.clear()
        while True:
            try:
                self._queue.put_nowait(None)
                break
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    continue
                _counter_add(self._dropped)
        process.join(timeout=30.0)
        if process.is_alive():
            raise TimeoutError("MCAP writer did not finish within 30 seconds")
        self._process = None
        return self.status()

    def is_recording(self) -> bool:
        process = self._process
        return self._accepting_event.is_set() and (
            process is None or process.is_alive()
        )

    def record(self, event: dict[str, Any]) -> bool:
        if not self._accepting_event.is_set():
            return False
        try:
            self._queue.put_nowait(event)
            return True
        except queue.Full:
            _counter_add(self._dropped)
            return False

    def status(self) -> dict[str, Any]:
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
                "messages": _counter_get(self._messages),
                "dropped": _counter_get(self._dropped),
                "writer_pid": process.pid if process is not None else None,
                "writer_priority": "low",
                "error": self._last_error,
            }

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
            }
            mark = self._read_mark(p)

            files.append({
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
                "marked": bool(mark.get("marked", False)),
                "marked_at": str(mark.get("marked_at", "")),
                "mark_source": str(mark.get("mark_source", "")),
            })
        files.sort(key=lambda x: x["timestamp"], reverse=True)
        return files

    def delete_recording(self, filename: str) -> None:
        filename = Path(filename).name
        target = (self.directory / filename).resolve()
        if not target.is_relative_to(self.directory) or not target.is_file():
            raise FileNotFoundError(f"recording not found: {filename}")
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
