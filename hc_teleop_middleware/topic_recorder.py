from __future__ import annotations

import json
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
        for dep_pkg, dep_name, text in sub_defs:
            out.append(f"\n================================================================================\nMSG: {dep_pkg}/msg/{dep_name}\n{text.rstrip()}")

        return "\n".join(out).encode("utf-8")
    except Exception:
        return b""


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
    """Non-blocking MCAP recorder for ROS 2 CDR messages and telemetry."""

    def __init__(self, config: dict[str, Any], config_dir: Path):
        directory = Path(str(config.get("directory", "runtime/topic_recordings"))).expanduser()
        if not directory.is_absolute():
            directory = config_dir / directory
        self.directory = directory.resolve()
        self.path: Path | None = None
        self._queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=8192)
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._messages = 0
        self._dropped = 0

    def start(self, filename: str = "") -> str:
        self.stop()
        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = filename.strip() if filename else f"ros2_{stamp}.mcap"
        if not fname.endswith(".mcap"):
            fname += ".mcap"
        self.path = self.directory / fname
        with self._lock:
            self._messages = 0
            self._dropped = 0
            self._queue = queue.Queue(maxsize=8192)
        self._thread = threading.Thread(
            target=self._run, name="topic-recorder", daemon=True
        )
        self._thread.start()
        return str(self.path)

    def stop(self) -> dict[str, Any]:
        if self._thread is None:
            return self.status()
        while True:
            try:
                self._queue.put_nowait(None)
                break
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    continue
                with self._lock:
                    self._dropped += 1
        self._thread.join(timeout=3.0)
        self._thread = None
        return self.status()

    def record(self, event: dict[str, Any]) -> None:
        if self._thread is None:
            return
        try:
            self._queue.put_nowait(event)
        except queue.Full:
            with self._lock:
                self._dropped += 1

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "recording": self._thread is not None and self._thread.is_alive(),
                "path": str(self.path) if (self.path and self._thread is not None) else (str(self.path) if self.path else ""),
                "messages": self._messages,
                "dropped": self._dropped,
            }

    def list_recordings(self) -> list[dict[str, Any]]:
        self.directory.mkdir(parents=True, exist_ok=True)
        files = []
        active_path = (
            self.path.resolve()
            if (self.path and self._thread is not None and self._thread.is_alive())
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
                "message_count": self._messages if is_cur else 0,
                "topic_count": 0,
                "channels": [],
                "avg_rate_hz": 0.0,
            }

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
            if (self.path and self._thread is not None and self._thread.is_alive())
            else None
        )
        if active_path is not None and target == active_path:
            raise ValueError(f"cannot delete actively recording file: {filename}")
        target.unlink()

    def _run(self) -> None:
        assert self.path is not None
        schemas: dict[str, int] = {}
        channels: dict[tuple[str, str], int] = {}

        with self.path.open("wb") as stream:
            writer = McapWriter(stream)
            writer.start(profile="ros2")
            try:
                while True:
                    try:
                        event = self._queue.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    if event is None:
                        break

                    topic = str(event.get("topic") or "/events")
                    msg_type = str(event.get("msg_type") or "std_msgs/msg/String")
                    raw_data = event.get("_raw")
                    stamp_ns = int(event.get("stamp_ns") or time.time_ns())

                    if raw_data is not None and isinstance(raw_data, (bytes, bytearray)):
                        if msg_type not in schemas:
                            msg_def = _get_msg_def(msg_type)
                            schemas[msg_type] = writer.register_schema(
                                name=msg_type,
                                encoding=SchemaEncoding.ROS2,
                                data=msg_def,
                            )
                        schema_id = schemas[msg_type]
                        chan_key = (topic, msg_type)
                        if chan_key not in channels:
                            channels[chan_key] = writer.register_channel(
                                topic=topic,
                                message_encoding=MessageEncoding.CDR,
                                schema_id=schema_id,
                            )
                        channel_id = channels[chan_key]
                        writer.add_message(
                            channel_id=channel_id,
                            log_time=stamp_ns,
                            data=bytes(raw_data),
                            publish_time=stamp_ns,
                        )
                    else:
                        payload = event.get("payload")
                        if payload is None:
                            payload = event
                        json_bytes = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
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
                        channel_id = channels[chan_key]
                        writer.add_message(
                            channel_id=channel_id,
                            log_time=stamp_ns,
                            data=json_bytes,
                            publish_time=stamp_ns,
                        )

                    with self._lock:
                        self._messages += 1
            finally:
                writer.finish()
