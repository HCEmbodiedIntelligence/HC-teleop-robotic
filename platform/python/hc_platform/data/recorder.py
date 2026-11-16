"""Bounded MCAP ingestion with all disk work isolated in a child process."""

import asyncio
import importlib.util
import json
import multiprocessing as mp
import os
import queue
import time
from pathlib import Path

from .errors import DataError


def available():
    return importlib.util.find_spec("mcap") is not None


def _writer(path, messages, stop, pending, written_counter, error_counter, status):
    counts, channels, schemas = {}, {}, {}
    written = 0
    first_ns = last_ns = None
    provenance = None
    try:
        from mcap.writer import CompressionType, Writer

        with open(path, "wb") as stream:
            writer = Writer(stream, compression=CompressionType.NONE, enable_crcs=True)
            writer.start(profile="ros2", library="hc-platform/1")
            status.send({"state": "ready"})
            while True:
                try:
                    item = messages.get(timeout=0.05)
                except queue.Empty:
                    with pending.get_lock():
                        empty = pending.value == 0
                    if stop.is_set() and empty:
                        break
                    continue
                (
                    topic,
                    body,
                    log_ns,
                    source_ns,
                    ros_type,
                    ros_schema,
                    budget_bytes,
                    metadata,
                ) = item
                if topic not in channels:
                    schema = writer.register_schema(
                        ros_type, "ros2msg", ros_schema.encode()
                    )
                    schemas[topic] = (ros_type, ros_schema)
                    channels[topic] = writer.register_channel(topic, "cdr", schema)
                if schemas[topic][0] != ros_type or (
                    ros_schema and schemas[topic][1] != ros_schema
                ):
                    raise ValueError("TOPIC_SCHEMA_CHANGED")
                sequence = counts.get(topic, 0)
                writer.add_message(
                    channels[topic], log_ns, body, source_ns, sequence=sequence
                )
                if metadata is not None:
                    if provenance is None:
                        provenance = Path(path).with_suffix(".samples.jsonl").open("w")
                    provenance.write(
                        json.dumps(
                            {
                                **metadata,
                                "topic": topic,
                                "channel_sequence": sequence,
                                "log_time_ns": log_ns,
                                "publish_time_ns": source_ns,
                            },
                            separators=(",", ":"),
                            allow_nan=False,
                        )
                        + "\n"
                    )
                counts[topic] = sequence + 1
                written += 1
                written_counter.value = written
                first_ns = log_ns if first_ns is None else min(first_ns, log_ns)
                last_ns = log_ns if last_ns is None else max(last_ns, log_ns)
                with pending.get_lock():
                    pending.value -= budget_bytes
            writer.finish()
            stream.flush()
            os.fsync(stream.fileno())
            if provenance:
                provenance.flush()
                os.fsync(provenance.fileno())
        status.send(
            {
                "state": "finished",
                "message_count": written,
                "topics": counts,
                "started_ns": first_ns,
                "ended_ns": last_ns,
                "duration_ns": (last_ns - first_ns) if first_ns is not None else 0,
            }
        )
    except BaseException as error:
        error_counter.value += 1
        try:
            status.send(
                {
                    "state": "failed",
                    "error": type(error).__name__,
                    "message_count": written,
                }
            )
        except (BrokenPipeError, OSError):
            pass
    finally:
        if provenance:
            provenance.close()
        status.close()


class RecorderProcess:
    def __init__(self, path, *, max_messages=4096, max_bytes=64 * 1024 * 1024):
        if not available():
            raise DataError("MCAP_UNAVAILABLE", "Install the optional mcap dependency")
        self.context = mp.get_context("spawn")
        self.messages = self.context.Queue(maxsize=max_messages)
        self.stop_event = self.context.Event()
        self.pending = self.context.Value("q", 0)
        self.written_counter = self.context.Value("q", 0)
        self.error_counter = self.context.Value("q", 0)
        self.status, child = self.context.Pipe(duplex=False)
        self.process = self.context.Process(
            target=_writer,
            args=(
                str(path),
                self.messages,
                self.stop_event,
                self.pending,
                self.written_counter,
                self.error_counter,
                child,
            ),
            name="hc-mcap-writer",
            daemon=True,
        )
        self.max_bytes = max_bytes
        self.failure = None
        self.result = None
        self.accepted = 0
        self.dropped = self.local_errors = 0
        self.topic_counts = {}
        self.counter_cache = {"written": 0, "errors": 0, "queue_bytes": 0}
        self.schemas = {}
        self.closed = False
        self.process.start()
        child.close()

    async def ready(self, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.status.poll():
                result = self.status.recv()
                if result.get("state") == "ready":
                    return
                self.failure = result.get("error", "START_FAILED")
                break
            if not self.process.is_alive():
                break
            await asyncio.sleep(0.01)
        await self.abort()
        raise DataError(
            "RECORDER_START_FAILED",
            self.failure or "Writer did not start before deadline",
        )

    def append(
        self,
        topic,
        body,
        log_ns,
        source_ns,
        ros_type="std_msgs/msg/String",
        ros_schema="string data\n",
        *,
        metadata=None,
    ):
        if self.closed or self.failure or not self.process.is_alive():
            self.dropped += 1
            raise DataError("RECORDER_UNAVAILABLE")
        schema = (ros_type, ros_schema)
        previous = self.schemas.get(topic)
        if previous is not None and previous != schema:
            self.failure = "TOPIC_SCHEMA_CHANGED"
            self.dropped += 1
            self.local_errors += 1
            raise DataError(self.failure)
        # A schema can be large; transmit it once, and budget its bytes too.
        transmitted_schema = ros_schema if previous is None else ""
        budget_bytes = (
            len(body)
            + len(transmitted_schema.encode())
            + len(topic.encode())
            + len(ros_type.encode())
            + 128
            + (len(json.dumps(metadata)) if metadata else 0)
        )
        pending_lock = self.pending.get_lock()
        if not pending_lock.acquire(timeout=0.002):
            self.failure = "RECORDER_COUNTER_UNAVAILABLE"
            self.local_errors += 1
            self.dropped += 1
            raise DataError(self.failure)
        try:
            if self.pending.value + budget_bytes > self.max_bytes:
                self.failure = "RECORDING_QUEUE_OVERFLOW"
                self.dropped += 1
                self.local_errors += 1
                raise DataError(self.failure)
            self.pending.value += budget_bytes
        finally:
            pending_lock.release()
        try:
            self.messages.put_nowait(
                (
                    topic,
                    body,
                    log_ns,
                    source_ns,
                    ros_type,
                    transmitted_schema,
                    budget_bytes,
                    metadata,
                )
            )
        except queue.Full:
            if pending_lock.acquire(timeout=0.002):
                try:
                    self.pending.value -= budget_bytes
                finally:
                    pending_lock.release()
            self.failure = "RECORDING_QUEUE_OVERFLOW"
            self.dropped += 1
            self.local_errors += 1
            raise DataError(self.failure) from None
        self.schemas[topic] = schema
        self.accepted += 1
        self.topic_counts[topic] = self.topic_counts.get(topic, 0) + 1

    def statistics(self):
        if not self.closed:
            try:
                while self.status.poll():
                    value = self.status.recv()
                    if value.get("state") in {"failed", "finished"}:
                        self.result = value
                    if value.get("state") == "failed":
                        self.failure = value.get("error", "RECORDER_WRITE_FAILED")
            except (EOFError, OSError):
                pass
            if (
                not self.process.is_alive()
                and self.result is None
                and self.failure is None
            ):
                self.failure = "RECORDER_EXITED"
                self.local_errors += 1
        stale = False
        for name, counter in (
            ("written", self.written_counter),
            ("errors", self.error_counter),
            ("queue_bytes", self.pending),
        ):
            lock = counter.get_lock()
            if lock.acquire(False):
                try:
                    self.counter_cache[name] = counter.get_obj().value
                finally:
                    lock.release()
            else:
                stale = True
        return {
            "accepted": self.accepted,
            "written": self.counter_cache["written"],
            "dropped": self.dropped,
            "errors": self.local_errors + self.counter_cache["errors"],
            "queue_bytes": self.counter_cache["queue_bytes"],
            "statistics_stale": stale,
            "queue_limit_bytes": self.max_bytes,
            "writer_alive": self.process.is_alive(),
            "last_error": self.failure,
            "topics": dict(self.topic_counts),
        }

    async def finish(self, timeout=30):
        self.closed = True
        self.stop_event.set()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.result is not None:
                break
            if self.status.poll():
                try:
                    result = self.status.recv()
                except EOFError:
                    break
                if result.get("state") in {"finished", "failed"}:
                    self.result = result
                    break
            if not self.process.is_alive():
                break
            await asyncio.sleep(0.02)
        if self.result is None:
            await self.abort()
            raise DataError(
                "RECORDER_FINALIZE_FAILED",
                "Writer exited or timed out; partial data retained",
            )
        await asyncio.to_thread(self.process.join, 1)
        self.messages.close()
        self.messages.cancel_join_thread()
        self.status.close()
        if self.result["state"] != "finished":
            raise DataError(
                "RECORDER_WRITE_FAILED", self.result.get("error", "Unknown write error")
            )
        if self.failure:
            raise DataError(self.failure, "Capture incomplete; partial data retained")
        return {
            **self.result,
            "accepted": self.accepted,
            "dropped": self.dropped,
            "errors": self.local_errors + self.error_counter.value,
        }

    async def abort(self):
        self.closed = True
        self.stop_event.set()
        if self.process.is_alive():
            self.process.terminate()
            await asyncio.to_thread(self.process.join, 2)
        if self.process.is_alive():
            self.process.kill()
            await asyncio.to_thread(self.process.join, 1)
        self.messages.close()
        self.messages.cancel_join_thread()
        self.status.close()
