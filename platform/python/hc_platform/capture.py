"""Independent ROS capture: bounded messages/bytes, raw CDR, no control ownership."""

from __future__ import annotations
import argparse
import asyncio
import base64
import contextlib
from collections import deque
import json
from pathlib import Path
import signal
import sys
import threading
import time

from .data import DataClient
from .configuration import atomic_json


class CaptureStartupError(RuntimeError):
    def __init__(self, code, *, topic=None, type_name=None, stage=None, cause=None):
        self.detail = {
            "code": code,
            "topic": topic,
            "type": type_name,
            "stage": stage,
            "message": str(cause) if cause else code,
        }
        super().__init__(json.dumps(self.detail, ensure_ascii=False))


def ros_schema(type_name):
    from ament_index_python.packages import get_package_share_directory

    root_package, _, name = type_name.split("/")
    seen = set()
    sections = []

    def read(package, message, primary=False):
        key = f"{package}/{message}"
        if key in seen:
            return
        seen.add(key)
        path = Path(get_package_share_directory(package)) / "msg" / f"{message}.msg"
        text = path.read_text()
        sections.append(text if primary else f"\n{'=' * 80}\nMSG: {key}\n{text}")
        for line in text.splitlines():
            tokens = line.split("#", 1)[0].split()
            if len(tokens) < 2 or "=" in tokens[1]:
                continue
            field = tokens[0].split("[", 1)[0]
            if field == "Header":
                read("std_msgs", "Header")
            elif "/" in field:
                parts = field.split("/")
                read(parts[0], parts[-1])
            elif field and field[0].isupper():
                read(package, field)

    read(root_package, name, True)
    return "".join(sections)


class Capture:
    def __init__(
        self,
        socket,
        session_id,
        topics,
        byte_budget=64 * 1024**2,
        ready_file=None,
        message_budget=4096,
    ):
        if (
            type(byte_budget) is not int
            or byte_budget < 1
            or type(message_budget) is not int
            or not 1 <= message_budget <= 65536
        ):
            raise ValueError(
                "Capture requires a positive byte budget and 1–65536 queued messages"
            )
        self.client = DataClient(str(socket))
        self.session_id, self.topics = session_id, topics
        self.byte_budget = byte_budget
        self.message_budget = message_budget
        self.queue = deque()
        self.lock = threading.Lock()
        self.bytes = 0
        self.dropped = 0
        self.overflow = {"messages": 0, "bytes": 0}
        self.last = {}
        self.stopping = False
        self.thread = None
        self.node = None
        self.executor = None
        self.ready_file = Path(ready_file) if ready_file else None

    def receive(self, data, topic, type_name, schema, max_hz):
        now = time.monotonic()
        with self.lock:
            if self.stopping or (
                max_hz > 0 and now - self.last.get(topic, -1e20) < 1 / max_hz
            ):
                return
            if (
                len(self.queue) >= self.message_budget
                or self.bytes + len(data) > self.byte_budget
            ):
                self.dropped += 1
                self.overflow[
                    "messages" if len(self.queue) >= self.message_budget else "bytes"
                ] += 1
                return
            self.last[topic] = now
            self.queue.append(
                (
                    topic,
                    type_name,
                    schema,
                    bytes(data),
                    time.time_ns(),
                    time.monotonic_ns(),
                )
            )
            self.bytes += len(data)

    def start_ros(self):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from rosidl_runtime_py.utilities import get_message
        except ImportError as error:
            raise CaptureStartupError(
                "CAPTURE_ROS_UNAVAILABLE", stage="imports", cause=error
            ) from error
        # Resolve the whole frozen selection before subscribing to any topic.
        # A custom type or schema failure must not silently shrink the capture.
        prepared = []
        for item in self.topics:
            topic, type_name = item.get("topic"), item.get("type")
            try:
                message_type = get_message(type_name)
            except Exception as error:
                raise CaptureStartupError(
                    "CAPTURE_SUBSCRIPTION_UNAVAILABLE",
                    topic=topic,
                    type_name=type_name,
                    stage="message_type",
                    cause=error,
                ) from error
            try:
                schema = ros_schema(type_name)
            except Exception as error:
                raise CaptureStartupError(
                    "CAPTURE_SUBSCRIPTION_UNAVAILABLE",
                    topic=topic,
                    type_name=type_name,
                    stage="schema",
                    cause=error,
                ) from error
            prepared.append((item, message_type, schema))
        try:
            self.context = rclpy.context.Context()
            rclpy.init(context=self.context)
            self.node = rclpy.create_node("hc_platform_capture", context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
        except Exception as error:
            raise CaptureStartupError(
                "CAPTURE_ROS_START_FAILED", stage="node", cause=error
            ) from error
        self.subscriptions = []
        for item, message_type, schema in prepared:
            topic, type_name = item["topic"], item["type"]
            try:
                qos = QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE
                    if item.get("qos_reliability") == "reliable"
                    else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                    if item.get("qos_durability") == "transient_local"
                    else DurabilityPolicy.VOLATILE,
                )
                self.subscriptions.append(
                    self.node.create_subscription(
                        message_type,
                        topic,
                        lambda data,
                        t=topic,
                        n=type_name,
                        s=schema,
                        hz=float(item.get("max_hz", 0)): self.receive(
                            data, t, n, s, hz
                        ),
                        qos,
                        raw=True,
                    )
                )
            except Exception as error:
                raise CaptureStartupError(
                    "CAPTURE_SUBSCRIPTION_UNAVAILABLE",
                    topic=topic,
                    type_name=type_name,
                    stage="subscription",
                    cause=error,
                ) from error
        self.thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="ros-capture"
        )
        self.thread.start()

    async def run(self):
        reported_drops = 0

        async def report_drops():
            nonlocal reported_drops
            with self.lock:
                drops, overflow = self.dropped, dict(self.overflow)
            if drops != reported_drops:
                await self.client.append_sample(
                    session_id=self.session_id,
                    topic="/capture/gaps",
                    payload={
                        "reason": "CAPTURE_QUEUE_OVERFLOW",
                        "dropped": drops,
                        "overflow": overflow,
                        "message_budget": self.message_budget,
                        "byte_budget": self.byte_budget,
                        "data_integrity": "incomplete",
                    },
                )
                reported_drops = drops

        try:
            self.start_ros()
            if self.ready_file:
                atomic_json(
                    self.ready_file,
                    {
                        "ready": True,
                        "session_id": self.session_id,
                        "topics": len(self.subscriptions),
                    },
                )
            while not self.stopping or self.queue:
                with self.lock:
                    item = self.queue.popleft() if self.queue else None
                    if item:
                        self.bytes -= len(item[3])
                if item:
                    topic, type_name, schema, data, wall, steady = item
                    await self.client.append_sample(
                        session_id=self.session_id,
                        topic=topic,
                        payload={
                            "__cdr_b64": base64.b64encode(data).decode(),
                            "__ros_type": type_name,
                            "__ros_schema": schema,
                            "received_monotonic_ns": steady,
                        },
                        source_time_ns=wall,
                    )
                else:
                    await asyncio.sleep(0.005)
                await report_drops()
            # Also audit a final oversized packet when no item was enqueued.
            await report_drops()
        finally:
            if self.executor:
                self.executor.shutdown()
            if self.thread:
                self.thread.join(timeout=2)
            if self.node:
                self.node.destroy_node()
            if getattr(self, "context", None):
                self.context.try_shutdown()
            close = getattr(self.client, "close", None)
            if close:
                await close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--topics", required=True)
    parser.add_argument("--ready-file")
    args = parser.parse_args()
    capture = Capture(
        args.socket,
        args.session_id,
        json.loads(Path(args.topics).read_text()),
        ready_file=args.ready_file,
    )

    async def run():
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, setattr, capture, "stopping", True)
        await capture.run()

    try:
        asyncio.run(run())
    except CaptureStartupError as error:
        print(json.dumps(error.detail, ensure_ascii=False), file=sys.stderr, flush=True)
        raise SystemExit(1) from None


if __name__ == "__main__":
    main()
