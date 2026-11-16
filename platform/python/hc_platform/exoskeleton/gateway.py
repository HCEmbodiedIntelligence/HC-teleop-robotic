"""ROS ingress with a latest-only handoff and HTTP validation before core output.

This process never selects a source, recovers safety, or writes a robot driver.
The HTTP command route applies the same geometry/lease gate as external programs.
"""

from __future__ import annotations
import asyncio
import contextlib
import json
import os
from pathlib import Path
import signal
import threading
import time

import httpx

from .normalizer import ExoskeletonNormalizer
from ..configuration import atomic_json


class ExoskeletonRelay:
    def __init__(self, config, http):
        self.config, self.http = config, http
        self.tokens = None
        self.activation_ns = 0
        self.last_sent_seq = 0
        self.command_seq = 0
        self.status = {
            "source": "exo",
            "selected": False,
            "accepted": 0,
            "rejected": 0,
            "reason": "AWAITING_EXO_SESSION",
        }

    async def poll_state(self):
        response = await self.http.get("/api/control/state")
        response.raise_for_status()
        message = response.json()
        state = message.get("state", message)
        active = (
            state.get("safety") == "ready"
            and state.get("mode") == "exo"
            and state.get("owner") == "exo"
            and not state.get("transition_pending")
        )
        tokens = (
            {
                key: state.get(key)
                for key in ("boot_id", "epoch", "lease_id", "reference_epoch")
            }
            if active
            else None
        )
        if tokens != self.tokens:
            self.tokens = tokens
            self.activation_ns = time.monotonic_ns()
            self.last_sent_seq = 0
        self.status.update(
            selected=active,
            reason="READY" if active else "AWAITING_EXO_SESSION",
            control_epoch=state.get("epoch"),
        )
        self.command_seq = max(self.command_seq, int(state.get("last_command_seq", 0)))
        return active

    async def forward(self, sample):
        if not self.tokens or not sample or sample["seq"] <= self.last_sent_seq:
            return False
        self.last_sent_seq = sample["seq"]
        now = time.monotonic_ns()
        if sample["sample_mono_ns"] < self.activation_ns:
            self.status["reason"] = "PRE_ACTIVATION_SAMPLE"
            return False
        if now >= sample["deadline_mono_ns"]:
            self.status["reason"] = "EXOSKELETON_SAMPLE_STALE"
            self.status["rejected"] += 1
            return False
        self.command_seq = max(self.command_seq + 1, now)
        payload = {
            key: sample[key]
            for key in (
                "joint_names",
                "positions",
                "sample_mono_ns",
                "deadline_mono_ns",
                "ttl_ms",
            )
        }
        payload.update(source="exo", seq=self.command_seq, **self.tokens)
        response = await self.http.post("/api/control/command", json=payload)
        if response.is_error:
            self.status.update(
                reason=response.json().get("code", "HTTP_COMMAND_REJECTED"),
                rejected=self.status["rejected"] + 1,
            )
            return False
        self.status.update(
            reason="ACTIVE",
            accepted=self.status["accepted"] + 1,
            last_ros_stamp_ns=sample["ros_stamp_ns"],
        )
        return True


async def run(config_path, api_url, state_dir):
    config = json.loads(Path(config_path).read_text())
    options = config.get("exoskeleton", {})
    if not options.get("enabled", False):
        raise RuntimeError("Set exoskeleton.enabled=true explicitly")
    # ROS imports are lazy so local simulation and HTTP exoskeleton input do not
    # acquire a mandatory ROS Python dependency.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState

    normalizer = ExoskeletonNormalizer(config)
    lock = threading.Lock()
    latest = None
    received = 0
    input_rejections = 0
    last_error = None
    rclpy.init(args=None)
    node = Node("hc_exoskeleton_gateway")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def callback(message):
        nonlocal latest, received, input_rejections, last_error
        try:
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            sample = normalizer.normalize(
                message.name,
                message.position,
                stamp,
                node.get_clock().now().nanoseconds,
                time.monotonic_ns(),
            )
            with lock:
                latest = sample
                received += 1
        except Exception as exc:
            with lock:
                input_rejections += 1
                last_error = str(exc)

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        JointState,
        options.get("topic", "/control/exoskeleton/joint_states"),
        callback,
        qos,
    )
    thread = threading.Thread(target=executor.spin, name="exo-ros-io", daemon=True)
    thread.start()
    shutdown = asyncio.Event()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, shutdown.set)
    headers = {}
    token = os.environ.get("HC_PLATFORM_TOKEN", "")
    if token:
        headers["Authorization"] = "Bearer " + token
    async with httpx.AsyncClient(
        base_url=api_url.rstrip("/"), headers=headers, timeout=0.5
    ) as http:
        relay = ExoskeletonRelay(config, http)
        last_status = 0
        try:
            while not shutdown.is_set():
                try:
                    active = await relay.poll_state()
                    with lock:
                        sample = latest
                    if active:
                        await relay.forward(sample)
                except Exception as exc:
                    relay.tokens = None
                    relay.status.update(
                        selected=False, reason="API_UNAVAILABLE", error=str(exc)
                    )
                if time.monotonic() - last_status > 0.5:
                    with lock:
                        status = {
                            **relay.status,
                            "received": received,
                            "input_rejections": input_rejections,
                            "last_input_error": last_error,
                        }
                    status.update(
                        pid=os.getpid(),
                        updated_mono_ns=time.monotonic_ns(),
                        topic=options.get("topic", "/control/exoskeleton/joint_states"),
                        transport="ROS2 JointState -> checked HTTP -> core",
                    )
                    await asyncio.to_thread(
                        atomic_json, Path(state_dir) / "exoskeleton.json", status
                    )
                    last_status = time.monotonic()
                try:
                    await asyncio.wait_for(shutdown.wait(), 0.01)
                except asyncio.TimeoutError:
                    pass
        finally:
            executor.shutdown(timeout_sec=1)
            thread.join(1)
            node.destroy_subscription(subscription)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
