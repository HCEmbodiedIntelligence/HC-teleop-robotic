"""Independent ROS media process with bounded latest frames and real encoding.

ROS callbacks retain bytes / counters. Decoding and encoding never share a
process or executor with the control gate.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
import contextlib
from dataclasses import dataclass, field
from fractions import Fraction
import importlib.util
import json
import math
import os
from pathlib import Path
import secrets
import socket
import threading
import time

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import Response, StreamingResponse
import uvicorn

from .camera_metadata import CameraMetadata
from .topics import public_publish_allowed

IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}


@dataclass
class Rate:
    received: int = 0
    dropped: int = 0
    last: float | None = None
    samples: deque = field(default_factory=lambda: deque(maxlen=256))
    error: str | None = None

    def tick(self, now):
        self.received += 1
        self.last = now
        self.samples.append(now)

    def status(self, now):
        stamps = [stamp for stamp in self.samples if now - stamp <= 2]
        hz = (
            (len(stamps) - 1) / (stamps[-1] - stamps[0])
            if len(stamps) > 1 and stamps[-1] > stamps[0]
            else 0.0
        )
        return {
            "received": self.received,
            "dropped": self.dropped,
            "fps": hz,
            "actual_hz": hz,
            "age_ms": (now - self.last) * 1000 if self.last is not None else None,
            "error": self.error,
        }


@dataclass
class Frame:
    data: bytes
    encoding: str
    width: int
    height: int
    step: int
    stamp_ns: int
    sequence: int
    received: float


class Media:
    def __init__(self, config):
        self.config = config
        self.camera = config.get("camera", {})
        self.streams = {}
        for source in self.camera.get("streams", []):
            stream = dict(source)
            for name, default, maximum in (
                ("width", 640, 4096),
                ("height", 400, 4096),
                ("fps", 30, 120),
            ):
                value = float(stream.get(name, self.camera.get(name, default)))
                if (
                    not math.isfinite(value)
                    or not 1 <= value <= maximum
                    or (name != "fps" and (value != int(value) or int(value) % 2))
                ):
                    raise ValueError("Invalid camera " + name)
                stream[name] = value if name == "fps" else int(value)
            self.streams[stream["id"]] = stream
            macroblocks = math.ceil(stream["width"] / 16) * math.ceil(
                stream["height"] / 16
            )
            if macroblocks > 3600 or macroblocks * stream["fps"] > 108000:
                raise ValueError(
                    "Camera dimensions/fps exceed the negotiated H264 level 3.1 limits"
                )
        self.frames = {}
        self.metrics = {identifier: Rate() for identifier in self.streams}
        self.observations = {}
        self.graph = []
        self.lock = threading.Lock()
        self.peers = set()
        self.sessions = {}
        self.last_encoder_attempt = {}
        self.reason = None
        self.node = self.executor = self.thread = self.context = None
        self.publishers = {}
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.setblocking(False)
        self.subscriptions = []
        self.topic_config = config.get("topics", [])
        self.topic_specs = {item["topic"]: item for item in self.topic_config}
        self.metrics.update({topic: Rate() for topic in self.topic_specs})
        self._last_ui = {}
        self.metadata = CameraMetadata(float(self.camera.get("metadata_max_age_s", 3)))
        self.metadata_topics = {}
        self.configuration_errors = {}

    def _subscribe(self, get_message, kind, topic, callback, qos, metric_key=None):
        try:
            self.subscriptions.append(
                self.node.create_subscription(get_message(kind), topic, callback, qos)
            )
        except Exception as error:
            reason = "SUBSCRIPTION_UNAVAILABLE: " + str(error)
            self.configuration_errors[str(topic) + ":" + str(kind)] = reason
            if metric_key in self.metrics:
                self.metrics[metric_key].error = reason

    def receive(self, identifier, message):
        now = time.monotonic()
        with self.lock:
            metric = self.metrics[identifier]
            if len(message.data) > 16 * 1024**2:
                metric.dropped += 1
                metric.error = "FRAME_EXCEEDS_16MIB"
                return
            metric.tick(now)
            metric.error = None
            header = getattr(message, "header", None)
            stamp = header.stamp.sec * 10**9 + header.stamp.nanosec if header else 0
            self.frames[identifier] = Frame(
                bytes(message.data),
                getattr(message, "format", getattr(message, "encoding", "")),
                getattr(message, "width", 0),
                getattr(message, "height", 0),
                getattr(message, "step", 0),
                stamp,
                metric.received,
                now,
            )

    def _subscribe_metadata(self, get_message, qos, static_qos):
        for identifier, stream in self.streams.items():
            if not stream.get("enabled", True):
                continue
            config = stream.get("metadata", {})
            subscriptions = []
            for kind in ("calibration", "extrinsics"):
                selected = config.get(kind + "_topics", stream.get(kind + "_topic"))
                if selected is None:
                    selected = ["/observation/cameras/" + identifier + "/" + kind]
                    if identifier == "head":
                        selected.append("/io_teleop/camera_head/" + kind)
                if isinstance(selected, str):
                    selected = [selected]
                for topic in selected[:8]:
                    subscriptions.append(topic)
                    self._subscribe(
                        get_message,
                        "std_msgs/msg/String",
                        topic,
                        lambda msg,
                        i=identifier,
                        k=kind,
                        t=topic: self.metadata.json_message(i, k, t, msg.data),
                        qos,
                    )
            info_topics = config.get(
                "camera_info_topics", stream.get("camera_info_topic")
            )
            if info_topics is None:
                info_topics = ["/observation/cameras/" + identifier + "/camera_info"]
                if identifier == "head":
                    info_topics += [
                        "/camera/color/camera_info",
                        "/io_teleop/camera_head/color/camera_info",
                    ]
            if isinstance(info_topics, str):
                info_topics = [info_topics]
            for topic in info_topics[:8]:
                subscriptions.append(topic)
                self._subscribe(
                    get_message,
                    "sensor_msgs/msg/CameraInfo",
                    topic,
                    lambda msg, i=identifier, t=topic: self.metadata.camera_info(
                        i, t, msg
                    ),
                    qos,
                )
            self.metadata_topics[identifier] = subscriptions
        self._subscribe(
            get_message, "tf2_msgs/msg/TFMessage", "/tf", self.metadata.tf_message, qos
        )
        self._subscribe(
            get_message,
            "tf2_msgs/msg/TFMessage",
            "/tf_static",
            lambda msg: self.metadata.tf_message(msg, True),
            static_qos,
        )

    def start(self):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from rosidl_runtime_py.utilities import get_message

            self.context = rclpy.context.Context()
            rclpy.init(context=self.context)
            self.node = rclpy.create_node("hc_platform_media", context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            static_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            for identifier, stream in self.streams.items():
                if stream.get("enabled", True):
                    kind = stream.get("type", "sensor_msgs/msg/CompressedImage")
                    if kind not in IMAGE_TYPES:
                        reason = "Unsupported camera message " + kind
                        self.configuration_errors[identifier] = reason
                        self.metrics[identifier].error = reason
                        continue
                    self._subscribe(
                        get_message,
                        kind,
                        stream["topic"],
                        lambda msg, i=identifier: self.receive(i, msg),
                        qos,
                        identifier,
                    )
            for item in self.topic_config:
                # Recording-only topics need diagnostics but no payload conversion.
                if not item.get("enabled", True):
                    continue
                selected_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE
                    if item.get("qos_reliability") == "reliable"
                    else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                    if item.get("qos_durability") == "transient_local"
                    else DurabilityPolicy.VOLATILE,
                )
                self._subscribe(
                    get_message,
                    item["type"],
                    item["topic"],
                    lambda msg, spec=item: self.observe(msg, spec),
                    selected_qos,
                    item["topic"],
                )
            self._subscribe_metadata(get_message, qos, static_qos)
            self.timer = self.node.create_timer(1.0, self.update_graph)
            self.thread = threading.Thread(
                target=self.executor.spin, name="ros-media", daemon=True
            )
            self.thread.start()
        except Exception as exc:
            self.reason = "ROS_MEDIA_UNAVAILABLE: " + str(exc)

    def update_graph(self):
        graph = self.node.get_topic_names_and_types()
        with self.lock:
            self.graph = [{"topic": name, "types": types} for name, types in graph]

    def observe(self, message, spec):
        topic = spec["topic"]
        now = time.monotonic()
        with self.lock:
            metric = self.metrics.setdefault(topic, Rate())
            metric.tick(now)
            if not set(spec.get("outputs", [])) & {"websocket", "udp"}:
                return
            rate = float(spec.get("event_max_hz", 10))
            if rate and now - self._last_ui.get(topic, -1e20) < 1 / rate:
                return
            self._last_ui[topic] = now
        # Raw images never become JSON arrays. Use the camera preview path.
        if spec["type"] in IMAGE_TYPES:
            value = {
                "bytes": len(message.data),
                "encoding": getattr(
                    message, "encoding", getattr(message, "format", "")
                ),
                "width": getattr(message, "width", None),
                "height": getattr(message, "height", None),
                "payload_omitted": True,
            }
        else:
            from rosidl_runtime_py.convert import message_to_ordereddict

            value = message_to_ordereddict(message)
        encoded = json.dumps(
            {"topic": topic, "type": spec["type"], "message": value}, default=list
        ).encode()
        if len(encoded) > 65507:
            with self.lock:
                metric.dropped += 1
                metric.error = "DISPLAY_PAYLOAD_EXCEEDS_DATAGRAM_LIMIT"
            return
        with self.lock:
            self.observations[topic] = {"message": value, "received_monotonic": now}
        if "udp" in spec.get("outputs", []):
            target = spec["udp_target"]
            try:
                self.udp.sendto(encoded, (target["host"], int(target["port"])))
            except OSError as error:
                with self.lock:
                    metric.dropped += 1
                    metric.error = "UDP_FORWARD_FAILED: " + str(error)

    def snapshot(self, identifier):
        import cv2
        import numpy as np

        stream = self.streams[identifier]
        with self.lock:
            frame = self.frames.get(identifier)
        if frame is None or time.monotonic() - frame.received > 2:
            raise ValueError("CAMERA_NO_FRESH_FRAME")
        if frame.width == 0:
            image = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(frame.encoding)
            if channels is None:
                raise ValueError("Unsupported raw encoding: " + frame.encoding)
            if (
                min(frame.width, frame.height, frame.step) <= 0
                or frame.step < frame.width * channels
                or len(frame.data) != frame.height * frame.step
            ):
                raise ValueError("INVALID_RAW_IMAGE_LAYOUT")
            rows = np.frombuffer(frame.data, np.uint8).reshape(frame.height, frame.step)
            image = rows[:, : frame.width * channels].reshape(
                frame.height, frame.width, channels
            )
            if frame.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif channels == 1:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image is None:
            raise ValueError("IMAGE_DECODE_FAILED")
        if image.shape[1] != stream["width"] or image.shape[0] != stream["height"]:
            image = cv2.resize(
                image, (stream["width"], stream["height"]), interpolation=cv2.INTER_AREA
            )
        return image, frame

    def jpeg(self, identifier):
        import cv2

        image, frame = self.snapshot(identifier)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise ValueError("JPEG_ENCODE_FAILED")
        return encoded.tobytes(), frame

    def encoder_status(self, session_id, identifier, value):
        with self.lock:
            session = self.sessions.get(session_id)
            if session is None:
                return
            item = session["encoders"].setdefault(
                identifier, {"rate": Rate(), "status": {}}
            )
            if value["frames_encoded"] > item["status"].get("frames_encoded", 0):
                item["rate"].tick(time.monotonic())
            item["status"] = value
            self.last_encoder_attempt[identifier] = dict(
                value, updated_mono=time.monotonic()
            )

    def frame_sent(self, session_id, identifier, timestamp, packet_size):
        with self.lock:
            session = self.sessions.get(session_id)
            if session is None:
                return
            item = session["sent"].setdefault(
                identifier, {"rate": Rate(), "last_timestamp": None}
            )
            if item["last_timestamp"] != timestamp:
                item["rate"].tick(time.monotonic())
                item["last_timestamp"] = timestamp

    def receiver(self, identifier, payload):
        if (
            not isinstance(payload.get("session_id"), str)
            or len(payload["session_id"]) > 128
        ):
            raise HTTPException(400, "INVALID_TELEMETRY_SESSION")
        with self.lock:
            session = self.sessions.get(payload.get("session_id"))
            if session is None or identifier not in session["streams"]:
                raise HTTPException(409, "UNKNOWN_TELEMETRY_SESSION")
            previous = session["receiver"].get(identifier)
            clean = {}
            names = (
                "sequence",
                "sample_ms",
                "packets_received",
                "packets_lost",
                "fec_packets_received",
                "fec_packets_discarded",
                "frames_received",
                "frames_decoded",
                "frames_dropped",
                "freeze_count",
                "freeze_duration_ms",
                "jitter_buffer_delay_ms",
                "jitter_buffer_emitted_count",
                "decode_time_ms",
            )
            for name in names:
                value = payload.get(name, 0)
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    or not (-1e9 if name == "packets_lost" else 0) <= value <= 1e15
                ):
                    raise HTTPException(400, "INVALID_RECEIVER_COUNTER: " + name)
                clean[name] = value
            if (
                clean["sequence"] != int(clean["sequence"])
                or "sequence" not in payload
                or "sample_ms" not in payload
            ):
                raise HTTPException(400, "INVALID_RECEIVER_SEQUENCE")
            if previous and (
                clean["sequence"] <= previous["sequence"]
                or clean["sample_ms"] <= previous["sample_ms"]
            ):
                raise HTTPException(409, "STALE_RECEIVER_TELEMETRY")
            clean["stats_id"] = str(payload.get("stats_id", ""))[:256]
            clean["received_mono"] = time.monotonic()
            clean["source"] = "receiver_reported"
            if previous and clean["stats_id"] == previous["stats_id"]:
                duration = (clean["sample_ms"] - previous["sample_ms"]) / 1000
                clean["decoded_fps"] = (
                    max(0, clean["frames_decoded"] - previous["frames_decoded"])
                    / duration
                )
            session["receiver"][identifier] = clean
        return {"accepted": True}

    def status(self):
        now = time.monotonic()
        with self.lock:
            streams = []
            for identifier, spec in self.streams.items():
                frame = self.frames.get(identifier)
                metric = self.metrics[identifier].status(now)
                sessions = [
                    session
                    for session in self.sessions.values()
                    if identifier in session["streams"]
                ]
                encoders = [
                    {
                        **session["encoders"][identifier]["status"],
                        "encode_fps": session["encoders"][identifier]["rate"].status(
                            now
                        )["fps"],
                    }
                    for session in sessions
                    if identifier in session["encoders"]
                ]
                alive = bool(frame and now - frame.received < 2)
                actual = sorted(
                    {encoder["actual"] for encoder in encoders if encoder["actual"]}
                )
                last_attempt = self.last_encoder_attempt.get(identifier)
                streams.append(
                    {
                        **spec,
                        "name": spec.get("name", identifier),
                        "state": "disabled"
                        if not spec.get("enabled", True)
                        else "error"
                        if metric["error"]
                        else "running"
                        if alive
                        else "waiting",
                        "source_alive": alive,
                        "frame_age_ms": metric["age_ms"],
                        "frames_received": metric["received"],
                        "fps": metric["fps"],
                        "capture_fps": metric["fps"],
                        "drops": metric["dropped"],
                        "error": metric["error"],
                        "webrtc_send_fps": max(
                            (
                                session["sent"][identifier]["rate"].status(now)["fps"]
                                for session in sessions
                                if identifier in session["sent"]
                            ),
                            default=0,
                        ),
                        "webrtc_send_fps_basis": "complete_frame_dispatched_to_dtls_transport",
                        "encoder_requested": spec.get(
                            "encoder_backend",
                            self.camera.get("encoder_backend", "x264"),
                        ),
                        "sender_skipped_frames": sum(
                            session.get("skipped_slots", {}).get(identifier, 0)
                            for session in sessions
                        ),
                        "encoder_actual": actual[0]
                        if len(actual) == 1
                        else actual or None,
                        "encoder_probe_state": "active"
                        if actual
                        else "failed"
                        if encoders
                        and all(encoder.get("error") for encoder in encoders)
                        else "probing"
                        if encoders
                        else "idle"
                        if last_attempt
                        else "not_tested",
                        "last_encoder_attempt": dict(
                            last_attempt,
                            age_ms=(now - last_attempt["updated_mono"]) * 1000,
                        )
                        if last_attempt
                        else None,
                        "fallback_reason": "; ".join(
                            dict.fromkeys(
                                encoder["fallback_reason"]
                                for encoder in encoders
                                if encoder.get("fallback_reason")
                            )
                        )
                        or None,
                        "encoders": encoders,
                        "repair": [session.get("repair", {}) for session in sessions],
                        "receivers": [
                            {
                                **session["receiver"][identifier],
                                "age_ms": (
                                    now
                                    - session["receiver"][identifier]["received_mono"]
                                )
                                * 1000,
                            }
                            for session in sessions
                            if identifier in session["receiver"]
                        ],
                    }
                )
            metrics = {}
            for topic, spec in self.topic_specs.items():
                value = self.metrics[topic].status(now)
                maximum_age = (
                    max(2, 3 / max(float(spec.get("min_hz", 0)), 0.1))
                    if spec.get("min_hz")
                    else 2
                )
                value["state"] = (
                    "disabled"
                    if not spec.get("enabled", True)
                    else "error"
                    if value["error"]
                    else "waiting"
                    if value["age_ms"] is None
                    else "stale"
                    if value["age_ms"] > maximum_age * 1000
                    else "slow"
                    if value["actual_hz"] < float(spec.get("min_hz", 0))
                    else "healthy"
                )
                value["measurement"] = "ros_callback_arrival"
                metrics[topic] = value
            result = {
                "streams": streams,
                "ros_available": self.node is not None and self.reason is None,
                "reason": self.reason,
                "topics": list(self.graph),
                "observations": dict(self.observations),
                "topic_metrics": metrics,
                "configuration_errors": dict(self.configuration_errors),
                "supported_video_codecs": ["H264"],
                "webrtc_available": bool(importlib.util.find_spec("aiortc")),
                "viewers": len(self.peers),
                "fec": {
                    "available": False,
                    "enabled": False,
                    "reason": "RED/ULPFEC sender is not implemented; negotiated NACK/RTX/PLI are separate repair mechanisms",
                },
            }
        for stream in result["streams"]:
            meta = stream.get("metadata", {})
            stream["metadata"] = self.metadata.status(
                stream["id"],
                meta.get("parent_frame", "base_link"),
                meta.get("camera_frame", stream.get("frame_id")),
            )
            stream["metadata"]["subscribed_topics"] = self.metadata_topics.get(
                stream["id"], []
            )
        return result

    async def close(self):
        await asyncio.gather(
            *(peer.close() for peer in tuple(self.peers)), return_exceptions=True
        )
        self.peers.clear()
        self.sessions.clear()
        if self.executor:
            self.executor.shutdown()
        if self.thread:
            self.thread.join(timeout=2)
        if self.node:
            self.node.destroy_node()
        if self.context:
            self.context.try_shutdown()
        self.udp.close()


def create_media_app(config):
    media = Media(config)

    @contextlib.asynccontextmanager
    async def lifespan(app):
        media.start()
        yield
        await media.close()

    app = FastAPI(lifespan=lifespan)
    app.state.media = media
    slots = asyncio.Semaphore(2)

    @app.get("/status")
    async def status():
        return media.status()

    @app.get("/validate-types")
    async def validate_types(types: str = ""):
        results = {}
        try:
            from rosidl_runtime_py.utilities import get_message
        except Exception as exc:
            return {"ros_available": False, "error": str(exc), "results": results}
        for item in [entry.strip() for entry in types.split(",") if entry.strip()]:
            try:
                get_message(item)
                results[item] = None
            except Exception as exc:
                results[item] = f"{type(exc).__name__}: {exc}"
        return {"ros_available": bool(media.node), "results": results}

    @app.get("/cameras/{identifier}/snapshot")
    async def snapshot(identifier: str):
        try:
            async with slots:
                data, frame = await asyncio.to_thread(media.jpeg, identifier)
            return Response(
                data,
                media_type="image/jpeg",
                headers={
                    "X-Frame-Sequence": str(frame.sequence),
                    "Cache-Control": "no-store",
                },
            )
        except (ValueError, KeyError) as exc:
            raise HTTPException(503, str(exc))

    @app.get("/cameras/{identifier}/preview")
    async def preview(identifier: str, request: Request):
        if identifier not in media.streams:
            raise HTTPException(404, "Unknown camera")

        async def frames():
            sequence = -1
            while not await request.is_disconnected():
                try:
                    async with slots:
                        data, frame = await asyncio.to_thread(media.jpeg, identifier)
                    if frame.sequence != sequence:
                        sequence = frame.sequence
                        yield (
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                            + data
                            + b"\r\n"
                        )
                except ValueError:
                    pass
                await asyncio.sleep(0.1)

        return StreamingResponse(
            frames(), media_type="multipart/x-mixed-replace; boundary=frame"
        )

    @app.post("/receiver/{identifier}")
    async def receiver(identifier: str, payload: dict):
        if len(json.dumps(payload)) > 32768:
            raise HTTPException(413, "Receiver telemetry too large")
        return media.receiver(identifier, payload)

    @app.post("/offer")
    async def offer(payload: dict):
        try:
            from aiortc import (
                RTCPeerConnection,
                RTCSessionDescription,
                RTCRtpSender,
                VideoStreamTrack,
                RTCConfiguration,
                RTCIceServer,
            )
            from aiortc.mediastreams import MediaStreamError
            from av import VideoFrame
            from .video_encoding import (
                install_sender_encoder,
                h264_preferences,
                negotiated_repair,
            )
        except ImportError as exc:
            raise HTTPException(503, "WEBRTC_DEPENDENCY_MISSING: " + str(exc))
        identifiers = (
            payload.get("camera_ids")
            or payload.get("streams")
            or [payload.get("camera_id", "head")]
        )
        if (
            not isinstance(identifiers, list)
            or not 1 <= len(identifiers) <= 4
            or any(
                not isinstance(i, str)
                or i not in media.streams
                or not media.streams[i].get("enabled", True)
                for i in identifiers
            )
            or len(set(identifiers)) != len(identifiers)
        ):
            raise HTTPException(400, "Select 1-4 different enabled camera IDs")
        if (
            payload.get("type") != "offer"
            or not isinstance(payload.get("sdp"), str)
            or len(payload["sdp"]) > 131072
        ):
            raise HTTPException(400, "Invalid SDP offer")
        if len(media.peers) >= 4:
            raise HTTPException(429, "Viewer limit reached")

        class Track(VideoStreamTrack):
            def __init__(self, identifier):
                super().__init__()
                self.identifier = identifier
                self.fps = media.streams[identifier]["fps"]
                self.started = None
                self.index = 0

            async def recv(self):
                if self.started is None:
                    self.started = time.monotonic()
                deadline = self.started + self.index / self.fps
                await asyncio.sleep(max(0, deadline - time.monotonic()))
                if self.readyState != "live":
                    raise MediaStreamError
                try:
                    async with slots:
                        image, _ = await asyncio.to_thread(
                            media.snapshot, self.identifier
                        )
                except (ValueError, KeyError):
                    self.stop()
                    raise MediaStreamError
                frame = VideoFrame.from_ndarray(image, format="bgr24")
                frame.pts, frame.time_base = (
                    round(self.index * 90000 / self.fps),
                    Fraction(1, 90000),
                )
                next_index = max(
                    self.index + 1,
                    math.floor((time.monotonic() - self.started) * self.fps) + 1,
                )
                skipped = next_index - self.index - 1
                with media.lock:
                    session = media.sessions.get(session_id)
                    if session is not None:
                        counts = session.setdefault("skipped_slots", {})
                        counts[self.identifier] = (
                            counts.get(self.identifier, 0) + skipped
                        )
                self.index = next_index
                return frame

        ice = [RTCIceServer(**value) for value in media.camera.get("ice_servers", [])]
        peer = RTCPeerConnection(RTCConfiguration(iceServers=ice))
        session_id = secrets.token_urlsafe(24)
        media.peers.add(peer)
        media.sessions[session_id] = {
            "streams": list(identifiers),
            "encoders": {},
            "receiver": {},
            "sent": {},
        }

        @peer.on("connectionstatechange")
        async def changed():
            if peer.connectionState in ("failed", "closed"):
                if peer.connectionState == "failed":
                    await peer.close()
                media.peers.discard(peer)
                with media.lock:
                    media.sessions.pop(session_id, None)

        try:
            for identifier in identifiers:
                spec = media.streams[identifier]
                sender = peer.addTrack(Track(identifier))
                transceiver = next(
                    t for t in peer.getTransceivers() if t.sender == sender
                )
                transceiver.setCodecPreferences(
                    h264_preferences(RTCRtpSender.getCapabilities("video"))
                )
                install_sender_encoder(
                    sender,
                    requested=spec.get(
                        "encoder_backend", media.camera.get("encoder_backend", "x264")
                    ),
                    fps=spec["fps"],
                    bitrate=int(
                        spec.get("bitrate", media.camera.get("bitrate", 2_000_000))
                    ),
                    on_status=lambda value, i=identifier: media.encoder_status(
                        session_id, i, value
                    ),
                    on_sent=lambda stamp, size, i=identifier: media.frame_sent(
                        session_id, i, stamp, size
                    ),
                )
            # aiortc computes the common codec list while setting the remote
            # offer. Preferences must already be installed at that point.
            await peer.setRemoteDescription(
                RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])
            )
            await peer.setLocalDescription(await peer.createAnswer())
            media.sessions[session_id]["repair"] = negotiated_repair(
                peer.localDescription.sdp
            )
            return {
                "sdp": peer.localDescription.sdp,
                "type": peer.localDescription.type,
                "telemetry_session_id": session_id,
            }
        except Exception as exc:
            await peer.close()
            media.peers.discard(peer)
            with media.lock:
                media.sessions.pop(session_id, None)
            raise HTTPException(400, str(exc))

    @app.post("/publish")
    async def publish(payload: dict):
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.set_message import set_message_fields

        if not public_publish_allowed(payload.get("topic", "")):
            raise HTTPException(
                403, "Only /external/ non-control topics may be published"
            )
        if not media.node:
            raise HTTPException(503, "ROS_UNAVAILABLE")
        if len(json.dumps(payload)) > 65536:
            raise HTTPException(413, "Message too large")
        try:
            kind = get_message(payload["type"])
            message = kind()
            set_message_fields(message, payload["message"])
        except (
            KeyError,
            ImportError,
            ModuleNotFoundError,
            ValueError,
            TypeError,
            AttributeError,
        ) as exc:
            raise HTTPException(400, f"Invalid ROS message or payload: {exc}") from exc
        key = (payload["topic"], payload["type"])
        if key not in media.publishers:
            if len(media.publishers) >= 32:
                raise HTTPException(429, "Publisher limit")
            media.publishers[key] = media.node.create_publisher(kind, key[0], 1)
        media.publishers[key].publish(message)
        return {"accepted": True}

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--socket", required=True)
    args = parser.parse_args()
    os.umask(0o077)
    uvicorn.run(
        create_media_app(json.loads(Path(args.config).read_text())),
        uds=args.socket,
        log_level="warning",
    )


if __name__ == "__main__":
    main()
