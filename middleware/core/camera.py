from __future__ import annotations

import asyncio
import io
from fractions import Fraction
import os
import threading
import time
from typing import Any

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class CameraService:
    """Multi-source camera streaming service with WebRTC projection for VR headsets.

    Supports:
      1. ROS 2 topic subscription (CompressedImage / Image, default or custom topics)
      2. Direct Intel RealSense USB pipeline fallback
      3. Real-time H.264 WebRTC streaming to PICO / Meta Quest VR headsets
      4. Fresh-frame-only WebRTC output paced by camera capture timestamps
    """

    def __init__(self, config: dict[str, Any], domain_id: int = 14):
        self.config = config
        self.domain_id = int(domain_id)
        self._latest: Any = None
        self._frame_sequence = 0
        self._frame_arrival_ns = 0
        self._frame_capture_ns = 0
        self._encoded_arrival_ns = 0
        self._encoded_capture_ns = 0
        self._last_capture_at = 0.0
        self._last_send_at = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._decode_thread: threading.Thread | None = None
        self._decode_event = threading.Event()
        self._encoded_latest: bytes | None = None
        self._pcs: set[Any] = set()
        self._stats_tasks: set[asyncio.Task[Any]] = set()
        self._frame_waiters: set[tuple[asyncio.AbstractEventLoop, asyncio.Event]] = set()
        self._frames_received = 0
        self._last_fps_calc = time.monotonic()
        self._fps_counter = 0
        self._decoded_last_fps_calc = time.monotonic()
        self._decoded_fps_counter = 0
        self._last_decoded_at = 0.0
        self._webrtc_last_fps_calc = time.monotonic()
        self._webrtc_fps_counter = 0
        self._webrtc_frames_sent = 0
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", False) else "starting",
            "source": config.get("source", "ros"),
            "topic": config.get("topic", "/observation/cameras/head/color/compressed"),
            "custom_topic": config.get("custom_topic", ""),
            "capture_fps": 0.0,
            "decoded_fps": 0.0,
            "jpeg_decode_ms": 0.0,
            "webrtc_send_fps": 0.0,
            "webrtc_frames_sent": 0,
            "webrtc_codec": "",
            "webrtc_bitrate_mbps": 0.0,
            "webrtc_target_bitrate_mbps": 0.0,
            "webrtc_encode_ms": 0.0,
            "webrtc_encoded_frame_kb": 0.0,
            "webrtc_forced_keyframes": 0,
            "webrtc_packets_sent": 0,
            "webrtc_remote_packets_lost": 0,
            "webrtc_fraction_lost_pct": 0.0,
            "webrtc_rtt_ms": None,
            "peers": 0,
            "webrtc_state": "idle",
            "ice_state": "new",
            "webrtc_error": None,
            "remote_ice_candidates": 0,
            "local_ice_candidates": 0,
            "error": None,
            "id": str(config.get("id", "main")),
            "name": str(config.get("name", "主相机")),
            "is_stereo": bool(
                config.get("stereo", False)
                or "stereo" in str(config.get("id", "")).lower()
                or "双目" in str(config.get("name", "")).lower()
                or "sbs" in str(config.get("id", "")).lower()
                or "stereo" in str(config.get("topic", "")).lower()
            ),
        }
        self._left_frame: np.ndarray | None = None
        self._right_frame: np.ndarray | None = None

    def start(self) -> None:
        if not self.config.get("enabled", False):
            return
        source = str(self.config.get("source", "ros")).lower()
        if source == "realsense":
            self._thread = threading.Thread(
                target=self._capture_loop_realsense, name="camera-realsense", daemon=True
            )
            self._thread.start()
        else:
            self._decode_thread = threading.Thread(
                target=self._decode_loop_ros,
                name="camera-ros-decode",
                daemon=True,
            )
            self._decode_thread.start()
            self._set_status(state="running", error=None)

    def handle_ros_message(self, topic: str, msg: Any) -> None:
        if not self.config.get("enabled", False):
            return
        arrival_ns = time.monotonic_ns()
        capture_ns = self._message_capture_ns(msg) or arrival_ns
        try:
            active_topic = self.config.get("custom_topic", "") or self.config.get("topic", "/observation/cameras/head/color/compressed")
            if active_topic and topic != active_topic and not topic.endswith(
                active_topic.lstrip("/")
            ):
                return

            frame = None
            if hasattr(msg, "data") and hasattr(msg, "format"):
                self._record_capture_frame(topic)
                # 没有 VR 客户端观看该路画面时，只统计输入帧率，不做 JPEG
                # 解码。多相机总开关关闭后可立即释放 CPU。
                if not self._pcs:
                    return
                with self._lock:
                    self._encoded_latest = bytes(msg.data)
                    self._encoded_arrival_ns = arrival_ns
                    self._encoded_capture_ns = capture_ns
                self._decode_event.set()
                return
            elif hasattr(msg, "data") and hasattr(msg, "encoding"):
                self._record_capture_frame(topic)
                if not self._pcs:
                    return
                if np is not None and cv2 is not None:
                    if msg.encoding in ("bgr8", "8UC3"):
                        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
                    elif msg.encoding == "rgb8":
                        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    elif msg.encoding == "mono8":
                        mono = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                        frame = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

            if frame is not None:
                self._accept_frame(
                    frame,
                    topic,
                    count_capture=False,
                    arrival_ns=arrival_ns,
                    capture_ns=capture_ns,
                )
        except Exception as exc:
            self._set_status(error=f"decode error: {exc}")

    async def stop(self) -> None:
        tasks = list(self._stats_tasks)
        self._stats_tasks.clear()
        for task in tasks:
            task.cancel()
        pcs = list(self._pcs)
        self._pcs.clear()
        for pc in pcs:
            try:
                await pc.close()
            except Exception:
                pass
        self._stop.set()
        self._decode_event.set()
        if self._decode_thread is not None and self._decode_thread.is_alive():
            await asyncio.to_thread(self._decode_thread.join, 2.0)
            self._decode_thread = None
        if self._thread is not None and self._thread.is_alive():
            await asyncio.to_thread(self._thread.join, 2.0)
            self._thread = None
        with self._lock:
            waiters = list(self._frame_waiters)
            self._frame_waiters.clear()
            self._latest = None
            self._frames_received = 0
            self._status["state"] = "stopped"
            self._status["capture_fps"] = 0.0
            self._status["webrtc_send_fps"] = 0.0
            self._status["webrtc_frames_sent"] = 0
            self._status["peers"] = 0
        for loop, event in waiters:
            if not loop.is_closed():
                loop.call_soon_threadsafe(event.set)

    @staticmethod
    def _message_capture_ns(msg: Any) -> int:
        """Return the ROS capture stamp, or zero when the source has none."""
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return 0
        sec = int(getattr(stamp, "sec", 0) or 0)
        nanosec = int(getattr(stamp, "nanosec", 0) or 0)
        value = sec * 1_000_000_000 + nanosec
        return value if value > 0 else 0

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
        now = time.monotonic()
        if now - self._last_capture_at > 1.8:
            result["capture_fps"] = 0.0
        if now - self._last_decoded_at > 1.8:
            result["decoded_fps"] = 0.0
        if now - self._last_send_at > 1.8:
            result["webrtc_send_fps"] = 0.0
        result.update(
            id=str(self.config.get("id", "main")),
            name=str(self.config.get("name", "主相机")),
            source=self.config.get("source", "ros"),
            topic=self.config.get("topic", "/observation/cameras/head/color/compressed"),
            custom_topic=self.config.get("custom_topic", ""),
            width=self.config.get("width", 640),
            height=self.config.get("height", 400),
            target_fps=self.config.get("fps", 30),
            codec=self.config.get("codec", "H264"),
            is_stereo=bool(
                self.config.get("stereo", False)
                or "stereo" in str(self.config.get("id", "")).lower()
                or "双目" in str(self.config.get("name", "")).lower()
                or "sbs" in str(self.config.get("id", "")).lower()
                or "stereo" in str(self.config.get("topic", "")).lower()
            ),
            peers=len(self._pcs),
        )
        return result

    def set_frame(self, image: Any) -> None:
        """Manually push a BGR numpy image frame."""
        self._accept_frame(image, str(self.config.get("topic", "manual")),
                           arrival_ns=time.monotonic_ns())

    def latest(self) -> Any:
        with self._lock:
            if self._latest is not None:
                return self._latest
        return self._generate_placeholder()

    def _generate_placeholder(self) -> Any:
        if np is None:
            return None
        w = int(self.config.get("width", 640) or 640)
        h = int(self.config.get("height", 400) or 400)
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = (25, 25, 30)
        if cv2 is not None:
            topic = self.config.get("topic", "/observation/cameras/head/color/compressed")
            custom_topic = self.config.get("custom_topic", "")
            active_topic = custom_topic if custom_topic else topic
            fps = self.config.get("fps", 30)
            cv2.putText(
                img,
                "HC-Teleop Camera Stream",
                (20, h // 2 - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 128),
                2,
            )
            cv2.putText(
                img,
                f"Awaiting Topic: {active_topic}",
                (20, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (220, 220, 220),
                1,
            )
            cv2.putText(
                img,
                f"Status: Ready | Target: {fps} FPS",
                (20, h // 2 + 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (160, 160, 160),
                1,
            )
            cv2.putText(
                img,
                f"Time: {time.strftime('%H:%M:%S')}",
                (20, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (100, 100, 100),
                1,
            )
        return img

    def _set_status(self, **changes: Any) -> None:
        with self._lock:
            self._status.update(changes)

    def _record_capture_frame(self, topic: str) -> None:
        """Track source rate without forcing image decode/copy."""
        now = time.monotonic()
        with self._lock:
            self._frames_received += 1
            self._last_capture_at = now
            self._fps_counter += 1
            elapsed = now - self._last_fps_calc
            if elapsed < 1.0:
                return
            fps = round(self._fps_counter / elapsed, 1)
            self._last_fps_calc = now
            self._fps_counter = 0
            self._status.update(
                capture_fps=fps,
                topic=topic,
                state="running",
                error=None,
            )

    def _accept_frame(
        self,
        frame: Any,
        topic: str,
        *,
        count_capture: bool = True,
        arrival_ns: int | None = None,
        capture_ns: int | None = None,
    ) -> None:
        arrival_ns = time.monotonic_ns() if arrival_ns is None else arrival_ns
        capture_ns = arrival_ns if capture_ns is None else capture_ns
        frame = self._normalize_frame(frame)
        if frame is None:
            return
        with self._lock:
            if arrival_ns < self._frame_arrival_ns:
                return  # A late decoder must not overwrite a newer frame.
            self._latest = frame
            self._frame_sequence += 1
            self._frame_arrival_ns = arrival_ns
            self._frame_capture_ns = capture_ns
            waiters = list(self._frame_waiters)
        for loop, event in waiters:
            if not loop.is_closed():
                loop.call_soon_threadsafe(event.set)
        if count_capture:
            self._record_capture_frame(topic)

    def _record_decoded_frame(self, decode_ms: float) -> None:
        """Track frames which completed JPEG/PNG decoding for WebRTC."""
        now = time.monotonic()
        with self._lock:
            self._last_decoded_at = now
            self._decoded_fps_counter += 1
            elapsed = now - self._decoded_last_fps_calc
            self._status["jpeg_decode_ms"] = round(float(decode_ms), 2)
            if elapsed >= 1.0:
                self._status["decoded_fps"] = round(
                    self._decoded_fps_counter / elapsed, 1
                )
                self._decoded_last_fps_calc = now
                self._decoded_fps_counter = 0

    def _normalize_frame(self, frame: Any) -> Any:
        """Make ROS frames match the configured WebRTC encoder dimensions."""
        if frame is None or cv2 is None:
            return frame
        target_w = max(2, int(self.config.get("width", 640) or 640))
        target_h = max(2, int(self.config.get("height", 400) or 400))
        # yuv420p requires even dimensions.
        target_w -= target_w % 2
        target_h -= target_h % 2
        height, width = frame.shape[:2]
        if width != target_w or height != target_h:
            interpolation = (
                cv2.INTER_AREA
                if width > target_w or height > target_h
                else cv2.INTER_LINEAR
            )
            frame = cv2.resize(frame, (target_w, target_h), interpolation=interpolation)
        if np is not None and not frame.flags.c_contiguous:
            frame = np.ascontiguousarray(frame)
        return frame

    def _record_webrtc_frame(self) -> None:
        """Count frames actually requested by the active WebRTC sender."""
        now = time.monotonic()
        with self._lock:
            self._last_send_at = now
            self._webrtc_fps_counter += 1
            self._webrtc_frames_sent += 1
            elapsed = now - self._webrtc_last_fps_calc
            if elapsed >= 1.0:
                self._status["webrtc_send_fps"] = round(
                    self._webrtc_fps_counter / elapsed, 1
                )
                self._status["webrtc_frames_sent"] = self._webrtc_frames_sent
                self._webrtc_last_fps_calc = now
                self._webrtc_fps_counter = 0

    def _reset_webrtc_rate(self) -> None:
        with self._lock:
            self._webrtc_last_fps_calc = time.monotonic()
            self._webrtc_fps_counter = 0
            self._webrtc_frames_sent = 0
            self._status["webrtc_send_fps"] = 0.0
            self._status["webrtc_frames_sent"] = 0
            self._status["webrtc_codec"] = ""
            self._status["webrtc_bitrate_mbps"] = 0.0
            self._status["webrtc_target_bitrate_mbps"] = 0.0
            self._status["webrtc_encode_ms"] = 0.0
            self._status["webrtc_encoded_frame_kb"] = 0.0
            self._status["webrtc_forced_keyframes"] = 0
            self._status["webrtc_packets_sent"] = 0
            self._status["webrtc_remote_packets_lost"] = 0
            self._status["webrtc_fraction_lost_pct"] = 0.0
            self._status["webrtc_rtt_ms"] = None

    async def _monitor_sender_stats(self, pc: Any, sender: Any) -> None:
        """Expose RTCP feedback so low headset FPS can be attributed correctly."""
        previous_bytes: int | None = None
        previous_at: float | None = None
        try:
            while pc in self._pcs and pc.connectionState not in ("failed", "closed"):
                report = await sender.getStats()
                outbound = next(
                    (item for item in report.values() if item.type == "outbound-rtp" and item.kind == "video"),
                    None,
                )
                remote = next(
                    (item for item in report.values() if item.type == "remote-inbound-rtp" and item.kind == "video"),
                    None,
                )
                now = time.monotonic()
                changes: dict[str, Any] = {}
                if outbound is not None:
                    sent_bytes = int(outbound.bytesSent)
                    if previous_bytes is not None and previous_at is not None and now > previous_at:
                        changes["webrtc_bitrate_mbps"] = round(
                            (sent_bytes - previous_bytes) * 8 / (now - previous_at) / 1_000_000,
                            3,
                        )
                    previous_bytes, previous_at = sent_bytes, now
                    changes["webrtc_packets_sent"] = int(outbound.packetsSent)
                if remote is not None:
                    changes.update(
                        webrtc_remote_packets_lost=int(remote.packetsLost),
                        # RTCP stores this as an unsigned 8-bit fraction of 256.
                        webrtc_fraction_lost_pct=round(
                            float(remote.fractionLost) * 100.0 / 256.0, 2
                        ),
                        webrtc_rtt_ms=(
                            round(float(remote.roundTripTime) * 1000, 1)
                            if remote.roundTripTime is not None
                            else None
                        ),
                    )
                encoder = getattr(sender, "_RTCRtpSender__encoder", None)
                if encoder is not None:
                    encoder_name = type(encoder).__name__
                    changes["webrtc_codec"] = (
                        "H264" if "H264" in encoder_name else "VP8" if "Vp8" in encoder_name else encoder_name
                    )
                    target_bitrate = getattr(encoder, "target_bitrate", None)
                    if target_bitrate is not None:
                        changes["webrtc_target_bitrate_mbps"] = round(
                            float(target_bitrate) / 1_000_000, 3
                        )
                    encode_ms = getattr(encoder, "average_encode_ms", None)
                    if encode_ms is not None:
                        changes["webrtc_encode_ms"] = round(float(encode_ms), 2)
                    encoded_bytes = getattr(encoder, "last_payload_bytes", None)
                    if encoded_bytes is not None:
                        changes["webrtc_encoded_frame_kb"] = round(
                            float(encoded_bytes) / 1024.0, 2
                        )
                    forced_keyframes = getattr(encoder, "forced_keyframes", None)
                    if forced_keyframes is not None:
                        changes["webrtc_forced_keyframes"] = int(forced_keyframes)
                if changes:
                    self._set_status(**changes)
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self._set_status(webrtc_error=f"stats error: {type(exc).__name__}: {exc}")

    def _decode_loop_ros(self) -> None:
        active_topic = self.config.get("custom_topic", "") or self.config.get(
            "topic", "/observation/cameras/head/color/compressed"
        )
        while not self._stop.is_set():
            self._decode_event.wait(0.5)
            self._decode_event.clear()
            with self._lock:
                payload = self._encoded_latest
                arrival_ns = self._encoded_arrival_ns
                capture_ns = self._encoded_capture_ns
                self._encoded_latest = None
            if not payload or np is None or cv2 is None:
                continue
            try:
                decode_started = time.perf_counter()
                buf = np.frombuffer(payload, dtype=np.uint8)
                frame = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
                if frame is not None and frame.ndim == 2 and frame.dtype == np.uint16:
                    # X1 depth is a lossless PNG in millimetres. Visualize only
                    # the display copy; ROS recording retains the original data.
                    valid = frame > 0
                    intensity = (255 * (1 - np.clip(frame.astype(np.float32) / 5000, 0, 1))).astype(np.uint8)
                    frame = cv2.applyColorMap(intensity, cv2.COLORMAP_TURBO)
                    frame[~valid] = 0
                elif frame is not None and frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif frame is not None and frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                if frame is not None:
                    self._record_decoded_frame(
                        (time.perf_counter() - decode_started) * 1000.0
                    )
                    self._accept_frame(
                        frame,
                        str(active_topic),
                        count_capture=False,
                        arrival_ns=arrival_ns,
                        capture_ns=capture_ns,
                    )
            except Exception as exc:
                self._set_status(error=f"decode error: {exc}")



    def _capture_loop_realsense(self) -> None:
        """Direct Intel RealSense pipeline capture fallback."""
        pipeline = None
        try:
            import pyrealsense2 as rs

            pipeline = rs.pipeline()
            rs_config = rs.config()
            rs_config.enable_stream(
                rs.stream.color,
                int(self.config.get("width", 640)),
                int(self.config.get("height", 480)),
                rs.format.bgr8,
                int(self.config.get("fps", 30)),
            )
            pipeline.start(rs_config)
            self._set_status(state="running", error=None)
            frames = 0
            window_start = time.perf_counter()
            while not self._stop.is_set():
                frame_set = pipeline.wait_for_frames(1000)
                color = frame_set.get_color_frame()
                if not color:
                    continue
                arrival_ns = time.monotonic_ns()
                image = np.ascontiguousarray(np.asanyarray(color.get_data())).copy()
                self._accept_frame(image, "realsense", arrival_ns=arrival_ns)
                frames += 1
                elapsed = time.perf_counter() - window_start
                if elapsed >= 1.0:
                    self._set_status(capture_fps=round(frames / elapsed, 1))
                    frames = 0
                    window_start = time.perf_counter()
        except Exception as exc:
            self._set_status(state="error", error=f"{type(exc).__name__}: {exc}")
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            if self.status()["state"] != "error":
                self._set_status(state="stopped")

    def _create_video_track(self):
        import av
        from aiortc import VideoStreamTrack
        from aiortc.mediastreams import MediaStreamError

        camera = self

        class LatestFrameTrack(VideoStreamTrack):
            def __init__(self):
                super().__init__()
                self._sequence = 0
                self._pts = 0
                self._period = 1.0 / max(1.0, float(camera.config.get("fps", 30) or 30))
                self._pts_step = max(1, round(90000 * self._period))
                self._next_send_at = None
                self._dimensions = None
                self._loop = asyncio.get_running_loop()
                self._frame_event = asyncio.Event()
                with camera._lock:
                    camera._frame_waiters.add((self._loop, self._frame_event))

            def stop(self):
                with camera._lock:
                    camera._frame_waiters.discard((self._loop, self._frame_event))
                self._frame_event.set()
                super().stop()

            async def recv(self):
                # Wait for the first real frame. Once started, send at a fixed
                # cadence and always sample the newest decoded image. A short
                # DDS/decode delay therefore repeats one frame instead of
                # exposing a 50/17 ms stop-and-catch-up pattern to the headset.
                while self.readyState == "live" and not camera._stop.is_set():
                    with camera._lock:
                        sequence = camera._frame_sequence
                        arrival_ns = camera._frame_arrival_ns
                        image = camera._latest
                    if image is not None and time.monotonic_ns() - arrival_ns <= 1_800_000_000:
                        break
                    self._frame_event.clear()
                    # Close the clear/read race: a decoder may publish between
                    # the snapshot above and Event.clear().
                    with camera._lock:
                        if camera._frame_sequence != self._sequence:
                            continue
                    await self._frame_event.wait()
                else:
                    raise MediaStreamError

                loop = asyncio.get_running_loop()
                if self._next_send_at is None:
                    self._next_send_at = loop.time()
                else:
                    self._next_send_at += self._period
                    delay = self._next_send_at - loop.time()
                    if delay > 0:
                        await asyncio.sleep(delay)
                    elif delay < -self._period:
                        # Never burst several late frames to catch up.
                        self._next_send_at = loop.time()

                with camera._lock:
                    sequence = camera._frame_sequence
                    arrival_ns = camera._frame_arrival_ns
                    image = camera._latest
                if image is None or time.monotonic_ns() - arrival_ns > 1_800_000_000:
                    self._next_send_at = None
                    return await self.recv()
                self._sequence = sequence
                pts = self._pts
                self._pts += self._pts_step
                h, w = image.shape[:2]
                if self._dimensions is None:
                    self._dimensions = (w, h)
                elif self._dimensions != (w, h):
                    image = cv2.resize(image, self._dimensions)
                frame = av.VideoFrame.from_ndarray(np.ascontiguousarray(image), format="bgr24")
                frame.pts = pts
                frame.time_base = Fraction(1, 90000)
                camera._record_webrtc_frame()
                return frame

        return LatestFrameTrack()

    async def offer(self, data: dict[str, Any]) -> dict[str, str]:
        if self.status()["state"] not in ("running", "starting"):
            raise RuntimeError(self.status().get("error") or "camera service is not running")
        try:
            import av
            from aiortc import (
                RTCPeerConnection,
                RTCRtpSender,
                RTCSessionDescription,
                VideoStreamTrack,
            )
        except ImportError as exc:
            raise RuntimeError(f"WebRTC dependencies unavailable: {exc}") from exc

        from .fast_h264 import install_fast_h264_encoder

        install_fast_h264_encoder()

        # Only exchange complete SDP: there is no trickle ICE endpoint.
        remote_candidates = sum(
            line.startswith("a=candidate:") for line in data.get("sdp", "").splitlines()
        )
        stream_id = self.config.get("id", "main")
        print(f"[WebRTC {stream_id}] offer candidates={remote_candidates}", flush=True)
        if remote_candidates == 0:
            message = "Offer has no ICE candidates; wait for ICE gathering before sending"
            self._set_status(webrtc_error=message, remote_ice_candidates=0)
            raise ValueError(message)

        # 清理旧连接，避免多会话冲突
        for old_pc in list(self._pcs):
            try:
                await old_pc.close()
            except Exception:
                pass
            self._pcs.discard(old_pc)

        remote = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
        pc = RTCPeerConnection()
        self._pcs.add(pc)
        self._reset_webrtc_rate()
        self._set_status(
            webrtc_state="connecting", ice_state="new", webrtc_error=None,
            remote_ice_candidates=remote_candidates, local_ice_candidates=0,
        )

        @pc.on("iceconnectionstatechange")
        def on_ice_state():
            print(f"[WebRTC {stream_id}] ICE -> {pc.iceConnectionState}", flush=True)
            if pc in self._pcs:
                self._set_status(ice_state=pc.iceConnectionState)
                if pc.iceConnectionState == "failed":
                    self._set_status(webrtc_error="ICE failed: check peer candidates and UDP connectivity")

        @pc.on("connectionstatechange")
        def on_conn_state():
            print(f"[WebRTC {stream_id}] peer -> {pc.connectionState}", flush=True)
            if pc in self._pcs:
                self._set_status(webrtc_state=pc.connectionState)
                if pc.connectionState == "failed":
                    self._set_status(webrtc_error="WebRTC connection failed; inspect ICE state and UDP connectivity")

        sender = pc.addTrack(self._create_video_track())
        stats_task = asyncio.create_task(self._monitor_sender_stats(pc, sender))
        self._stats_tasks.add(stats_task)
        stats_task.add_done_callback(self._stats_tasks.discard)

        try:
            transceiver = next(item for item in pc.getTransceivers() if item.sender == sender)
            requested_codec = str(self.config.get("codec", "H264")).lower()
            requested_mime = f"video/{requested_codec}"
            preferred = [
                codec
                for codec in RTCRtpSender.getCapabilities("video").codecs
                if codec.mimeType.lower() == requested_mime
            ]
            if not preferred:
                preferred = [
                    codec
                    for codec in RTCRtpSender.getCapabilities("video").codecs
                    if codec.mimeType.lower() in ("video/h264", "video/vp8")
                ]
            if preferred:
                transceiver.setCodecPreferences(preferred)
        except Exception:
            pass

        @pc.on("connectionstatechange")
        async def connection_state_changed() -> None:
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                self._pcs.discard(pc)
                if not self._pcs:
                    self._reset_webrtc_rate()

        try:
            await pc.setRemoteDescription(remote)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            local_candidates = sum(
                line.startswith("a=candidate:") for line in pc.localDescription.sdp.splitlines()
            )
            self._set_status(local_ice_candidates=local_candidates)
            print(f"[WebRTC {stream_id}] answer candidates={local_candidates}", flush=True)
            return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        except Exception as exc:
            self._set_status(webrtc_error=f"{type(exc).__name__}: {exc}")
            await pc.close()
            self._pcs.discard(pc)
            raise
