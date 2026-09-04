from __future__ import annotations

import asyncio
import io
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
      4. Dynamic fallback test pattern frame when awaiting video signal
    """

    def __init__(self, config: dict[str, Any], domain_id: int = 14):
        self.config = config
        self.domain_id = int(domain_id)
        self._latest: Any = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._decode_thread: threading.Thread | None = None
        self._decode_event = threading.Event()
        self._encoded_latest: bytes | None = None
        self._pcs: set[Any] = set()
        self._frames_received = 0
        self._last_fps_calc = time.monotonic()
        self._fps_counter = 0
        self._webrtc_last_fps_calc = time.monotonic()
        self._webrtc_fps_counter = 0
        self._webrtc_frames_sent = 0
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", False) else "starting",
            "source": config.get("source", "ros"),
            "topic": config.get("topic", "/hc_teleop/camera_head/color/compressed"),
            "custom_topic": config.get("custom_topic", ""),
            "capture_fps": 0.0,
            "webrtc_send_fps": 0.0,
            "webrtc_frames_sent": 0,
            "peers": 0,
            "error": None,
            "id": str(config.get("id", "main")),
            "name": str(config.get("name", "主相机")),
        }

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
        try:
            active_topic = self.config.get("custom_topic", "") or self.config.get("topic", "/hc_teleop/camera_head/color/compressed")
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
                self._accept_frame(frame, topic, count_capture=False)
        except Exception as exc:
            self._set_status(error=f"decode error: {exc}")

    async def stop(self) -> None:
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
            self._latest = None
            self._frames_received = 0
            self._status["state"] = "stopped"
            self._status["capture_fps"] = 0.0
            self._status["webrtc_send_fps"] = 0.0
            self._status["webrtc_frames_sent"] = 0
            self._status["peers"] = 0

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
        result.update(
            id=str(self.config.get("id", "main")),
            name=str(self.config.get("name", "主相机")),
            source=self.config.get("source", "ros"),
            topic=self.config.get("topic", "/hc_teleop/camera_head/color/compressed"),
            custom_topic=self.config.get("custom_topic", ""),
            width=self.config.get("width", 640),
            height=self.config.get("height", 400),
            target_fps=self.config.get("fps", 30),
            codec=self.config.get("codec", "H264"),
            peers=len(self._pcs),
        )
        return result

    def set_frame(self, image: Any) -> None:
        """Manually push a BGR numpy image frame."""
        image = self._normalize_frame(image)
        with self._lock:
            self._latest = image
            self._frames_received += 1

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
            topic = self.config.get("topic", "/hc_teleop/camera_head/color/compressed")
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
    ) -> None:
        frame = self._normalize_frame(frame)
        with self._lock:
            self._latest = frame
        if count_capture:
            self._record_capture_frame(topic)

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

    def _decode_loop_ros(self) -> None:
        active_topic = self.config.get("custom_topic", "") or self.config.get(
            "topic", "/hc_teleop/camera_head/color/compressed"
        )
        while not self._stop.is_set():
            self._decode_event.wait(0.5)
            self._decode_event.clear()
            with self._lock:
                payload = self._encoded_latest
                self._encoded_latest = None
            if not payload or np is None or cv2 is None:
                continue
            try:
                buf = np.frombuffer(payload, dtype=np.uint8)
                frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if frame is not None:
                    self._accept_frame(
                        frame,
                        str(active_topic),
                        count_capture=False,
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
                image = np.ascontiguousarray(np.asanyarray(color.get_data())).copy()
                with self._lock:
                    self._latest = image
                    self._frames_received += 1
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

        camera = self

        class LatestFrameTrack(VideoStreamTrack):
            kind = "video"

            def __init__(self):
                super().__init__()
                self._fixed_w = None
                self._fixed_h = None

            async def recv(self):
                pts, time_base = await self.next_timestamp()
                image = camera.latest()
                if image is None:
                    image = camera._generate_placeholder()
                try:
                    image = camera._normalize_frame(image)
                    h, w = image.shape[:2]
                    if self._fixed_w is None or self._fixed_h is None:
                        self._fixed_w = w
                        self._fixed_h = h
                    elif w != self._fixed_w or h != self._fixed_h:
                        if cv2 is not None:
                            image = cv2.resize(image, (self._fixed_w, self._fixed_h))
                    if not image.flags.c_contiguous:
                        image = np.ascontiguousarray(image)
                    frame = av.VideoFrame.from_ndarray(image, format="bgr24")
                    frame.pts = pts
                    frame.time_base = time_base
                    camera._record_webrtc_frame()
                    return frame
                except Exception as exc:
                    # In case of any encoding exception, return a safe blank frame matching fixed dimensions
                    target_w = self._fixed_w or 640
                    target_h = self._fixed_h or 480
                    blank = np.zeros((target_h, target_w, 3), dtype=np.uint8)
                    frame = av.VideoFrame.from_ndarray(blank, format="bgr24")
                    frame.pts = pts
                    frame.time_base = time_base
                    camera._record_webrtc_frame()
                    return frame

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

        @pc.on("iceconnectionstatechange")
        def on_ice_state():
            print(f"[WebRTC ICE] State -> {pc.iceConnectionState}", flush=True)

        @pc.on("connectionstatechange")
        def on_conn_state():
            print(f"[WebRTC Peer] State -> {pc.connectionState}", flush=True)

        sender = pc.addTrack(LatestFrameTrack())

        try:
            transceiver = next(item for item in pc.getTransceivers() if item.sender == sender)
            requested_codec = str(camera.config.get("codec", "H264")).lower()
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

        await pc.setRemoteDescription(remote)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
