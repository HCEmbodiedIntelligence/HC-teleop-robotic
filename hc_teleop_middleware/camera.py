from __future__ import annotations

import asyncio
import threading
import time
from typing import Any


class CameraService:
    """Optional RealSense latest-frame capture and aiortc peer management."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._pcs: set[Any] = set()
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", False) else "starting",
            "capture_fps": 0.0,
            "peers": 0,
            "error": None,
        }

    def start(self) -> None:
        if not self.config.get("enabled", False):
            return
        self._thread = threading.Thread(target=self._capture_loop, name="d435", daemon=True)
        self._thread.start()

    async def stop(self) -> None:
        await asyncio.gather(*(pc.close() for pc in list(self._pcs)), return_exceptions=True)
        self._pcs.clear()
        self._stop.set()
        if self._thread is not None:
            await asyncio.to_thread(self._thread.join, 3)

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
        result.update(
            width=self.config.get("width"),
            height=self.config.get("height"),
            target_fps=self.config.get("fps"),
            codec=self.config.get("codec", "H264"),
            peers=len(self._pcs),
        )
        return result

    def latest(self):
        with self._lock:
            return self._latest

    def _set_status(self, **changes: Any) -> None:
        with self._lock:
            self._status.update(changes)

    def _capture_loop(self) -> None:
        pipeline = None
        try:
            import numpy as np
            import pyrealsense2 as rs

            pipeline = rs.pipeline()
            rs_config = rs.config()
            rs_config.enable_stream(
                rs.stream.color,
                int(self.config["width"]),
                int(self.config["height"]),
                rs.format.bgr8,
                int(self.config["fps"]),
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
                pipeline.stop()
            if self.status()["state"] != "error":
                self._set_status(state="stopped")

    async def offer(self, data: dict[str, Any]) -> dict[str, str]:
        if self.status()["state"] != "running":
            raise RuntimeError(self.status().get("error") or "camera is not running")
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

        camera = self

        class LatestFrameTrack(VideoStreamTrack):
            kind = "video"

            async def recv(self):
                pts, time_base = await self.next_timestamp()
                image = camera.latest()
                while image is None:
                    await asyncio.sleep(0.01)
                    image = camera.latest()
                frame = av.VideoFrame.from_ndarray(image, format="bgr24")
                frame.pts = pts
                frame.time_base = time_base
                return frame

        remote = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
        pc = RTCPeerConnection()
        self._pcs.add(pc)
        sender = pc.addTrack(LatestFrameTrack())
        transceiver = next(item for item in pc.getTransceivers() if item.sender == sender)
        h264 = [
            codec
            for codec in RTCRtpSender.getCapabilities("video").codecs
            if codec.mimeType.lower() == "video/h264"
        ]
        if not h264:
            await pc.close()
            self._pcs.discard(pc)
            raise RuntimeError("this PyAV/aiortc build has no H.264 encoder")
        transceiver.setCodecPreferences(h264)

        @pc.on("connectionstatechange")
        async def connection_state_changed() -> None:
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                self._pcs.discard(pc)

        await pc.setRemoteDescription(remote)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
