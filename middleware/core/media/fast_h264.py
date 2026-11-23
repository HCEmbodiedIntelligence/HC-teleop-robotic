from __future__ import annotations

from contextvars import ContextVar
import fractions
import time

import av
from aiortc import codecs as aiortc_codecs
from aiortc.codecs import h264


# aiortc constructs its encoder in the RTP task before dispatching encode() to
# an executor. Task-local selection avoids cross-camera/global configuration.
encoder_backend: ContextVar[str] = ContextVar("teleop_encoder_backend", default="x264")


class FastH264Encoder(h264.H264Encoder):
    """Low latency H.264 with bounded feedback reconfiguration and fallback."""

    reconfigure_interval = 2.0
    reconfigure_threshold = 0.20

    def __init__(self) -> None:
        super().__init__()
        self.requested_backend = encoder_backend.get()
        if self.requested_backend not in ("auto", "x264", "nvenc", "jetson"):
            raise ValueError(f"Unknown encoder backend: {self.requested_backend}")
        self.actual_backend = ""
        self.fallback_reason: str | None = None
        self._failed_backends = set()
        self.dynamic_bitrate_updates = 0
        self._configured_bitrate: int | None = None
        self._last_configuration = float("-inf")
        self.reconfigurations = 0
        self.average_encode_ms = 0.0
        self.last_payload_bytes = 0
        self.frames_encoded = 0
        self.force_keyframe_requests = 0
        self.forced_keyframes = 0

    @property
    def configured_bitrate(self) -> int | None:
        return self._configured_bitrate

    def _close_codec(self) -> None:
        self.buffer_data = b""
        self.buffer_pts = None
        codec = self.codec
        self.codec = None
        if codec is not None and callable(getattr(codec, "close", None)):
            codec.close()
        self._configured_bitrate = None

    def _create_codec(self, frame, backend):
        bitrate = int(self.target_bitrate)
        if backend == "jetson":
            from .jetson_h264 import JetsonH264Codec
            return JetsonH264Codec(frame.width, frame.height, bitrate, h264.MAX_FRAME_RATE)
        codec = av.CodecContext.create("h264_nvenc" if backend == "nvenc" else "libx264", "w")
        codec.width = frame.width
        codec.height = frame.height
        codec.bit_rate = bitrate
        codec.pix_fmt = "yuv420p"
        codec.framerate = fractions.Fraction(h264.MAX_FRAME_RATE, 1)
        codec.time_base = fractions.Fraction(1, 90000)
        codec.profile = "Baseline"
        if backend == "nvenc":
            codec.options = {
                "preset": "p1", "tune": "ull", "rc": "cbr",
                "zerolatency": "1", "bf": "0", "rc-lookahead": "0",
                "delay": "0", "forced-idr": "1", "level": "3.1",
                "maxrate": str(bitrate), "bufsize": str(max(1, bitrate // 4)),
            }
        else:
            codec.options = {
                "level": "31", "preset": "ultrafast", "tune": "zerolatency",
                "x264-params": (
                    "keyint=300:min-keyint=60:scenecut=0:intra-refresh=1:"
                    "bframes=0:ref=1:sliced-threads=1:slice-max-size=1100:"
                    f"vbv-maxrate={max(1, bitrate // 1000)}:"
                    f"vbv-bufsize={max(1, bitrate // 4000)}"
                ),
            }
        return codec

    def _select_backend(self):
        if self.requested_backend == "auto":
            from .jetson_h264 import jetson_platform
            candidates = (["jetson"] if jetson_platform()["is_jetson"] else []) + ["nvenc", "x264"]
        elif self.requested_backend in ("jetson", "nvenc"):
            candidates = [self.requested_backend, "x264"]
        else:
            candidates = ["x264"]
        return next(backend for backend in candidates if backend not in self._failed_backends)

    def _hardware_failure(self, backend, phase, exc):
        self._failed_backends.add(backend)
        reason = f"{backend.upper()} {phase} failed: {type(exc).__name__}: {exc}"
        self.fallback_reason = f"{self.fallback_reason}; {reason}" if self.fallback_reason else reason

    def close(self):
        self._close_codec()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def _encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
        encode_started = time.perf_counter()
        if force_keyframe:
            self.force_keyframe_requests += 1
            self.forced_keyframes += 1
        now = time.monotonic()
        dimensions_changed = bool(self.codec and (
            frame.width != self.codec.width or frame.height != self.codec.height
        ))
        # Updating PyAV's bit_rate property alone does not reliably reconfigure
        # libx264's live rate control. Apply real changes by rebuilding, bounded
        # in frequency and size. Tight VBV bounds the resulting recovery burst.
        bitrate_changed = bool(self._configured_bitrate and
            abs(int(self.target_bitrate) - self._configured_bitrate) / self._configured_bitrate >= self.reconfigure_threshold and
            now - self._last_configuration >= self.reconfigure_interval)
        if bitrate_changed and not dimensions_changed and self.actual_backend == "jetson":
            try:
                if self.codec.set_bitrate(int(self.target_bitrate)):
                    self._configured_bitrate = int(self.codec.bit_rate)
                    self._last_configuration = now
                    self.dynamic_bitrate_updates += 1
                    bitrate_changed = False
            except Exception as exc:
                self._hardware_failure("jetson", "bitrate update", exc)
                self._close_codec()
        if dimensions_changed or bitrate_changed:
            self._close_codec()
            self.reconfigurations += 1

        frame.pict_type = av.video.frame.PictureType.I if force_keyframe else av.video.frame.PictureType.NONE
        if self.codec is not None:
            try:
                data_to_send = b"".join(bytes(packet) for packet in self.codec.encode(frame))
            except Exception as exc:
                if self.actual_backend not in ("nvenc", "jetson"):
                    raise
                self._hardware_failure(self.actual_backend, "encode", exc)
                self._close_codec()
                frame.pict_type = av.video.frame.PictureType.I
                self.reconfigurations += 1
        if self.codec is None:
            while True:
                backend = self._select_backend()
                codec = None
                try:
                    codec = self._create_codec(frame, backend)
                    # Real first-frame probe. Its output is retained rather
                    # than discarding an AU or shifting RTP frame timestamps.
                    data_to_send = b"".join(bytes(packet) for packet in codec.encode(frame))
                    if not data_to_send:
                        raise RuntimeError("low-latency encoder buffered the first frame")
                    break
                except Exception as exc:
                    if codec is not None and callable(getattr(codec, "close", None)):
                        codec.close()
                    if backend == "x264":
                        raise
                    self._hardware_failure(backend, "probe", exc)
            self.codec = codec
            self.actual_backend = backend
            self._configured_bitrate = int(codec.bit_rate)
            self._last_configuration = now
        elapsed_ms = (time.perf_counter() - encode_started) * 1000.0
        self.average_encode_ms = elapsed_ms if not self.frames_encoded else self.average_encode_ms * 0.9 + elapsed_ms * 0.1
        self.last_payload_bytes = len(data_to_send)
        self.frames_encoded += 1
        if data_to_send:
            yield from self._split_bitstream(data_to_send)


def install_fast_h264_encoder() -> None:
    """Install once; backend selection belongs to each RTP task's context."""
    if aiortc_codecs.H264Encoder is not FastH264Encoder:
        aiortc_codecs.H264Encoder = FastH264Encoder
