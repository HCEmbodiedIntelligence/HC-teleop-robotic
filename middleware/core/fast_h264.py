from __future__ import annotations

import fractions
import time

import av
from aiortc import codecs as aiortc_codecs
from aiortc.codecs import h264


class FastH264Encoder(h264.H264Encoder):
    """aiortc H.264 encoder tuned for low-latency ARM teleoperation."""

    # REMB is receiver feedback, not a request to reset the video codec.  A
    # short headset stall during MCAP finalization can drive REMB to aiortc's
    # 500 kbps floor. Reopening x264 at that value emits a new IDR and traps
    # the stream in a low-FPS / high-RTT feedback loop. Keep one codec context
    # for the complete peer session; only a dimension change requires reopen.

    def __init__(self) -> None:
        super().__init__()
        self._configured_bitrate: int | None = None
        self.average_encode_ms = 0.0
        self.last_payload_bytes = 0
        self.frames_encoded = 0

    def _close_codec(self) -> None:
        self.buffer_data = b""
        self.buffer_pts = None
        self.codec = None
        self._configured_bitrate = None

    def _encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
        encode_started = time.perf_counter()
        if self.codec and (
            frame.width != self.codec.width
            or frame.height != self.codec.height
        ):
            self._close_codec()

        frame.pict_type = (
            av.video.frame.PictureType.I
            if force_keyframe
            else av.video.frame.PictureType.NONE
        )

        if self.codec is None:
            configured_bitrate = int(self.target_bitrate)
            maxrate_kbps = max(1, configured_bitrate // 1000)
            # A 250 ms VBV bounds one access-unit burst without adding a deep
            # encoder queue.  x264 may otherwise spend more than a second of
            # the target bitrate on one IDR frame.
            vbv_buffer_kbits = max(1, configured_bitrate // 4000)
            self.codec = av.CodecContext.create("libx264", "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = configured_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.framerate = fractions.Fraction(h264.MAX_FRAME_RATE, 1)
            self.codec.time_base = fractions.Fraction(1, h264.MAX_FRAME_RATE)
            self.codec.options = {
                "level": "31",
                "preset": "ultrafast",
                "tune": "zerolatency",
                # Periodic intra refresh and a tight VBV spread recovery work
                # across frames instead of sending a 100-200 KB IDR burst
                # every 60 frames. PLI/FIR can still request an immediate IDR.
                "x264-params": (
                    "keyint=300:min-keyint=60:scenecut=0:intra-refresh=1:"
                    "bframes=0:ref=1:sliced-threads=1:slice-max-size=1100:"
                    f"vbv-maxrate={maxrate_kbps}:vbv-bufsize={vbv_buffer_kbits}"
                ),
            }
            self.codec.profile = "Baseline"
            self._configured_bitrate = configured_bitrate

        data_to_send = b""
        for package in self.codec.encode(frame):
            data_to_send += bytes(package)
        elapsed_ms = (time.perf_counter() - encode_started) * 1000.0
        self.average_encode_ms = (
            elapsed_ms
            if not self.frames_encoded
            else self.average_encode_ms * 0.9 + elapsed_ms * 0.1
        )
        self.last_payload_bytes = len(data_to_send)
        self.frames_encoded += 1
        if data_to_send:
            yield from self._split_bitstream(data_to_send)


def install_fast_h264_encoder() -> None:
    """Install the fast encoder before aiortc creates an RTP sender."""
    if aiortc_codecs.H264Encoder is not FastH264Encoder:
        aiortc_codecs.H264Encoder = FastH264Encoder
