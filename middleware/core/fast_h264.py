from __future__ import annotations

import fractions
import time

import av
from aiortc import codecs as aiortc_codecs
from aiortc.codecs import h264


class FastH264Encoder(h264.H264Encoder):
    """aiortc H.264 encoder tuned for low-latency ARM teleoperation."""

    # REMB estimates can move by more than 10% on every feedback cycle.  The
    # stock aiortc encoder responds by recreating libx264, which emits another
    # large IDR frame.  On a headset this becomes a self-reinforcing loop:
    # IDR burst -> queue/loss -> lower REMB/PLI -> another IDR burst.  Only
    # reopen for a material, sustained bitrate change.
    BITRATE_REOPEN_THRESHOLD = 0.50
    BITRATE_REOPEN_INTERVAL = 15.0

    def __init__(self) -> None:
        super().__init__()
        self._configured_bitrate: int | None = None
        self._last_reopen_at = 0.0

    def _close_codec(self) -> None:
        self.buffer_data = b""
        self.buffer_pts = None
        self.codec = None
        self._configured_bitrate = None

    def _bitrate_reopen_due(self, now: float) -> bool:
        if self._configured_bitrate is None:
            return False
        relative_change = (
            abs(self.target_bitrate - self._configured_bitrate)
            / self._configured_bitrate
        )
        return (
            relative_change >= self.BITRATE_REOPEN_THRESHOLD
            and now - self._last_reopen_at >= self.BITRATE_REOPEN_INTERVAL
        )

    def _encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
        now = time.monotonic()
        if self.codec and (
            frame.width != self.codec.width
            or frame.height != self.codec.height
            or self._bitrate_reopen_due(now)
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
            self._last_reopen_at = now

        data_to_send = b""
        for package in self.codec.encode(frame):
            data_to_send += bytes(package)
        if data_to_send:
            yield from self._split_bitstream(data_to_send)


def install_fast_h264_encoder() -> None:
    """Install the fast encoder before aiortc creates an RTP sender."""
    if aiortc_codecs.H264Encoder is not FastH264Encoder:
        aiortc_codecs.H264Encoder = FastH264Encoder
