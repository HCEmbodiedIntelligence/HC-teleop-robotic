from __future__ import annotations

import fractions

import av
from aiortc import codecs as aiortc_codecs
from aiortc.codecs import h264


class FastH264Encoder(h264.H264Encoder):
    """aiortc H.264 encoder tuned for low-latency ARM teleoperation."""

    def _encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
        if self.codec and (
            frame.width != self.codec.width
            or frame.height != self.codec.height
            or abs(self.target_bitrate - self.codec.bit_rate) / self.codec.bit_rate > 0.1
        ):
            self.buffer_data = b""
            self.buffer_pts = None
            self.codec = None

        frame.pict_type = (
            av.video.frame.PictureType.I
            if force_keyframe
            else av.video.frame.PictureType.NONE
        )

        if self.codec is None:
            self.codec = av.CodecContext.create("libx264", "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = self.target_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.framerate = fractions.Fraction(h264.MAX_FRAME_RATE, 1)
            self.codec.time_base = fractions.Fraction(1, h264.MAX_FRAME_RATE)
            self.codec.options = {
                "level": "31",
                "preset": "ultrafast",
                "tune": "zerolatency",
            }
            self.codec.profile = "Baseline"

        data_to_send = b""
        for package in self.codec.encode(frame):
            data_to_send += bytes(package)
        if data_to_send:
            yield from self._split_bitstream(data_to_send)


def install_fast_h264_encoder() -> None:
    """Install the fast encoder before aiortc creates an RTP sender."""
    if aiortc_codecs.H264Encoder is not FastH264Encoder:
        aiortc_codecs.H264Encoder = FastH264Encoder
