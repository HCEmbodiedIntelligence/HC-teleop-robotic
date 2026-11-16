"""aiortc 1.15.0 encoder policy; the only sender-private integration point.

Hardware support is established by successfully encoding a real frame, never
by finding a codec name. All hardware failures retain their actual error.
"""

import fractions
import time
from typing import Callable

import av
import aiortc
from aiortc.codecs.h264 import H264Encoder

AIORTC_VERSION = "1.15.0"


class AdaptiveH264Encoder(H264Encoder):
    def __init__(self, requested="x264", fps=30, bitrate=2_000_000, on_status=None):
        super().__init__()
        aliases = {
            "x264": "libx264",
            "software": "libx264",
            "nvenc": "h264_nvenc",
            "v4l2": "h264_v4l2m2m",
            "v4l2m2m": "h264_v4l2m2m",
            "vaapi": "h264_vaapi",
        }
        self.requested = requested
        requested = aliases.get(requested, requested)
        if requested not in (
            "auto",
            "libx264",
            "h264_nvenc",
            "h264_v4l2m2m",
            "h264_vaapi",
        ):
            raise ValueError("unsupported encoder backend")
        self.candidates = (
            ["h264_nvenc", "h264_v4l2m2m", "libx264"]
            if requested == "auto"
            else ([requested, "libx264"] if requested != "libx264" else ["libx264"])
        )
        self.index = 0
        self.fps = fractions.Fraction(str(float(fps))).limit_denominator(1001)
        self.target_bitrate = int(bitrate)
        self.on_status = on_status
        self.failures = []
        self.actual = None
        self.frames_encoded = 0
        self.empty_frames = 0
        self.last_error = None
        self._emit()

    def _emit(self):
        if self.on_status:
            self.on_status(
                {
                    "requested": self.requested,
                    "actual": self.actual,
                    "hardware": self.actual in ("h264_nvenc", "h264_v4l2m2m"),
                    "fallback_reason": "; ".join(self.failures) or None,
                    "frames_encoded": self.frames_encoded,
                    "last_encoded_mono": time.monotonic()
                    if self.frames_encoded
                    else None,
                    "width": self.codec.width if self.codec else None,
                    "height": self.codec.height if self.codec else None,
                    "target_fps": float(self.fps),
                    "bitrate": self.target_bitrate,
                    "error": self.last_error,
                }
            )

    def _create(self, name, frame):
        if name == "h264_vaapi":
            raise RuntimeError(
                "VAAPI upload/device context is not implemented by this pinned aiortc adapter"
            )
        codec = av.CodecContext.create(name, "w")
        codec.width, codec.height = frame.width, frame.height
        codec.bit_rate = self.target_bitrate
        codec.pix_fmt = "yuv420p"
        codec.framerate = self.fps
        codec.time_base = 1 / self.fps
        codec.gop_size = max(1, int(float(self.fps) * 2))
        codec.max_b_frames = 0
        if name == "libx264":
            codec.options = {
                "level": "31",
                "tune": "zerolatency",
                "preset": "ultrafast",
            }
            codec.profile = "Baseline"
        elif name == "h264_nvenc":
            codec.options = {
                "preset": "p1",
                "tune": "ull",
                "rc": "cbr",
                "zerolatency": "1",
                "delay": "0",
                "rc-lookahead": "0",
                "forced-idr": "1",
                "level": "3.1",
            }
            codec.profile = "Baseline"
        codec.open()
        return codec

    def _encode_frame(self, frame, force_keyframe):
        if self.codec and (
            frame.width != self.codec.width
            or frame.height != self.codec.height
            or abs(self.target_bitrate - self.codec.bit_rate)
            / max(self.codec.bit_rate, 1)
            > 0.1
        ):
            self.codec = None
            self.actual = None
            self.empty_frames = 0
        frame.pict_type = (
            av.video.frame.PictureType.I
            if force_keyframe
            else av.video.frame.PictureType.NONE
        )
        while self.index < len(self.candidates):
            name = self.candidates[self.index]
            try:
                if self.codec is None:
                    self.codec = self._create(name, frame)
                packets = self.codec.encode(frame)
                bitstream = b"".join(bytes(packet) for packet in packets)
                if not bitstream:
                    self.empty_frames += 1
                    if self.empty_frames > 2:
                        raise RuntimeError(
                            "encoder exceeded the two-frame latency budget without output"
                        )
                    return
                nalus = list(self._split_bitstream(bitstream))
                if not nalus:
                    raise RuntimeError(
                        "encoder did not produce an Annex-B H264 bitstream"
                    )
                self.actual = name
                self.empty_frames = 0
                self.frames_encoded += 1
                self.last_error = None
                self._emit()
                yield from nalus
                return
            except Exception as error:
                self.failures.append(
                    name + ": " + type(error).__name__ + ": " + str(error)[:300]
                )
                self.codec = None
                self.actual = None
                self.empty_frames = 0
                self.index += 1
                self.last_error = str(error)[:300]
                self._emit()
                frame.pict_type = av.video.frame.PictureType.I
        raise RuntimeError(
            "All H264 encoder candidates failed: " + "; ".join(self.failures)
        )


def install_sender_encoder(
    sender, *, requested="x264", fps=30, bitrate=2_000_000, on_status=None, on_sent=None
):
    """Version-pinned private hook, kept out of camera and application logic."""
    if aiortc.__version__ != AIORTC_VERSION:
        raise RuntimeError("Encoder integration requires aiortc==" + AIORTC_VERSION)
    attribute = "_RTCRtpSender__encoder"
    if not hasattr(sender, attribute):
        raise RuntimeError("aiortc sender encoder hook changed")
    encoder = AdaptiveH264Encoder(requested, fps, bitrate, on_status)
    setattr(sender, attribute, encoder)
    if on_sent is not None:
        # Count complete original frames after DTLS transport accepted the RTP
        # marker packet. Retransmissions have a distinct SSRC and are excluded.
        # This hook, like the encoder hook above, is pinned to aiortc 1.15.0.
        transport = sender.transport
        callbacks = getattr(transport, "_hc_frame_callbacks", None)
        if callbacks is None:
            callbacks = {}
            transport._hc_frame_callbacks = callbacks
            original = transport._send_rtp

            async def send_and_observe(data):
                await original(data)
                if (
                    len(data) >= 12
                    and data[0] >> 6 == 2
                    and data[1] & 128
                    and not 192 <= data[1] <= 223
                ):
                    callback = callbacks.get(int.from_bytes(data[8:12], "big"))
                    if callback is not None:
                        callback(int.from_bytes(data[4:8], "big"), len(data))

            transport._send_rtp = send_and_observe
        callbacks[sender._ssrc] = on_sent
    return encoder


def h264_preferences(capabilities):
    """RTX is a negotiated repair codec, not a second video encoding format."""
    codecs = [
        codec for codec in capabilities.codecs if codec.mimeType.lower() == "video/h264"
    ]
    codecs.extend(
        codec for codec in capabilities.codecs if codec.mimeType.lower() == "video/rtx"
    )
    if not codecs or not any(
        codec.mimeType.lower() == "video/h264" for codec in codecs
    ):
        raise RuntimeError("H264 capability unavailable")
    return codecs


def negotiated_repair(sdp):
    from aiortc.sdp import SessionDescription

    result = {
        "nack": False,
        "pli": False,
        "rtx": False,
        "fec": False,
        "fec_reason": "RED/ULPFEC sender is not implemented",
    }
    for media in SessionDescription.parse(sdp).media:
        if media.kind != "video":
            continue
        h264 = {
            codec.payloadType
            for codec in media.rtp.codecs
            if codec.mimeType.lower() == "video/h264"
        }
        for codec in media.rtp.codecs:
            if (
                codec.mimeType.lower() == "video/rtx"
                and codec.parameters.get("apt") in h264
            ):
                result["rtx"] = True
            if codec.payloadType in h264:
                for feedback in codec.rtcpFeedback:
                    if feedback.type == "nack":
                        if feedback.parameter == "pli":
                            result["pli"] = True
                        elif not feedback.parameter:
                            result["nack"] = True
    return result
