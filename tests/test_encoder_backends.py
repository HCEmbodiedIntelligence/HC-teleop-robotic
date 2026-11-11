"""Real x264 rate-control regression plus deterministic hardware failures."""
import asyncio
from fractions import Fraction
import unittest
from unittest.mock import patch

import av
import numpy as np

from middleware.core.fast_h264 import FastH264Encoder, encoder_backend


def frame(image, index):
    result = av.VideoFrame.from_ndarray(image, format="bgr24")
    result.pts = index * 3000
    result.time_base = Fraction(1, 90000)
    return result


class EncoderBackendsTests(unittest.TestCase):
    def test_feedback_reduces_real_output_without_reopening_on_every_frame(self):
        encoder = FastH264Encoder()
        encoder.target_bitrate = 2_500_000
        rng = np.random.default_rng(17)
        sizes = []
        with patch("middleware.core.fast_h264.time.monotonic") as clock:
            for index in range(180):
                clock.return_value = index / 30
                if index == 90:
                    encoder.target_bitrate = 500_000
                image = rng.integers(0, 256, (400, 640, 3), dtype=np.uint8)
                sizes.append(sum(map(len, encoder._encode_frame(frame(image, index), False))))
        high = sum(sizes[30:80]) * 8 * 30 / 50
        low = sum(sizes[120:170]) * 8 * 30 / 50
        self.assertGreater(high, 1_500_000)
        self.assertLess(low, high * 0.4)
        self.assertLess(low, 700_000)
        self.assertEqual(encoder.configured_bitrate, 500_000)
        self.assertEqual(encoder.reconfigurations, 1)

    def test_small_feedback_does_not_reopen(self):
        encoder = FastH264Encoder()
        image = np.zeros((400, 640, 3), dtype=np.uint8)
        list(encoder._encode_frame(frame(image, 0), False))
        codec = encoder.codec
        encoder._last_configuration = -100
        encoder.target_bitrate = int(encoder.configured_bitrate * 0.9)
        list(encoder._encode_frame(frame(image, 1), False))
        self.assertIs(codec, encoder.codec)

    def test_nvenc_probe_failure_falls_back_to_real_software_encoding(self):
        token = encoder_backend.set("nvenc")
        try:
            encoder = FastH264Encoder()
        finally:
            encoder_backend.reset(token)
        original = encoder._create_codec
        attempts = []

        def create(image, backend):
            attempts.append(backend)
            if backend == "nvenc":
                raise RuntimeError("no GPU")
            return original(image, backend)

        with patch.object(encoder, "_create_codec", side_effect=create):
            payloads = list(encoder._encode_frame(frame(np.zeros((400, 640, 3), np.uint8), 0), False))
        self.assertTrue(payloads)
        self.assertEqual(attempts, ["nvenc", "x264"])
        self.assertEqual(encoder.requested_backend, "nvenc")
        self.assertEqual(encoder.actual_backend, "x264")
        self.assertIn("no GPU", encoder.fallback_reason)

    def test_nvenc_device_failure_after_probe_recovers_with_software(self):
        token = encoder_backend.set("nvenc")
        try:
            encoder = FastH264Encoder()
        finally:
            encoder_backend.reset(token)
        original = encoder._create_codec

        class Device:
            width, height, bit_rate = 640, 400, 1_000_000

            def __init__(self):
                self.codec = None

            def encode(self, image):
                if self.codec is not None:
                    raise RuntimeError("device disconnected")
                self.codec = original(image, "x264")
                self.bit_rate = self.codec.bit_rate
                return self.codec.encode(image)

        def create(image, backend):
            return Device() if backend == "nvenc" else original(image, backend)

        image = np.zeros((400, 640, 3), np.uint8)
        with patch.object(encoder, "_create_codec", side_effect=create):
            self.assertTrue(list(encoder._encode_frame(frame(image, 0), False)))
            self.assertEqual(encoder.actual_backend, "nvenc")
            self.assertTrue(list(encoder._encode_frame(frame(image, 1), False)))
        self.assertEqual(encoder.actual_backend, "x264")
        self.assertIn("device disconnected", encoder.fallback_reason)

    def test_task_contexts_do_not_leak_backend_selection(self):
        async def create(backend):
            token = encoder_backend.set(backend)
            try:
                await asyncio.sleep(0)
                return FastH264Encoder().requested_backend
            finally:
                encoder_backend.reset(token)

        async def run():
            return await asyncio.gather(create("x264"), create("nvenc"))

        self.assertEqual(asyncio.run(run()), ["x264", "nvenc"])
        self.assertEqual(encoder_backend.get(), "x264")

    def test_jetson_on_non_jetson_host_falls_back_with_guidance(self):
        token = encoder_backend.set("jetson")
        try:
            encoder = FastH264Encoder()
        finally:
            encoder_backend.reset(token)
        with patch('middleware.core.jetson_h264.jetson_platform', return_value={'is_jetson': False}):
            list(encoder._encode_frame(frame(np.zeros((400, 640, 3), np.uint8), 0), False))
        self.assertEqual(encoder.actual_backend, "x264")
        self.assertIn("GStreamer", encoder.fallback_reason)
