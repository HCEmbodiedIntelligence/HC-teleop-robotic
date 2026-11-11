"""Jetson native pipeline contract tests; real hardware test is opt-in."""
from fractions import Fraction
import os
import re
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import av
import numpy as np

from middleware.core.jetson_h264 import JetsonH264Codec, validate_access_unit, jetson_platform
from middleware.core.fast_h264 import FastH264Encoder, encoder_backend


IDR = b'\x00\x00\x00\x01\x67\x42\xc0\x1f\x00\x00\x01\x68\x00\x00\x00\x01\x65\x01'
DELTA = b'\x00\x00\x01\x41\x01'


def video_frame(index=0, force=False, width=64, height=48):
    frame = av.VideoFrame.from_ndarray(np.zeros((height, width, 3), np.uint8), format='bgr24')
    frame.pts = index * 3000
    frame.time_base = Fraction(1, 90000)
    if force:
        frame.pict_type = av.video.frame.PictureType.I
    return frame


class Buffer:
    def __init__(self, data=b''):
        self.data = data
        self.pts = 0
    def fill(self, offset, data): self.data = data
    def extract_dup(self, offset, length): return self.data[offset:offset+length]
    def get_size(self): return len(self.data)
    @classmethod
    def new_allocate(cls, *_): return cls()


class Pipeline:
    def __init__(self):
        self.states = []
        self.properties = {}
        self.buffer = None
        self.timeout = False
        self.first = True
        self.force = False
        self.mutable = True
        self.native_force_supported = True
        self.native_force_calls = 0
        self.standard_force_calls = 0
        self.bus_error = None
        self.source = NS(emit=self.push)
        self.sink = NS(emit=self.pull)
        self.encoder = NS(find_property=self.find_property, set_property=self.set_property,
                          emit=self.native_force, get_property=lambda name: self.properties.get(name),
                          get_static_pad=lambda _: NS(send_event=self.event))
    def find_property(self, name): return NS(flags=4096 if self.mutable else 0)
    def set_property(self, name, value): self.properties[name] = value
    def get_by_name(self, name): return getattr(self, name)
    def get_bus(self): return NS(have_pending=lambda: bool(self.bus_error), timed_pop_filtered=lambda *_: self.bus_error)
    def set_state(self, state): self.states.append(state); return 'success'
    def native_force(self, signal):
        assert signal == 'force-IDR'
        self.native_force_calls += 1
        if not self.native_force_supported:
            raise TypeError('unknown signal name: force-IDR')
        self.force = True
    def event(self, event):
        self.standard_force_calls += 1
        self.force = True
        return True
    def push(self, signal, buffer):
        assert signal == 'push-buffer'
        self.buffer = buffer
        return 'ok'
    def pull(self, signal, timeout):
        assert signal == 'try-pull-sample' and timeout > 0
        if self.timeout: return None
        data = IDR if self.first or self.force else DELTA
        self.first = self.force = False
        result = Buffer(data)
        result.pts = self.buffer.pts
        return NS(get_buffer=lambda: result)


class JetsonH264Tests(unittest.TestCase):
    def setUp(self):
        self.pipeline = Pipeline()
        self.descriptions = []
        def parse(description):
            self.descriptions.append(description)
            return self.pipeline
        def video_info(caps):
            width, height = int(re.search(r'width=(\d+)', caps)[1]), int(re.search(r'height=(\d+)', caps)[1])
            y_stride, uv_stride = (width + 3) // 4 * 4, (width // 2 + 3) // 4 * 4
            return NS(size=y_stride * height + uv_stride * height,
                      stride=[y_stride, uv_stride, uv_stride],
                      offset=[0, y_stride * height, y_stride * height + uv_stride * height // 2])
        self.gst_video = NS(video_event_new_downstream_force_key_unit=lambda *_: object(),
                            VideoInfo=NS(new_from_caps=video_info))
        self.gst = NS(Caps=NS(from_string=lambda value: value), ElementFactory=NS(find=lambda _: True), parse_launch=parse,
                      State=NS(PLAYING='playing', NULL='null'),
                      StateChangeReturn=NS(FAILURE='failure'), MessageType=NS(ERROR='error'),
                      FlowReturn=NS(OK='ok'), SECOND=1_000_000_000,
                      CLOCK_TIME_NONE=2**64-1, PARAM_MUTABLE_PLAYING=4096, Buffer=Buffer)
        self.platform = {'is_jetson': True, 'model': 'Jetson AGX Orin', 'hardware_note': None}
        self.platform_patch = patch('middleware.core.jetson_h264.jetson_platform', return_value=self.platform)
        self.gst_patch = patch('middleware.core.jetson_h264._load_gst', return_value=(self.gst, self.gst_video))
        self.platform_patch.start()
        self.gst_patch.start()
        self.addCleanup(self.platform_patch.stop)
        self.addCleanup(self.gst_patch.stop)

    def test_real_raw_frame_conversion_annexb_timestamp_and_force_idr_contract(self):
        codec = JetsonH264Codec(64, 48, 1_000_000)
        self.addCleanup(codec.close)
        self.assertEqual(codec.encode(video_frame(0)), [IDR])
        self.assertEqual(len(self.pipeline.buffer.data), 64*48*3//2)
        self.assertEqual(codec.encode(video_frame(1)), [DELTA])
        self.assertEqual(self.pipeline.buffer.pts, 1_000_000_000//30)
        self.assertEqual(codec.encode(video_frame(2, force=True)), [IDR])
        self.assertEqual(self.pipeline.native_force_calls, 1)
        self.assertEqual(self.pipeline.standard_force_calls, 0)
        description = self.descriptions[0]
        for setting in ('nvv4l2h264enc', 'nvvidconv', 'memory:NVMM', 'max-buffers=2',
                        'max-bytes=4608', 'block=false', 'alignment=au', 'level=(string)3.1'):
            self.assertIn(setting, description)

    def test_unknown_native_action_falls_back_to_standard_force_key_event(self):
        codec = JetsonH264Codec(64, 48, 1_000_000)
        self.addCleanup(codec.close)
        codec.encode(video_frame())
        self.pipeline.native_force_supported = False
        self.assertEqual(codec.encode(video_frame(1, force=True)), [IDR])
        self.assertEqual(self.pipeline.native_force_calls, 1)
        self.assertEqual(self.pipeline.standard_force_calls, 1)

    def test_native_force_action_failure_is_not_silently_ignored(self):
        codec = JetsonH264Codec(64, 48, 1_000_000)
        codec.encode(video_frame())
        self.pipeline.encoder.emit = lambda *_: (_ for _ in ()).throw(TypeError('driver argument failure'))
        with self.assertRaisesRegex(TypeError, 'driver argument failure'):
            codec.encode(video_frame(1, force=True))
        self.assertEqual(self.pipeline.standard_force_calls, 0)
        self.assertTrue(codec.closed)

    def test_device_enumeration_preserves_nvhost_and_adds_v4l2_nvenc(self):
        # Bypass setUp's platform mock to exercise actual discovery logic.
        function = jetson_platform
        with patch('middleware.core.jetson_h264.Path.read_bytes',
                   side_effect=[b'NVIDIA Jetson AGX Orin Developer Kit', b'nvidia,tegra234']), \
             patch('middleware.core.jetson_h264.Path.glob',
                   side_effect=[[Path('/dev/nvhost-msenc')], [], [Path('/dev/v4l2-nvenc')]]) as glob:
            platform = function()
        self.assertEqual(platform['encoder_devices'], ['/dev/nvhost-msenc', '/dev/v4l2-nvenc'])
        self.assertTrue(platform['is_jetson'])
        self.assertEqual(glob.call_count, 3)

    def test_i420_uses_gstreamer_plane_stride_and_offset_for_width_642(self):
        codec = JetsonH264Codec(642, 400, 1_000_000)
        self.addCleanup(codec.close)
        codec.encode(video_frame(width=642, height=400))
        data = self.pipeline.buffer.data
        self.assertEqual(len(data), 644 * 400 + 324 * 200 * 2)
        self.assertIn(f'max-bytes={len(data)}', self.descriptions[0])
        # Real YUV conversion emits Y=16 / U=128 / V=128 for this black frame;
        # Gst padding must stay zero, not absorb pixels from the next row/plane.
        self.assertEqual(data[641:645], bytes([16, 0, 0, 16]))
        uv_offset = 644 * 400
        self.assertEqual(data[uv_offset + 320:uv_offset + 325], bytes([128, 0, 0, 0, 128]))
        self.assertEqual(data[uv_offset + 324 * 200], 128)

    def test_timeout_stops_pipeline_and_prevents_more_inputs(self):
        codec = JetsonH264Codec(64, 48, 1_000_000)
        self.pipeline.timeout = True
        with self.assertRaisesRegex(RuntimeError, 'within 3s'):
            codec.encode(video_frame())
        self.assertEqual(self.pipeline.states[-1], 'null')
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            codec.encode(video_frame(1))
        codec.close()
        self.assertEqual(self.pipeline.states.count('null'), 1)

    def test_vbv_size_is_a_bit_budget_not_a_frame_count(self):
        codec = JetsonH264Codec(64, 48, 1_000_000)
        self.addCleanup(codec.close)
        self.assertEqual(self.pipeline.properties['vbv-size'], 250_000)
        # Real L4T 36.4 fails with BlockSide error 0x4 when this is set to
        # fps//4 (=7). The 250ms budget must scale with bits per second.
        self.assertGreater(self.pipeline.properties['vbv-size'], 1_000)

    def test_dynamic_bitrate_requires_advertised_playing_mutability(self):
        codec = JetsonH264Codec(64, 48, 1_000_000)
        self.addCleanup(codec.close)
        self.assertTrue(codec.set_bitrate(500_000))
        self.assertEqual(codec.bit_rate, 500_000)
        self.pipeline.mutable = False
        self.assertFalse(codec.set_bitrate(800_000))
        self.assertEqual(codec.bit_rate, 500_000)

    def test_missing_plugin_is_actionable(self):
        self.gst.ElementFactory.find = lambda name: name != 'nvv4l2h264enc'
        with self.assertRaisesRegex(RuntimeError, 'nvidia-l4t-gstreamer'):
            JetsonH264Codec(64, 48, 1_000_000)

    def test_orin_nano_has_explicit_hardware_limitation(self):
        self.platform['hardware_note'] = 'Jetson Orin Nano has no hardware video encoder; use x264.'
        with self.assertRaisesRegex(RuntimeError, 'cannot add the missing encoder'):
            JetsonH264Codec(64, 48, 1_000_000)

    def test_profile_level_missing_headers_and_non_annexb_are_rejected(self):
        for data in (b'not h264', DELTA, IDR.replace(b'\x67\x42\xc0\x1f', b'\x67\x64\xc0\x1f'),
                     IDR.replace(b'\x67\x42\xc0\x1f', b'\x67\x42\x00\x1f'),
                     IDR.replace(b'\x67\x42\xc0\x1f', b'\x67\x42\xc0\x28')):
            with self.subTest(data=data):
                with self.assertRaises(RuntimeError):
                    validate_access_unit(data, require_idr=True, require_headers=True)

    def test_auto_tries_jetson_then_nvenc_then_real_x264(self):
        token = encoder_backend.set('auto')
        try:
            encoder = FastH264Encoder()
        finally:
            encoder_backend.reset(token)
        original = encoder._create_codec
        attempted = []
        def create(frame, backend):
            attempted.append(backend)
            if backend != 'x264':
                raise RuntimeError(f'{backend} not available')
            return original(frame, backend)
        with patch.object(encoder, '_create_codec', side_effect=create):
            self.assertTrue(list(encoder._encode_frame(video_frame(), False)))
        self.assertEqual(attempted, ['jetson', 'nvenc', 'x264'])
        self.assertEqual(encoder.actual_backend, 'x264')
        self.assertIn('JETSON', encoder.fallback_reason)
        self.assertIn('NVENC', encoder.fallback_reason)

    def test_owner_closes_native_pipeline_and_can_apply_live_bitrate(self):
        token = encoder_backend.set('jetson')
        try:
            encoder = FastH264Encoder()
        finally:
            encoder_backend.reset(token)
        self.addCleanup(encoder.close)
        self.assertTrue(list(encoder._encode_frame(video_frame(), False)))
        self.assertEqual(encoder.actual_backend, 'jetson')
        codec = encoder.codec
        encoder._last_configuration = -100
        encoder.target_bitrate = 500_000
        list(encoder._encode_frame(video_frame(1), False))
        self.assertIs(encoder.codec, codec)
        self.assertEqual(encoder.configured_bitrate, 500_000)
        self.assertEqual(encoder.dynamic_bitrate_updates, 1)
        encoder.close()
        self.assertEqual(self.pipeline.states[-1], 'null')

    def test_failed_jetson_frame_closes_pipeline_and_falls_back(self):
        token = encoder_backend.set('jetson')
        try:
            encoder = FastH264Encoder()
        finally:
            encoder_backend.reset(token)
        list(encoder._encode_frame(video_frame(), False))
        self.pipeline.timeout = True
        self.assertTrue(list(encoder._encode_frame(video_frame(1), False)))
        self.assertEqual(encoder.actual_backend, 'x264')
        self.assertEqual(self.pipeline.states[-1], 'null')


@unittest.skipUnless(os.environ.get('HC_TEST_JETSON_ENCODER') == '1', 'requires actual Jetson hardware and JetPack')
class RealJetsonTests(unittest.TestCase):
    def test_hardware_encode_idr_and_software_decode(self):
        codec = JetsonH264Codec(640, 400, 1_000_000)
        decoder = av.CodecContext.create('h264', 'r')
        frames = []
        try:
            for index in range(3):
                frame = av.VideoFrame.from_ndarray(np.zeros((400, 640, 3), np.uint8), format='bgr24')
                frame.pts = index * 3000
                frame.time_base = Fraction(1, 90000)
                if index == 2:
                    frame.pict_type = av.video.frame.PictureType.I
                data = b''.join(codec.encode(frame))
                frames.extend(decoder.decode(av.Packet(data)))
            frames.extend(decoder.decode(None))
            self.assertEqual(len(frames), 3)
            self.assertTrue(all((f.width, f.height) == (640, 400) for f in frames))
        finally:
            codec.close()
