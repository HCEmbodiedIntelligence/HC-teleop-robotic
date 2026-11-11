import asyncio
import struct
import time
from types import SimpleNamespace
import unittest

from middleware.core.camera_metrics import CameraPipelineMetrics, instrument_sender


class PipelineMetricsTests(unittest.TestCase):
    def test_local_overwrite_is_reported_without_network_loss_feedback(self):
        from middleware.core.camera import CameraService
        camera=CameraService({'enabled':True})
        camera._pcs.add(object())  # An active viewer enables the decode mailbox.
        msg=SimpleNamespace(format='jpeg', data=b'pending JPEG')
        topic='/observation/cameras/head/color/compressed'
        camera.handle_ros_message(topic,msg)
        camera.handle_ros_message(topic,msg)
        status=camera.status()
        self.assertEqual(status['decode_frames_overwritten'],1)
        self.assertEqual(status['pipeline_counts']['capture'],2)
        self.assertEqual(status['pipeline_counts']['decode'],0)
        self.assertFalse(status['webrtc_feedback_received'])
        self.assertEqual(status['webrtc_remote_packets_lost'],0)
        camera._pcs.clear()

    def test_stage_rates_share_one_window_and_expire(self):
        now = [10.0]
        m = CameraPipelineMetrics(clock=lambda: now[0])
        for index, stage in enumerate(('capture', 'decode', 'submit')):
            now[0] = 10.1 + index*.1
            m.record(stage)
        now[0] = 10.4
        m.output(12, 3)
        now[0] = 12.0
        status = m.snapshot()
        for field in ('capture_fps','decoded_fps','webrtc_submit_fps','webrtc_output_fps'):
            self.assertEqual(status[field], .5)
        self.assertEqual(status['webrtc_submit_to_transport_ms'], 12)
        self.assertEqual(status['webrtc_transport_ms'], 3)
        now[0] = 13
        status = m.snapshot()
        self.assertEqual(status['webrtc_output_fps'], 0)
        self.assertIsNone(status['webrtc_submit_to_transport_ms'])
        self.assertEqual(status['webrtc_frames_output'], 1)

    def test_latency_quantiles_use_the_live_window(self):
        now = [0.0]
        m = CameraPipelineMetrics(clock=lambda: now[0])
        for duration in (1, 2, 3, 4, 100):
            m.output(duration, duration / 2)
        status = m.snapshot()
        self.assertEqual(status['webrtc_submit_to_transport_p50_ms'], 3)
        self.assertEqual(status['webrtc_submit_to_transport_p95_ms'], 100)
        self.assertEqual(status['webrtc_transport_p99_ms'], 50)
        self.assertEqual(status['webrtc_latency_sample_count'], 5)
        now[0] = 3
        self.assertIsNone(m.snapshot()['webrtc_submit_to_transport_p99_ms'])

    def test_submit_is_not_counted_as_output_and_session_reset_preserves_input(self):
        now = [0.0]
        m = CameraPipelineMetrics(clock=lambda: now[0])
        now[0] = .1
        m.record('capture');m.record('decode');m.record('submit')
        now[0] = 1
        self.assertEqual(m.snapshot()['webrtc_output_fps'], 0)
        self.assertEqual(m.snapshot()['webrtc_submit_fps'], 1)
        m.reset_sender()
        self.assertEqual(m.snapshot()['pipeline_counts'], {'capture':1,'decode':1,'submit':0,'output':0})


class SenderInstrumentationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.metrics = CameraPipelineMetrics()
        self.track = SimpleNamespace(_handoff_at=time.monotonic()-.03)
        self.fail = False
        async def transmit(data):
            await asyncio.sleep(.003)
            if self.fail:raise ConnectionError('closed')
        async def encoded(codec):return object()
        self.sender=SimpleNamespace(_ssrc=88, _next_encoded_frame=encoded,
                                    transport=SimpleNamespace(_send_rtp=transmit))
        instrument_sender(SimpleNamespace(connectionState='connected'),self.sender,self.track,self.metrics)

    def packet(self, *, last=False, ssrc=88, timestamp=3000):
        return struct.pack('!BBHII',128,102+(128 if last else 0),1,timestamp,ssrc)+b'x'

    async def test_count_only_completed_primary_frame_and_measure_full_duration(self):
        await self.sender._next_encoded_frame(None)
        await self.sender.transport._send_rtp(self.packet())
        self.assertEqual(self.metrics.snapshot()['webrtc_frames_output'],0)
        await self.sender.transport._send_rtp(self.packet(last=True,ssrc=99))
        await self.sender.transport._send_rtp(self.packet(last=True,timestamp=1))
        await self.sender.transport._send_rtp(b'\x80\xc8'+b'\x00'*26)
        self.assertEqual(self.metrics.snapshot()['webrtc_frames_output'],0)
        await self.sender.transport._send_rtp(self.packet(last=True))
        result=self.metrics.snapshot()
        self.assertEqual(result['webrtc_frames_output'],1)
        self.assertGreaterEqual(result['webrtc_submit_to_transport_ms'],40)
        self.assertGreaterEqual(result['webrtc_transport_ms'],6)
        self.assertGreater(result['webrtc_submit_to_transport_ms'],result['webrtc_transport_ms'])
        await self.sender.transport._send_rtp(self.packet(last=True))
        self.assertEqual(self.metrics.snapshot()['webrtc_frames_output'],1)

    async def test_transport_failure_does_not_count_as_output(self):
        await self.sender._next_encoded_frame(None)
        self.fail=True
        with self.assertRaises(ConnectionError):await self.sender.transport._send_rtp(self.packet(last=True))
        self.assertEqual(self.metrics.snapshot()['webrtc_frames_output'],0)


if __name__ == '__main__':unittest.main()
