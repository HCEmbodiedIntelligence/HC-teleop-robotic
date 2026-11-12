"""Regression coverage for WebRTC scheduling and stale-frame waiting."""
import asyncio
from pathlib import Path
import subprocess
import sys
import textwrap
import threading
import unittest

from middleware.core.camera import CameraService


class CameraRtcIsolationTests(unittest.IsolatedAsyncioTestCase):
    async def test_offer_and_peer_shutdown_use_dedicated_loop(self):
        camera = CameraService({'enabled': True})
        main_loop = asyncio.get_running_loop()
        main_thread = threading.get_ident()
        calls = []

        class Peer:
            async def close(self):
                calls.append(('close', threading.get_ident(), asyncio.get_running_loop()))

        async def offer(data):
            calls.append(('offer', threading.get_ident(), asyncio.get_running_loop()))
            camera._pcs.add(Peer())
            await asyncio.sleep(0)
            return {'sdp': data['sdp'], 'type': 'answer'}

        camera._offer = offer
        try:
            result = await asyncio.wait_for(camera.offer({'sdp': 'test'}), 2)
            self.assertEqual(result, {'sdp': 'test', 'type': 'answer'})
            rtc_thread = camera._rtc_thread
            rtc_loop = camera._rtc_loop
            self.assertTrue(rtc_thread.is_alive())
            self.assertNotEqual(calls[0][1], main_thread)
            self.assertIsNot(calls[0][2], main_loop)
            self.assertIs(calls[0][2], rtc_loop)
        finally:
            await asyncio.wait_for(camera.stop(), 3)
        self.assertEqual([entry[0] for entry in calls], ['offer', 'close'])
        self.assertEqual(calls[0][1], calls[1][1])
        self.assertIs(calls[1][2], rtc_loop)
        self.assertFalse(rtc_thread.is_alive())
        self.assertTrue(rtc_loop.is_closed())
        self.assertIsNone(camera._rtc_thread)
        self.assertIsNone(camera._rtc_loop)
        self.assertFalse(camera._pcs)
        await camera.stop()  # Repeated shutdown remains safe.


class StaleFrameWaitTests(unittest.TestCase):
    def test_unsent_stale_frame_yields_then_accepts_fresh_frame(self):
        # A regression spins without awaiting, so an asyncio timeout alone
        # cannot bound the failure. Use a subprocess with an outer deadline.
        script = textwrap.dedent('''
            import asyncio
            import time
            import numpy as np
            from middleware.core.camera import CameraService

            async def check():
                camera = CameraService({'enabled': True, 'width': 64, 'height': 48})
                image = np.zeros((48, 64, 3), dtype=np.uint8)
                camera._accept_frame(image, 'test',
                    arrival_ns=time.monotonic_ns() - 3_000_000_000)
                track = camera._create_video_track()
                pending = asyncio.create_task(track.recv())
                try:
                    await asyncio.sleep(0.04)
                    assert not pending.done(), 'stale frame was transmitted'
                    camera.set_frame(image + 23)
                    frame = await asyncio.wait_for(pending, 1)
                    assert frame.to_ndarray(format='bgr24').mean() == 23
                finally:
                    track.stop()
                    await camera.stop()
            asyncio.run(check())
        ''')
        result = subprocess.run(
            [sys.executable, '-c', script],
            cwd=Path(__file__).resolve().parents[1],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
