import asyncio
import time
import unittest
from fractions import Fraction
from unittest.mock import patch

import numpy as np
from aiortc.mediastreams import MediaStreamError
from middleware.core.camera import CameraService


class VideoClockTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.camera = CameraService({'enabled': True, 'width': 64, 'height': 48, 'fps': 30})
        self.image = np.zeros((48, 64, 3), dtype=np.uint8)
        self.track = self.camera._create_video_track()

    async def asyncTearDown(self):
        self.track.stop()
        await self.camera.stop()

    async def test_slow_sender_clock_follows_elapsed_time(self):
        self.camera.set_frame(self.image)
        first = await self.track.recv()
        started = time.monotonic()
        await asyncio.sleep(.17)  # Encoding/network stall, then a fresh source frame.
        self.camera.set_frame(self.image + 10)
        second = await self.track.recv()
        media_seconds = (second.pts - first.pts) / 90000
        self.assertAlmostEqual(media_seconds, time.monotonic() - started, delta=.025)
        self.assertGreater(second.pts - first.pts, 12000)
        self.assertEqual(second.time_base, Fraction(1, 90000))

    async def test_no_repeat_and_newest_frame_wins(self):
        self.camera.set_frame(self.image)
        await self.track.recv()
        pending = asyncio.create_task(self.track.recv())
        await asyncio.sleep(.08)
        self.assertFalse(pending.done())
        for value in (10, 20, 30):
            self.camera.set_frame(self.image + value)
        frame = await asyncio.wait_for(pending, .5)
        self.assertEqual(float(frame.to_ndarray(format='bgr24').mean()), 30)
        self.assertEqual(self.camera.status()['webrtc_frames_skipped'], 2)

    async def test_old_frame_is_not_sent_on_first_connect(self):
        self.camera._accept_frame(self.image, 'test', arrival_ns=time.monotonic_ns()-400_000_000)
        pending = asyncio.create_task(self.track.recv())
        await asyncio.sleep(.05)
        self.assertFalse(pending.done())
        self.camera.set_frame(self.image+40)
        frame = await asyncio.wait_for(pending, .5)
        self.assertEqual(float(frame.to_ndarray(format='bgr24').mean()), 40)

    async def test_stop_unblocks_receiver(self):
        pending = asyncio.create_task(self.track.recv())
        await asyncio.sleep(.01)
        self.track.stop()
        with self.assertRaises(MediaStreamError):
            await asyncio.wait_for(pending, .5)

    async def test_source_clock_jump_does_not_rewind_rtp(self):
        self.camera._accept_frame(self.image, 'test', capture_ns=100_000_000_000)
        first = await self.track.recv()
        await asyncio.sleep(.04)
        self.camera._accept_frame(self.image, 'test', capture_ns=1)
        second = await self.track.recv()
        self.assertGreater(second.pts, first.pts)

    async def test_concurrent_offers_are_serialized(self):
        active = 0
        peak = 0
        async def handshake(data):
            nonlocal active, peak
            active += 1
            peak = max(peak, active)
            await asyncio.sleep(.02)
            active -= 1
            return data
        with patch.object(self.camera, '_offer_locked', side_effect=handshake):
            result = await asyncio.gather(self.camera._offer({'id': 1}), self.camera._offer({'id': 2}))
        self.assertEqual(peak, 1)
        self.assertEqual(result, [{'id': 1}, {'id': 2}])


if __name__ == '__main__':
    unittest.main()
