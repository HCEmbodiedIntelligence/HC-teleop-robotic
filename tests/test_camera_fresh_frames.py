import asyncio
import time
import unittest
from unittest.mock import patch
import numpy as np
from aiortc.mediastreams import MediaStreamError
from middleware.core.camera import CameraService

class FreshFrameTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.camera=CameraService({'enabled':True,'width':64,'height':48})
        self.image=np.zeros((48,64,3),dtype=np.uint8)

    async def test_no_placeholder_no_duplicates_and_variable_timestamps(self):
        track=self.camera._create_video_track()
        pending=asyncio.create_task(track.recv())
        await asyncio.sleep(.04)
        self.assertFalse(pending.done())
        now=time.monotonic_ns()
        self.camera._accept_frame(self.image,'test',arrival_ns=now)
        first=await asyncio.wait_for(pending,.5)
        pending=asyncio.create_task(track.recv())
        await asyncio.sleep(.08)  # Source below 30 fps must not be padded.
        self.assertFalse(pending.done())
        self.camera._accept_frame(self.image,'test',arrival_ns=now+80_000_000)
        second=await asyncio.wait_for(pending,.5)
        self.assertEqual(second.pts-first.pts,7200)
        self.assertEqual(float(second.time_base),1/90000)
        self.assertEqual(self.camera._webrtc_frames_sent,2)
        track.stop()

    async def test_each_peer_receives_newest_frame_without_consuming_other_peer(self):
        a,b=self.camera._create_video_track(),self.camera._create_video_track()
        self.camera.set_frame(self.image)
        await asyncio.gather(a.recv(),b.recv())
        self.camera.set_frame(self.image+10)
        self.camera.set_frame(self.image+20)
        frames=await asyncio.gather(a.recv(),b.recv())
        for frame in frames:self.assertEqual(frame.to_ndarray(format='bgr24').mean(),20)
        a.stop();b.stop()

    async def test_disconnect_or_shutdown_unblocks_waiting_receiver(self):
        for shutdown in [False,True]:
            camera=CameraService({'enabled':True})
            track=camera._create_video_track();pending=asyncio.create_task(track.recv())
            await asyncio.sleep(.01)
            if shutdown:await camera.stop()
            else:track.stop()
            with self.assertRaises(MediaStreamError):await asyncio.wait_for(pending,.5)

    async def test_old_decode_cannot_replace_newer_frame_and_rate_expires(self):
        now=time.monotonic_ns()
        self.camera._accept_frame(self.image+20,'test',arrival_ns=now)
        self.camera._accept_frame(self.image+10,'test',arrival_ns=now-1000)
        self.assertEqual(self.camera.latest().mean(),20)
        self.camera._set_status(capture_fps=20.,webrtc_send_fps=20.)
        self.camera._record_webrtc_frame()
        with patch('middleware.core.camera.time.monotonic',return_value=time.monotonic()+2):
            self.assertEqual(self.camera.status()['capture_fps'],0)
            self.assertEqual(self.camera.status()['webrtc_send_fps'],0)
