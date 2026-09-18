"""Exercise the complete-SDP camera handshake without a running ROS server."""
import asyncio
import unittest

import numpy as np
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
from middleware.core.camera import CameraService


class CameraHandshakeTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.camera = CameraService({'enabled': True, 'id': 'test', 'width': 64, 'height': 48})
        self.camera._set_status(state='running')

    async def asyncTearDown(self):
        for peer in list(self.camera._pcs):
            await peer.close()

    async def test_missing_candidates_is_explicit_and_does_not_create_peer(self):
        with self.assertRaisesRegex(ValueError, 'no ICE candidates'):
            await self.camera.offer({'sdp': 'v=0\r\n', 'type': 'offer'})
        self.assertEqual(self.camera.status()['peers'], 0)
        self.assertIn('no ICE candidates', self.camera.status()['webrtc_error'])

    async def test_invalid_sdp_releases_peer(self):
        with self.assertRaises(Exception):
            await self.camera.offer({'sdp': 'a=candidate:invalid\r\n', 'type': 'offer'})
        self.assertEqual(self.camera.status()['peers'], 0)
        self.assertTrue(self.camera.status()['webrtc_error'])

    async def test_complete_offer_delivers_camera_pixels(self):
        self.camera.set_frame(np.full((48, 64, 3), 180, dtype=np.uint8))
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        track_ready = asyncio.get_running_loop().create_future()

        @client.on('track')
        def on_track(track):
            if not track_ready.done():
                track_ready.set_result(track)

        try:
            client.addTransceiver('video', direction='recvonly')
            await client.setLocalDescription(await client.createOffer())
            answer = await self.camera.offer({'sdp': client.localDescription.sdp, 'type': 'offer'})
            await client.setRemoteDescription(RTCSessionDescription(**answer))
            track = await asyncio.wait_for(track_ready, 10)
            frame = await asyncio.wait_for(track.recv(), 10)
            self.assertEqual((frame.width, frame.height), (64, 48))
            self.assertAlmostEqual(float(frame.to_ndarray(format='bgr24').mean()), 180, delta=8)
            status = self.camera.status()
            self.assertEqual(status['webrtc_state'], 'connected')
            self.assertGreater(status['remote_ice_candidates'], 0)
            self.assertGreater(status['local_ice_candidates'], 0)
            self.assertIsNone(status['webrtc_error'])
        finally:
            await client.close()
