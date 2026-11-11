"""Exercise the complete-SDP camera handshake without a running ROS server."""
import asyncio
import unittest

import numpy as np
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
from middleware.core.camera import CameraService


class CameraHandshakeTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.camera = CameraService({'enabled': True, 'id': 'test', 'width': 64, 'height': 48, 'dscp': 34})
        self.camera._set_status(state='running')

    async def asyncTearDown(self):
        await self.camera.stop()

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

    async def test_cancelled_offer_releases_partial_peer(self):
        import threading
        from unittest.mock import patch
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        entered, cleaned = threading.Event(), threading.Event()
        original_close = RTCPeerConnection.close
        async def stalled_local(peer, description):
            entered.set()
            await asyncio.Event().wait()
        async def closing(peer):
            try:
                return await original_close(peer)
            finally:
                cleaned.set()
        try:
            client.addTransceiver('video', direction='recvonly')
            await client.setLocalDescription(await client.createOffer())
            with patch.object(RTCPeerConnection, 'setLocalDescription', stalled_local), patch.object(RTCPeerConnection, 'close', closing):
                task = asyncio.create_task(self.camera.offer({'sdp':client.localDescription.sdp,'type':'offer'}))
                self.assertTrue(await asyncio.to_thread(entered.wait, 3))
                task.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await task
                self.assertTrue(await asyncio.to_thread(cleaned.wait, 3))
                # Let the cancellation handler remove the now-closed peer.
                for _ in range(50):
                    if not self.camera._pcs:
                        break
                    await asyncio.sleep(.01)
                self.assertEqual(self.camera.status()['peers'], 0)
        finally:
            await client.close()

    async def test_complete_offer_delivers_camera_pixels(self):
        async def source():
            while True:
                self.camera.set_frame(np.full((48, 64, 3), 180, dtype=np.uint8))
                await asyncio.sleep(.05)
        producer = asyncio.create_task(source())
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
            lines = answer['sdp'].splitlines()
            first_payload = next(line for line in lines if line.startswith('m=video ')).split()[3]
            first_codec = next(
                line for line in lines if line.startswith(f'a=rtpmap:{first_payload} ')
            )
            self.assertIn(' H264/', first_codec)
            self.assertTrue(any(' rtx/90000' in line for line in lines))
            self.assertTrue(answer['telemetry_session_id'])
            await client.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))
            track = await asyncio.wait_for(track_ready, 10)
            frame = await asyncio.wait_for(track.recv(), 10)
            self.assertEqual((frame.width, frame.height), (64, 48))
            self.assertAlmostEqual(float(frame.to_ndarray(format='bgr24').mean()), 180, delta=8)
            await asyncio.sleep(1.1)
            status = self.camera.status()
            self.assertEqual(status['webrtc_state'], 'connected')
            self.assertEqual(status['webrtc_codec'], 'H264')
            self.assertGreater(status['webrtc_packets_sent'], 0)
            self.assertGreater(status['webrtc_frames_output'], 0)
            self.assertGreater(status['webrtc_output_fps'], 0)
            self.assertGreaterEqual(status['webrtc_submit_to_transport_ms'], status['webrtc_transport_ms'])
            self.assertEqual(status['fps_window_seconds'], 2.0)
            self.assertGreater(status['webrtc_target_bitrate_mbps'], 0)
            self.assertGreater(status['remote_ice_candidates'], 0)
            self.assertGreater(status['local_ice_candidates'], 0)
            self.assertIsNone(status['webrtc_error'])
            self.assertEqual(status['media_qos']['state'], 'marked')
            self.assertGreater(status['media_qos']['marked_sockets'], 0)
            self.assertEqual(status['encoder_backend_actual'], 'x264')
            self.assertIsNotNone(status['webrtc_submit_to_transport_p95_ms'])
            # Exercise negotiated RTX with an explicit receiver NACK over a
            # recent actual packet; inspect the wire-format repair payload.
            from aiortc.rtp import RtcpRtpfbPacket, RtpPacket
            from unittest.mock import AsyncMock
            async def check_repair():
                server_pc = next(iter(self.camera._pcs))
                sender = server_pc.getSenders()[0]
                history = sender._RTCRtpSender__rtp_history
                original = next(iter(history.values()))
                real_send = sender.transport._send_rtp
                capture = AsyncMock()
                sender.transport._send_rtp = capture
                try:
                    await sender._handle_rtcp_packet(RtcpRtpfbPacket(
                        fmt=1, ssrc=123, media_ssrc=sender._ssrc,
                        lost=[original.sequence_number]))
                    repair = RtpPacket.parse(capture.call_args.args[0])
                    self.assertEqual(repair.ssrc, sender._rtx_ssrc)
                    self.assertEqual(repair.payload_type, sender._RTCRtpSender__rtx_payload_type)
                    self.assertEqual(int.from_bytes(repair.payload[:2], 'big'), original.sequence_number)
                finally:
                    sender.transport._send_rtp = real_send
            await asyncio.wrap_future(asyncio.run_coroutine_threadsafe(check_repair(), self.camera._rtc_loop))
            payload = {'session_id': answer['telemetry_session_id'], 'sequence': 0,
                       'sample_ms': 0, 'stats_id': 'client-video', 'frames_decoded': 0}
            self.assertTrue(self.camera.receiver_report(payload))
            self.assertFalse(self.camera.receiver_report(payload))
            with self.assertRaisesRegex(ValueError, 'session mismatch'):
                self.camera.receiver_report({**payload, 'session_id': 'old'})
        finally:
            producer.cancel()
            await asyncio.gather(producer, return_exceptions=True)
            await client.close()
