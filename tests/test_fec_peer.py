"""Actual ICE/DTLS/SRTP send with a RED/FEC receive shim for aiortc tests.

aiortc itself has no native FEC receive support. The test shim removes parity
sequence slots before its jitter buffer; production receivers use libwebrtc.
Independent native rtpulpfecdec interoperability is tested in test_rtp_fec.py.
"""
import asyncio
from collections import OrderedDict
from types import SimpleNamespace as NS
import struct
import unittest

import numpy as np
from aiortc import RTCPeerConnection, RTCConfiguration, RTCSessionDescription
from aiortc.sdp import SessionDescription
from aiortc.rtcrtpparameters import RTCRtpCodecParameters as Codec

from middleware.core.camera import CameraService
from middleware.core.config import validate_config, ConfigError
from middleware.core.rtp_fec import (red_unwrap, unwrap_red_payload, rtp_layout,
    protected_sequences, recover_ulpfec)


def add_fec(sdp):
    description = SessionDescription.parse(sdp)
    video = next(m for m in description.media if m.kind == 'video')
    video.fmt.extend([116, 117, 118])
    video.rtp.codecs.extend([Codec(mimeType='video/red', clockRate=90000, payloadType=116),
        Codec(mimeType='video/rtx', clockRate=90000, payloadType=117, parameters={'apt':116}),
        Codec(mimeType='video/ulpfec', clockRate=90000, payloadType=118)])
    return str(description)


class FecPeerTests(unittest.IsolatedAsyncioTestCase):
    async def test_encrypted_video_recovers_one_lost_packet_and_decodes_pixels(self):
        camera = CameraService({'id':'test','enabled':True,'width':64,'height':48,'codec':'VP8',
                                'fec':{'enabled':True,'group_size':5}})
        camera._set_status(state='running')
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        track_ready = asyncio.get_running_loop().create_future()
        @client.on('track')
        def received(track):
            if not track_ready.done(): track_ready.set_result(track)
        async def publish():
            while True:
                camera.set_frame(np.full((48,64,3),180,dtype=np.uint8))
                await asyncio.sleep(.04)
        producer = asyncio.create_task(publish())
        try:
            transceiver = client.addTransceiver('video', direction='recvonly')
            await client.setLocalDescription(await client.createOffer())
            answer = await camera.offer({'sdp':add_fec(client.localDescription.sdp),'type':'offer'})
            self.assertIn(' red/90000', answer['sdp'])
            self.assertIn(' ulpfec/90000', answer['sdp'])
            # Decode after genuine SRTP unprotect. Lose one media datagram and
            # recover only when its parity arrives, before forwarding to aiortc.
            transport = transceiver.receiver.transport
            original = transport._handle_rtp_data
            media = OrderedDict()
            dropped, recovered_count, next_seq = [False], [0], [0]
            async def receive(data, arrival_time_ms):
                layout = rtp_layout(data)
                block_pt, payload = unwrap_red_payload(data[layout.header_size:layout.payload_end])
                if block_pt != 118:
                    primary = red_unwrap(data)
                    if not dropped[0]:
                        dropped[0] = True
                        return
                    media[layout.sequence_number] = primary
                    return
                sequences = protected_sequences(payload)
                available = [media[s] for s in sequences if s in media]
                recovered = recover_ulpfec(payload, available, layout.ssrc)
                if recovered is not None:
                    media[rtp_layout(recovered).sequence_number] = recovered
                    recovered_count[0] += 1
                for seq in sequences:
                    packet = media.pop(seq, None)
                    if packet is None: continue
                    packet = bytearray(packet)
                    struct.pack_into('!H', packet, 2, next_seq[0])
                    next_seq[0] = (next_seq[0]+1)&65535
                    await original(bytes(packet), arrival_time_ms)
                while len(media)>64: media.popitem(last=False)
            transport._handle_rtp_data = receive
            await client.setRemoteDescription(RTCSessionDescription(sdp=answer['sdp'],type=answer['type']))
            track = await asyncio.wait_for(track_ready, 5)
            frame = await asyncio.wait_for(track.recv(), 8)
            self.assertEqual((frame.width,frame.height),(64,48))
            self.assertAlmostEqual(frame.to_ndarray(format='bgr24').mean(),180,delta=10)
            self.assertEqual(recovered_count[0],1)
            status = camera.status()['webrtc_fec']
            self.assertEqual(status['state'],'enabled')
            self.assertGreater(status['fec_packets'],0)
            self.assertFalse(status['recovery_verified'])
        finally:
            producer.cancel()
            await asyncio.gather(producer, return_exceptions=True)
            await client.close()
            await camera.stop()

    async def test_unsupported_peer_continues_plain_video(self):
        camera = CameraService({'id':'test','enabled':True,'codec':'VP8','fec':{'enabled':True}})
        camera._set_status(state='running')
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        try:
            client.addTransceiver('video',direction='recvonly')
            await client.setLocalDescription(await client.createOffer())
            answer = await camera.offer({'sdp':client.localDescription.sdp,'type':'offer'})
            self.assertNotIn(' red/90000',answer['sdp'])
            self.assertEqual(camera.status()['webrtc_fec']['state'],'peer_unsupported')
            await client.setRemoteDescription(RTCSessionDescription(sdp=answer['sdp'], type=answer['type']))
            for _ in range(100):
                if client.connectionState == 'connected':
                    break
                await asyncio.sleep(.01)
            self.assertEqual(client.connectionState, 'connected')
        finally:
            await client.close()
            await camera.stop()


class FecConfigTests(unittest.TestCase):
    def test_invalid_options_rejected(self):
        for fec in (True, {'enabled':1}, {'group_size':True}, {'group_size':11},
                    {'media_bitrate_ratio':float('nan')}, {'media_bitrate_ratio':1}):
            with self.subTest(fec=fec), self.assertRaises(ConfigError):
                validate_config({'camera':{'fec':fec}})
    def test_inheritance_does_not_pin_fec_to_old_global_settings(self):
        config = validate_config({'camera':{'streams':[{'id':'head','topic':'/head'}]}})
        self.assertNotIn('fec',config['camera']['streams'][0])
        config['camera']['fec']['enabled'] = True
        self.assertTrue(validate_config(config)['camera']['fec']['enabled'])
