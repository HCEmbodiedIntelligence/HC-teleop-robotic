import json
from pathlib import Path
from unittest.mock import AsyncMock, patch

from aiohttp.test_utils import TestClient, TestServer
import unittest
from middleware.core.app import create_app, MiddlewareRuntime
from middleware.core.camera import CameraService
from middleware.core.config import ConfigStore


class ReceiverApiTests(unittest.IsolatedAsyncioTestCase):
    async def test_receiver_api_limits_session_errors_and_static_page(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            await self.check_api(Path(directory))

    async def check_api(self, tmp_path):
        camera = CameraService({'id':'head', 'enabled':True})
        camera._receiver_metrics.reset('current')
        # Match process service's async boundary without starting ROS or cameras.
        async def report(payload):
            return camera.receiver_report(payload)
        camera.receiver_report_async = AsyncMock(side_effect=report)
        class Proxy:
            receiver_report = camera.receiver_report_async
        async def start(runtime):
            runtime.cameras = {'head': Proxy()}
        config = tmp_path / 'config.yaml'
        config.write_text('ros:\n  enabled: false\nvr:\n  enabled: false\n')
        with patch.object(MiddlewareRuntime, 'start', start), patch.object(MiddlewareRuntime, 'stop', AsyncMock()):
            async with TestClient(TestServer(create_app(ConfigStore(config)))) as client:
                url = '/api/webrtc/receiver/head'
                payload = {'session_id':'current','sequence':0,'sample_ms':0,'stats_id':'video'}
                response = await client.post(url, json=payload)
                assert response.status == 200 and (await response.json())['accepted']
                response = await client.post(url, json=payload)
                assert not (await response.json())['accepted']
                response = await client.post(url, json={**payload,'session_id':'old'})
                assert response.status == 400
                assert (await client.post(url, data='{"bad":')).status == 400
                assert (await client.post(url, data='x'*16385)).status == 413
                assert (await client.post('/api/webrtc/receiver/unknown', json=payload)).status == 404
                response = await client.get('/static/receiver.html')
                assert response.status == 200
                assert 'receiver_metrics.js' in await response.text()
