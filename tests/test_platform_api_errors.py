"""The browser receives safe, actionable errors; logs never contain pairing secrets."""
import asyncio
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from humanoid_upload.common import UploadError
from middleware.core.platform_api import register_platform_routes


class PlatformErrorTests(unittest.IsolatedAsyncioTestCase):
    async def test_missing_connection_service_reports_recovery_action(self):
        app = web.Application()
        register_platform_routes(app, SimpleNamespace(platform=None))
        async with TestClient(TestServer(app)) as client:
            for method, path in (('get', 'status'), ('post', 'connect')):
                response = await getattr(client, method)('/api/platform/' + path)
                self.assertEqual(response.status, 503)
                body = await response.json()
                self.assertEqual(body['code'], 'PLATFORM_UNAVAILABLE')
                self.assertTrue(body['retryable'])
                self.assertIn('重启采集端', body['reason'])
                self.assertEqual(response.headers['Cache-Control'], 'no-store')

    async def test_busy_connection_status_does_not_block_other_requests(self):
        entered, release = threading.Event(), threading.Event()

        def status():
            entered.set()
            release.wait(2)
            return {'connection': None}

        app = web.Application()
        register_platform_routes(app, SimpleNamespace(platform=SimpleNamespace(status=status)))

        async def ping(request):
            return web.json_response({'ok': True})

        app.router.add_get('/ping', ping)
        async with TestClient(TestServer(app)) as client:
            started = time.monotonic()
            pending = asyncio.create_task(client.get('/api/platform/status'))
            try:
                self.assertTrue(await asyncio.to_thread(entered.wait, 1))
                response = await asyncio.wait_for(client.get('/ping'), 1)
                self.assertEqual(response.status, 200)
                self.assertLess(time.monotonic() - started, 1)
            finally:
                release.set()
                await pending

    async def test_connection_failure_preserves_code_reason_and_retryability(self):
        for code, reason, retryable in (
            ('NETWORK_TIMEOUT', '连接平台超时，请检查网络', True),
            ('CONNECTION_TICKET_EXPIRED', '连接链接已过期，请重新生成', False),
            ('ROBOT_CREDENTIAL_REVOKED', '上传凭据已撤销，请重新生成连接链接', False),
        ):
            with self.subTest(code=code):
                platform = SimpleNamespace(connect=Mock(side_effect=UploadError(code, reason, retry=retryable)), error=None)
                app = web.Application()
                register_platform_routes(app, SimpleNamespace(platform=platform))
                with self.assertLogs('middleware.core.platform_api', level='WARNING') as logs:
                    async with TestClient(TestServer(app)) as client:
                        response = await client.post('/api/platform/connect', json={
                            'link': 'http://platform/#robot-connect=never-log-this-ticket'})
                        self.assertEqual(response.status, 409)
                        self.assertEqual(await response.json(), {
                            'code': code, 'reason': reason, 'retryable': retryable})
                        self.assertEqual(response.headers['Cache-Control'], 'no-store')
                self.assertIn(code, '\n'.join(logs.output))
                self.assertNotIn('never-log-this-ticket', '\n'.join(logs.output))
