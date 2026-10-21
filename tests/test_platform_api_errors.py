"""The browser receives safe, actionable errors; logs never contain pairing secrets."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from humanoid_upload.common import UploadError
from middleware.core.platform_api import register_platform_routes


class PlatformErrorTests(unittest.IsolatedAsyncioTestCase):
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
