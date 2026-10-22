import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from middleware.core.control_ownership import ControlOwnership
from middleware.core.intervention_api import register_intervention_routes


class InterventionApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.control = ControlOwnership()
        self.recorder = SimpleNamespace(corrections=None, is_recording=Mock(return_value=False),
                                        prepare_corrections=Mock(), stop_corrections=Mock())
        ros = SimpleNamespace(ownership=self.control, control_ready=lambda: True,
                              recover_intervention=Mock())
        runtime = SimpleNamespace(ros=ros, recorder=self.recorder, player=None, session_profile_metadata={})
        app = web.Application()
        register_intervention_routes(app, runtime)
        self.client = TestClient(TestServer(app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()

    async def command(self, action, **fields):
        state = self.control.snapshot()
        return await self.client.post('/api/control/intervention/'+action, json={
            'request_id': action, 'session_id': state['session_id'], 'expected_epoch': state['epoch'], **fields})

    async def test_session_requires_motion_and_does_not_interrupt_ordinary_recording(self):
        response = await self.command('enable')
        self.assertEqual(response.status, 409)
        self.assertEqual(self.control.state, 'OFF')
        self.control.peer('motion', 'motion')
        self.control.peer('teleop', 'teleop')
        self.recorder.is_recording.return_value = True
        response = await self.command('enable')
        self.assertEqual(response.status, 409)
        self.assertEqual(self.control.state, 'OFF')
        self.recorder.prepare_corrections.assert_not_called()

    async def test_failed_capture_prepare_keeps_hold_and_explicit_retry_can_prepare(self):
        self.control.peer('motion', 'motion')
        self.control.peer('teleop', 'teleop')
        self.recorder.prepare_corrections.side_effect = RuntimeError('disk unavailable')
        response = await self.command('enable')
        self.assertEqual(response.status, 409)
        self.assertEqual(self.control.state, 'HOLD')
        self.recorder.prepare_corrections.side_effect = None
        response = await self.command('enable')
        self.assertEqual(response.status, 202)
        self.assertEqual(self.recorder.prepare_corrections.call_count, 2)
        response = await self.command('return')
        self.assertEqual(response.status, 409)
        self.assertEqual(self.control.state, 'HOLD')
        response = await self.command('disable')
        self.assertEqual(response.status, 202)
        self.recorder.stop_corrections.assert_called_once()

    async def test_malformed_or_stale_requests_cannot_change_authority(self):
        for fields in ({'request_id': []}, {'expected_epoch': True}, {'session_id': 'old'}):
            response = await self.command('hold', **fields)
            self.assertEqual(response.status, 409)
            self.assertEqual(self.control.state, 'OFF')
