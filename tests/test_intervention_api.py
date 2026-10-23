import asyncio
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from middleware.core.control_ownership import ControlOwnership
from middleware.core.intervention_api import register_intervention_routes


class InterventionApiTests(unittest.IsolatedAsyncioTestCase):
    def recording_lifecycle(self):
        self.control.peer('motion', 'motion')
        self.control.peer('teleop', 'teleop')

        def prepare(_metadata, *, capture_mode):
            self.recorder.corrections = SimpleNamespace(capture_mode=capture_mode, status=lambda: {'ready': True})

        self.recorder.prepare_corrections.side_effect = prepare
        self.recorder.stop_corrections.side_effect = lambda: setattr(self.recorder, 'corrections', None)

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

    async def test_capture_mode_is_validated_before_enabling_and_passed_to_recorder(self):
        self.control.peer('motion', 'motion')
        self.control.peer('teleop', 'teleop')
        for invalid in ('all_topics', None, [], {}):
            response = await self.command('enable', capture_mode=invalid)
            self.assertEqual(response.status, 400)
            self.assertEqual(self.control.state, 'OFF')
        response = await self.command('enable', capture_mode='full_session')
        self.assertEqual(response.status, 202)
        self.assertEqual(self.recorder.prepare_corrections.call_args.kwargs['capture_mode'], 'full_session')
        self.assertEqual(self.recorder.prepare_corrections.call_args.args[0]['session_id'], self.control.session)

    async def test_active_capture_mode_cannot_be_changed_even_by_reusing_request_id(self):
        self.control.peer('motion', 'motion')
        self.control.peer('teleop', 'teleop')
        response = await self.command('enable')
        self.assertEqual(response.status, 202)
        self.assertEqual(self.recorder.prepare_corrections.call_args.kwargs['capture_mode'], 'corrections_only')
        self.recorder.corrections = SimpleNamespace(capture_mode='corrections_only', status=lambda: {})
        response = await self.command('enable', capture_mode='full_session')
        self.assertEqual(response.status, 409)
        self.assertEqual(self.control.state, 'HOLD')
        self.assertEqual(self.recorder.prepare_corrections.call_count, 1)

    async def test_old_disable_retry_cannot_stop_new_session_capture(self):
        self.recording_lifecycle()
        self.assertEqual((await self.command('enable')).status, 202)
        state = self.control.snapshot()
        old_stop = dict(request_id='old-stop', session_id=state['session_id'], expected_epoch=state['epoch'])
        self.assertEqual((await self.command('disable', **old_stop)).status, 202)
        self.assertEqual((await self.command('enable', request_id='new-enable')).status, 202)
        capture = self.recorder.corrections
        state = self.control.snapshot()
        self.assertEqual((await self.command('disable', **old_stop)).status, 202)
        self.assertEqual(self.control.snapshot(), state)
        self.assertIs(self.recorder.corrections, capture)
        self.assertEqual(self.recorder.stop_corrections.call_count, 1)

    async def test_old_enable_retry_cannot_create_orphan_capture_after_disable(self):
        self.recording_lifecycle()
        state = self.control.snapshot()
        old_enable = dict(request_id='enable', session_id=state['session_id'], expected_epoch=state['epoch'])
        self.assertEqual((await self.command('enable', **old_enable)).status, 202)
        self.assertEqual((await self.command('disable')).status, 202)
        self.assertEqual((await self.command('enable', **old_enable)).status, 202)
        self.assertEqual(self.control.state, 'OFF')
        self.assertIsNone(self.recorder.corrections)
        self.assertEqual(self.recorder.prepare_corrections.call_count, 1)

    async def test_failed_prepare_retry_after_disable_cannot_create_orphan_capture(self):
        self.recording_lifecycle()
        state = self.control.snapshot()
        old_enable = dict(request_id='enable', session_id=state['session_id'], expected_epoch=state['epoch'])
        self.recorder.prepare_corrections.side_effect = OSError('disk full')
        self.assertEqual((await self.command('enable', **old_enable)).status, 409)
        self.assertEqual((await self.command('disable')).status, 202)
        self.recording_lifecycle()
        self.assertEqual((await self.command('enable', **old_enable)).status, 202)
        self.assertIsNone(self.recorder.corrections)

    async def test_concurrent_identical_enable_prepares_once(self):
        self.recording_lifecycle()
        state = self.control.snapshot()
        fields = dict(request_id='same', session_id=state['session_id'], expected_epoch=state['epoch'])
        responses = await asyncio.gather(*(self.command('enable', **fields) for _ in range(8)))
        self.assertEqual([r.status for r in responses], [202] * 8)
        self.assertEqual(self.recorder.prepare_corrections.call_count, 1)

    async def test_failed_stop_can_retry_only_in_its_original_generation(self):
        self.recording_lifecycle()
        self.assertEqual((await self.command('enable')).status, 202)
        self.recorder.stop_corrections.side_effect = OSError('temporary writer failure')
        state = self.control.snapshot()
        fields = dict(request_id='stop', session_id=state['session_id'], expected_epoch=state['epoch'])
        self.assertEqual((await self.command('disable', **fields)).status, 409)
        self.recording_lifecycle()
        self.assertEqual((await self.command('disable', **fields)).status, 202)
        self.assertEqual(self.recorder.stop_corrections.call_count, 2)
        self.assertIsNone(self.recorder.corrections)

    async def test_capture_preparation_cannot_arm_input_or_trip_active_input_watchdog(self):
        self.recording_lifecycle()
        now=[1_000_000_000]
        self.control.clock=self.control.ros_clock=lambda:now[0]
        prepare=self.recorder.prepare_corrections.side_effect

        def slow_prepare(metadata, *, capture_mode):
            # Input arrives while writers/subscriptions are still being created.
            self.control.input(device_instance='vr',sequence=1,valid=True,gripped=False,stamp_ns=now[0])
            now[0]+=400_000_000
            self.control.watchdog()
            self.assertEqual(self.control.state,'HOLD')
            self.control.input(device_instance='vr',sequence=2,valid=True,gripped=True,stamp_ns=now[0])
            self.assertEqual(self.control.state,'HOLD')
            self.assertFalse(self.control.input_armed)
            prepare(metadata,capture_mode=capture_mode)

        self.recorder.prepare_corrections.side_effect=slow_prepare
        self.assertEqual((await self.command('enable')).status,202)
        self.assertFalse(self.control.input_armed)
        self.control.input(device_instance='vr',sequence=3,valid=True,gripped=True,stamp_ns=now[0])
        self.assertEqual(self.control.state,'HOLD','held Grip during preparation must not become a new takeover')
        self.control.input(device_instance='vr',sequence=4,valid=True,gripped=False,stamp_ns=now[0])
        self.assertTrue(self.control.input_armed)
        # Once preparation finishes, the original 300 ms watchdog still applies.
        now[0]+=301_000_000
        self.control.watchdog()
        self.assertEqual(self.control.state,'FAULT')

    async def test_estop_during_preparation_remains_latched_after_prepare_finishes(self):
        self.recording_lifecycle()
        prepare=self.recorder.prepare_corrections.side_effect
        def stopped(metadata, *, capture_mode):
            self.control.fault('test estop',estop=True)
            prepare(metadata,capture_mode=capture_mode)
        self.recorder.prepare_corrections.side_effect=stopped
        self.assertEqual((await self.command('enable')).status,202)
        self.assertEqual(self.control.state,'ESTOP')
        self.assertFalse(self.control.input_armed)

    async def test_async_capture_readiness_requires_fresh_release_and_keeps_output_closed(self):
        self.recording_lifecycle()
        ready = [False]
        prepare = self.recorder.prepare_corrections.side_effect
        def delayed(metadata, *, capture_mode):
            prepare(metadata, capture_mode=capture_mode)
            self.recorder.corrections.status = lambda: {'ready': ready[0]}
        self.recorder.prepare_corrections.side_effect = delayed
        self.control.capture_ready = lambda: ready[0]
        self.assertEqual((await self.command('enable')).status, 202)
        self.assertTrue(self.control.snapshot()['preparing_capture'])
        for seq, grip in enumerate([False, True], 1):
            self.control.input(device_instance='vr', sequence=seq, valid=True,
                               gripped=grip, stamp_ns=self.control.ros_clock())
        self.assertFalse(self.control.takeover())
        self.assertEqual((await self.command('return')).status, 409)
        self.assertEqual(self.control.state, 'HOLD')
        self.control.finish_capture_preparation('stale-session')
        self.assertTrue(self.control.preparing_capture)
        ready[0] = True
        self.control.watchdog()
        self.assertFalse(self.control.preparing_capture)
        self.control.input(device_instance='vr', sequence=3, valid=True,
                           gripped=True, stamp_ns=self.control.ros_clock())
        self.assertFalse(self.control.input_armed)
        self.assertEqual(self.control.state, 'HOLD')
        for seq, grip in [(4, False), (5, True)]:
            self.control.input(device_instance='vr', sequence=seq, valid=True,
                               gripped=grip, stamp_ns=self.control.ros_clock())
        self.assertEqual(self.control.state, 'TAKEOVER')

    async def test_preparation_release_delivered_after_readiness_cannot_arm_input(self):
        self.recording_lifecycle()
        now = [1_000_000_000]
        self.control.clock = self.control.ros_clock = lambda: now[0]
        def prepare(metadata, *, capture_mode):
            # This release was produced before readiness but is still less
            # than 300 ms old when its delayed callback is dispatched.
            now[0] += 100_000_000
            self.recorder.corrections = SimpleNamespace(capture_mode=capture_mode, status=lambda: {'ready': True})
        self.recorder.prepare_corrections.side_effect = prepare
        self.assertEqual((await self.command('enable')).status, 202)
        self.control.input(device_instance='vr', sequence=1, valid=True,
                           gripped=False, stamp_ns=now[0]-50_000_000)
        self.assertFalse(self.control.input_armed)
        self.control.input(device_instance='vr', sequence=2, valid=True,
                           gripped=True, stamp_ns=now[0])
        self.assertEqual(self.control.state, 'HOLD')
        for seq, grip in [(3, False), (4, True)]:
            self.control.input(device_instance='vr', sequence=seq, valid=True,
                               gripped=grip, stamp_ns=now[0]+1)
        self.assertEqual(self.control.state, 'TAKEOVER')

    async def test_concurrent_distinct_enable_with_same_epoch_has_one_winner(self):
        self.recording_lifecycle()
        state = self.control.snapshot()
        responses = await asyncio.gather(*(self.command('enable', request_id=f'new-{i}',
            session_id=state['session_id'], expected_epoch=state['epoch']) for i in range(8)))
        self.assertEqual(sorted(r.status for r in responses), [202] + [409] * 7)
        self.assertEqual(self.recorder.prepare_corrections.call_count, 1)

    async def test_invalid_json_is_a_client_error_and_never_changes_authority(self):
        for body in (b'{', b'', b'\xff', b'null', b'[]', b'42'):
            with self.subTest(body=body):
                response = await self.client.post('/api/control/intervention/enable', data=body,
                    headers={'Content-Type': 'application/json'})
                self.assertEqual(response.status, 400)
                self.assertEqual(self.control.state, 'OFF')
        self.recorder.prepare_corrections.assert_not_called()
