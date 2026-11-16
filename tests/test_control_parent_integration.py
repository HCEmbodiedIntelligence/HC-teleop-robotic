"""Parent lifecycle and authorization rules for isolated control."""
import asyncio
import sys
import threading
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest

from middleware.core.app import MiddlewareRuntime, control_bridge_class
from middleware.core.config import validate_config
from middleware.core.ros_bridge import RosBridge


class DeferredCommands:
    def __init__(self):
        self.pending = []

    def submit(self, operation, *args, **kwargs):
        future = Future()
        self.pending.append((operation, args, kwargs, future))
        return future

    def drain(self):
        for operation, args, kwargs, future in self.pending:
            future.set_result(operation(*args, **kwargs))
        self.pending.clear()


class IsolatedFake:
    isolated_control_process = True

    def __init__(self, state='running'):
        self.state = state
        self.generation = 0
        self.ownership = SimpleNamespace(active=False, disable=Mock())
        self.resume_calls = []
        self.publish = Mock(return_value=True)

    def status(self):
        return {'state': self.state, 'command_mux': {'source': 'vr'}}

    def authorization_fence(self):
        return ('boot', self.generation, 0)

    def emergency_stop(self, *_args):
        self.generation += 1

    def resume_output(self, topic, *, expected_fence):
        if expected_fence != self.authorization_fence():
            raise ValueError('authorization was revoked')
        self.resume_calls.append((topic, expected_fence))


def runtime_for(tmp_path, state='running'):
    runtime = MiddlewareRuntime(validate_config({}), tmp_path)
    runtime.ros = IsolatedFake(state)
    runtime._safety_reason = 'original fault'
    runtime.emit = Mock()
    runtime._log = Mock()
    return runtime


def test_bridge_selection_is_opt_in_and_environment_is_reversible(monkeypatch):
    monkeypatch.delenv('HC_ISOLATED_CONTROL_PROCESS', raising=False)
    assert control_bridge_class({}) is RosBridge
    facade = type('IsolatedRosBridge', (), {})
    monkeypatch.setitem(sys.modules, 'middleware.core.control_process',
                        SimpleNamespace(IsolatedRosBridge=facade))
    assert control_bridge_class({'isolated_control_process': True}) is facade
    monkeypatch.setenv('HC_ISOLATED_CONTROL_PROCESS', '1')
    assert control_bridge_class({}) is facade
    monkeypatch.setenv('HC_ISOLATED_CONTROL_PROCESS', '0')
    assert control_bridge_class({'isolated_control_process': True}) is RosBridge


def test_remote_capture_is_installed_on_base_writer_without_transferring_recorder(tmp_path):
    runtime = runtime_for(tmp_path)
    runtime.config['ros']['legacy_control_profile'] = 'legacy'
    recorder = SimpleNamespace(records_topic=Mock(return_value=True))
    proxy = object()
    runtime.ros.create_publication_capture = Mock(return_value=proxy)
    runtime._configure_publication_capture(recorder)
    assert recorder.publication_capture is proxy
    assert runtime.config['ros']['record_action_at_publication'] is True
    args, kwargs = runtime.ros.create_publication_capture.call_args
    assert args == (recorder,)
    assert kwargs['selected']()
    recorder.records_topic.assert_called_once_with('/action/joint_states', 'sensor_msgs/msg/JointState')


def test_facade_keeps_camera_observation_health_when_merging_core_status():
    from middleware.core.control_process import IsolatedRosBridge
    bridge = IsolatedRosBridge.__new__(IsolatedRosBridge)
    camera_health = {'/test/camera/color/compressed': {'has_data': True, 'hz': 30}}
    bridge._telemetry = SimpleNamespace(status=lambda: {'state': 'running', 'topic_health': camera_health,
        'command_mux': {'forwarded_hz': 999}})
    core = {'state': 'running', 'command_mux': {'channels': {'arms': {'forwarded_hz': 100}},
            'forwarded_hz': 300, 'output_enabled': False}, 'ownership': {'state': 'OFF'},
            'control_actor': {'pending_targets': 0, 'coalesced': 7}}
    bridge._client = SimpleNamespace(snapshot=lambda: core)
    status = bridge.status()
    assert status['topic_health'] is camera_health
    assert status['command_mux']['channels']['arms']['forwarded_hz'] == 100
    assert status['command_mux']['forwarded_hz'] == 300
    assert status['command_mux']['output_enabled'] is False
    assert status['isolated_control_process'] is True
    assert status['control_actor'] == core['control_actor']


@pytest.mark.parametrize('state', ['starting', 'error', 'stopped'])
def test_dead_or_starting_core_cannot_clear_fault_or_resume(tmp_path, state):
    runtime = runtime_for(tmp_path, state)
    runtime.control_runtime = SimpleNamespace(enabled=True, status=lambda: {'state': 'running'})
    assert runtime._on_safety_resume('A pressed') is False
    assert runtime._safety_reason == 'original fault'
    assert runtime.ros.resume_calls == []
    runtime.ros.publish.assert_not_called()
    runtime.emit.assert_not_called()


def test_isolated_resume_is_one_child_transaction(tmp_path):
    runtime = runtime_for(tmp_path)
    assert runtime._on_safety_resume('A pressed') is True
    assert runtime.ros.resume_calls == [('/control/emergency_stop', ('boot', 0, 0))]
    runtime.ros.publish.assert_not_called()
    assert runtime._safety_reason == ''


def test_queued_old_a_does_not_become_new_authorization_after_stop(tmp_path):
    runtime = runtime_for(tmp_path)
    commands = DeferredCommands()
    runtime._control_commands = commands
    runtime._dispatch_safety_resume('A pressed before stop')
    assert runtime.ros.resume_calls == []
    assert commands.pending[0][2]['expected_fence'] == ('boot', 0, 0)
    runtime._on_safety_event('later stop')
    commands.drain()
    assert runtime.ros.resume_calls == []
    assert runtime._safety_reason == 'later stop'
    assert not any(call.args[0]['kind'] == 'safety_resume' for call in runtime.emit.call_args_list)


def test_stop_after_successful_child_transaction_is_not_overwritten_by_delayed_reply(tmp_path):
    runtime = runtime_for(tmp_path)
    original_resume = runtime.ros.resume_output

    def resume_then_stop(topic, *, expected_fence):
        original_resume(topic, expected_fence=expected_fence)
        runtime._on_safety_event('stop after child resume before RPC reply')

    runtime.ros.resume_output = resume_then_stop
    assert runtime._on_safety_resume('A pressed') is False
    assert runtime._safety_reason == 'stop after child resume before RPC reply'
    assert not any(call.args[0]['kind'] == 'safety_resume' for call in runtime.emit.call_args_list)


def test_a_rpc_and_recording_work_never_run_on_pose_receive_thread(tmp_path):
    runtime = runtime_for(tmp_path)
    runtime._control_commands = DeferredCommands()
    runtime._record_commands = DeferredCommands()
    operation = Mock()
    runtime._dispatch_recording(operation)
    runtime._on_pose(SimpleNamespace(right_input=SimpleNamespace(held_mask=1, pressed_mask=1)), {})
    assert runtime.ros.resume_calls == []
    assert len(runtime._control_commands.pending) == 1
    operation.assert_not_called()
    assert len(runtime._record_commands.pending) == 1
    runtime.ros.publish.assert_called_once()  # VR pose continues immediately.
    runtime._control_commands.drain()
    assert len(runtime.ros.resume_calls) == 1
    operation.assert_not_called()  # Disk work has its own queue.
    runtime._record_commands.drain()
    operation.assert_called_once()


def test_unstarted_or_stopping_runtime_never_runs_inline_recording(tmp_path):
    runtime = runtime_for(tmp_path)
    operation = Mock()
    runtime._dispatch_recording(operation)
    runtime._stopping = True
    runtime._control_commands = DeferredCommands()
    runtime._record_commands = DeferredCommands()
    runtime._dispatch_recording(operation)
    runtime._dispatch_safety_resume('stale A')
    operation.assert_not_called()
    assert runtime._control_commands.pending == []
    assert runtime._record_commands.pending == []


@pytest.mark.parametrize('core_dead', [False, True])
def test_shutdown_revokes_then_drains_capture_before_core_and_writer(tmp_path, core_dead):
    runtime = runtime_for(tmp_path)
    order = []
    runtime.ros.ownership.active = True
    runtime.ros.ownership.disable = Mock(side_effect=RuntimeError('core died') if core_dead else lambda: order.append('disable'))
    runtime.ros.emergency_stop = lambda *a: order.append('revoke')
    runtime.ros.stop = lambda: order.append('core drain/stop')
    runtime.vr = SimpleNamespace(stop=lambda: order.append('vr stop'))
    runtime._control_commands = SimpleNamespace(shutdown=lambda **k: order.append('control commands drained'))
    runtime._record_commands = SimpleNamespace(shutdown=lambda **k: order.append('record commands drained'))
    runtime.recorder = SimpleNamespace(stop=lambda: order.append('capture seal/drain'),
                                       shutdown=lambda: order.append('writer shutdown'))
    runtime.recording_ros = SimpleNamespace(stop=lambda: order.append('raw readers stop'))
    asyncio.run(runtime.stop())
    assert order[0] == 'revoke'
    assert order.index('vr stop') < order.index('control commands drained')
    assert order.index('record commands drained') < order.index('capture seal/drain')
    assert order.index('capture seal/drain') < order.index('raw readers stop') < order.index('core drain/stop')
    assert order.index('core drain/stop') < order.index('writer shutdown')
    assert runtime._stopping is True


def test_http_resume_waits_off_event_loop_and_preserves_arrival_fence(tmp_path, monkeypatch):
    from aiohttp.test_utils import TestClient, TestServer
    import middleware.core.app as app_module
    from middleware.core.config import ConfigStore

    runtime = runtime_for(tmp_path)
    runtime.start = AsyncMock()
    runtime.stop = AsyncMock()
    entered, release = threading.Event(), threading.Event()
    original_resume = runtime.ros.resume_output

    def delayed_resume(topic, *, expected_fence):
        entered.set()
        assert release.wait(3)
        original_resume(topic, expected_fence=expected_fence)

    runtime.ros.resume_output = delayed_resume
    monkeypatch.setattr(app_module, 'MiddlewareRuntime', lambda *a: runtime)

    async def check():
        app = app_module.create_app(ConfigStore(tmp_path/'config.yaml'))
        runtime.control_runtime = SimpleNamespace(enabled=False, status=lambda: {'state': 'unmanaged'})
        async with TestClient(TestServer(app)) as client:
            resume = asyncio.create_task(client.post('/api/safety/resume', json={}))
            try:
                assert await asyncio.to_thread(entered.wait, 3)
                # The dashboard and emergency stop remain responsive during a
                # slow core RPC; the later stop revokes this request's fence.
                response = await asyncio.wait_for(client.post('/api/safety/stop', json={'reason': 'later stop'}), 1)
                assert response.status == 202
            finally:
                release.set()
            response = await resume
            assert response.status == 409
            assert (await response.json())['accepted'] is False
            assert runtime.ros.resume_calls == []
            assert runtime._safety_reason == 'later stop'

    asyncio.run(check())


@pytest.mark.parametrize('cancel_request', [False, True])
def test_shutdown_waits_for_inflight_capture_even_if_http_request_is_cancelled(tmp_path, cancel_request):
    from aiohttp import web
    from middleware.core.control_ownership import ControlOwnership
    from middleware.core.intervention_api import register_intervention_routes

    runtime = runtime_for(tmp_path)
    control = runtime.ros.ownership = ControlOwnership()
    for peer in ('motion', 'teleop'):
        control.peer(peer, peer)
    runtime.ros.control_ready = lambda: True
    runtime.ros.recover_intervention = Mock()
    entered, release = threading.Event(), threading.Event()
    order = []

    def apply_intervention(name, data, **_kwargs):
        applied = False
        def apply():
            nonlocal applied
            assert name == 'enable'
            control.enable(preparing_capture=True)
            applied = True
        state = control.request(data['request_id'], data['session_id'], data['expected_epoch'], apply)
        return {'ownership': state, 'applied': applied}

    runtime.ros.apply_intervention = apply_intervention
    runtime.ros.stop = lambda: order.append('core stopped')
    recorder = SimpleNamespace(corrections=None, is_recording=lambda: False,
                               stop=lambda: order.append('ordinary stopped'))
    runtime.recorder = recorder

    def prepare(*_args, **_kwargs):
        entered.set()
        assert release.wait(3)
        recorder.corrections = SimpleNamespace(capture_mode='corrections_only', status=lambda: {'ready': False})
        order.append('capture prepared')

    def shutdown():
        assert recorder.corrections is not None
        recorder.corrections = None
        order.append('capture cleaned')

    recorder.prepare_corrections, recorder.shutdown = prepare, shutdown
    app = web.Application()
    register_intervention_routes(app, runtime)
    handler = next(route.handler for route in app.router.routes() if route.method == 'POST')

    def request(identifier):
        state = control.snapshot()
        async def json():
            return {'request_id': identifier, 'session_id': state['session_id'], 'expected_epoch': state['epoch']}
        return SimpleNamespace(json=json, match_info={'action': 'enable'})

    async def check():
        action = asyncio.create_task(handler(request('prepare')))
        assert await asyncio.to_thread(entered.wait, 3)
        queued = asyncio.create_task(handler(request('queued-before-stop')))
        await asyncio.sleep(0)
        stopping = asyncio.create_task(runtime.stop())
        await asyncio.sleep(0)
        assert runtime._stopping and not stopping.done()
        assert 'core stopped' not in order
        with pytest.raises(web.HTTPServiceUnavailable):
            await handler(request('arrived-after-stop'))
        # Emergency revocation remains immediate while the lifecycle lease is
        # held by slow disk preparation.
        generation = runtime.ros.generation
        runtime._on_safety_event('urgent stop during capture preparation')
        assert runtime.ros.generation == generation+1
        if cancel_request:
            action.cancel()
            await asyncio.sleep(0)
            action.cancel()  # Repeated cancellation still cannot detach disk work.
            await asyncio.sleep(0)
            assert not action.done()
        release.set()
        if cancel_request:
            with pytest.raises(asyncio.CancelledError):
                await action
        else:
            assert (await action).status == 202
        with pytest.raises(web.HTTPServiceUnavailable):
            await queued
        await stopping
        assert order.index('capture prepared') < order.index('core stopped') < order.index('capture cleaned')
        assert recorder.corrections is None

    try:
        asyncio.run(check())
    finally:
        release.set()
