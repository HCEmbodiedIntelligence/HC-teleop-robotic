"""Real spawned control/observer integration, restricted to localhost domain 231."""
import asyncio
import os
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('std_msgs.msg')

from middleware.core.app import MiddlewareRuntime
from middleware.core.config import ConfigStore, validate_config


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '231' or
                    os.environ.get('ROS_LOCALHOST_ONLY') != '1',
                    reason='isolated ROS integration requires localhost domain 231')
def test_runtime_uses_spawned_authority_and_keeps_observer_health(tmp_path, monkeypatch):
    monkeypatch.setenv('HC_RUNTIME_MODE', 'teleop')
    monkeypatch.setenv('HC_MANAGED_CONTROL', '0')
    monkeypatch.delenv('HC_ROBOT_NAME', raising=False)
    monkeypatch.delenv('HC_ISOLATED_CONTROL_PROCESS', raising=False)
    config = validate_config({'robot_profiles': {'active': ''}, 'vr': {'enabled': False},
        'camera': {'enabled': False}, 'ros': {'enabled': True, 'domain_id': 231,
            'isolated_control_process': True,
            'recording': {'directory': str(tmp_path/'records')},
            'subscriptions': [{'topic': '/test/isolated_observer', 'type': 'std_msgs/msg/String',
                               'outputs': ['websocket']}]},
        'uploads': {'queue_directory': str(tmp_path/'uploads')}})
    runtime = MiddlewareRuntime(config, tmp_path)
    from aiohttp.test_utils import TestClient, TestServer
    import middleware.core.app as app_module
    monkeypatch.setenv('HC_MIDDLEWARE_LOG_DIR', str(tmp_path/'logs'))
    monkeypatch.setattr(app_module, 'MiddlewareRuntime', lambda *a: runtime)
    store = ConfigStore(tmp_path/'config.yaml')
    store.save(config)

    async def check():
        app = app_module.create_app(store)
        async with TestClient(TestServer(app)) as client:
            status = runtime.ros.status()
            assert status['state'] == 'running'
            assert status['isolated_control_process'] is True
            assert status['control_process']['pid'] != os.getpid()
            assert status['command_mux']['output_enabled'] is False
            assert runtime.ros.ownership.snapshot()['state'] == 'OFF'
            response = await client.get('/api/control/ownership')
            assert response.status == 200
            assert (await response.json())['state'] == 'OFF'
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                assert runtime.ros.publish('/test/isolated_observer', 'std_msgs/msg/String', {'data': 'observer'})
                await asyncio.sleep(.05)
                if runtime.ros.status()['topic_health']['/test/isolated_observer']['has_data']:
                    break
            assert runtime.ros.status()['topic_health']['/test/isolated_observer']['has_data']
            response = await client.post('/api/safety/resume', json={'reason': 'localhost test explicit resume'})
            assert response.status == 202
            assert (await response.json())['accepted'] is True
            deadline = time.monotonic()+2
            while not runtime.ros.status()['command_mux']['output_enabled'] and time.monotonic()<deadline:
                await asyncio.sleep(.01)
            assert runtime.ros.status()['command_mux']['output_enabled'] is True
            response = await client.post('/api/safety/stop', json={'reason': 'localhost test stop'})
            assert response.status == 202
            deadline = time.monotonic()+2
            while runtime.ros.status()['command_mux']['output_enabled'] and time.monotonic()<deadline:
                await asyncio.sleep(.01)
            assert runtime.ros.status()['command_mux']['output_enabled'] is False
        core = runtime.ros._client.process
        assert not core.is_alive()
        assert runtime._control_commands is None
        assert runtime._record_commands is None

    asyncio.run(check())
