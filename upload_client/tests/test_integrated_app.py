"""Exercise main app upload lifecycle with ROS disabled and temporary storage."""
import asyncio
import os
import sys
from pathlib import Path

import pytest


def test_integrated_upload_lifecycle(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root))
    # This test must never initialize a ROS context even on ROS-enabled hosts.
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    from middleware.core import app as app_module
    from middleware.core.config import ConfigStore
    from aiohttp.test_utils import TestClient, TestServer
    assert Path(app_module.__file__).resolve().is_relative_to(root)
    runtimes = []
    original = app_module.MiddlewareRuntime.start

    async def start(runtime):
        runtimes.append(runtime)
        await original(runtime)

    monkeypatch.setattr(app_module.MiddlewareRuntime, 'start', start)
    store = ConfigStore(tmp_path / 'config.yaml')
    cfg = store.load()
    for section in ('ros', 'vr', 'camera', 'safety'):
        cfg[section]['enabled'] = False
    cfg['safety']['stop_on_startup'] = False
    cfg['robot_profiles']['root'] = str(tmp_path / 'profiles')
    cfg['ros']['recording'].update(directory=str(tmp_path/'data'),
        upload_job_state_dir=str(tmp_path/'jobs'), artifact_state_dir=str(tmp_path/'ready'))
    cfg['uploads'].update(enabled=True, auto_enqueue=False,
        queue_directory=str(tmp_path/'upload-queue'))
    store.save(cfg)

    async def exercise():
        client = TestClient(TestServer(app_module.create_app(store)))
        try:
            await client.start_server()
            response = await client.get('/api/uploads')
            assert response.status == 200, await response.text()
            status = await response.json()
            assert status['enabled'] and status['running']
            runtime = runtimes[0]
            upload_service = runtime.uploads
            assert not runtime.upload_error
            assert runtime.recorder.upload_service is runtime.uploads
            assert runtime.uploads.request_raw_seal is None
            assert runtime.recorder.__class__.__name__ == 'TaskRecorder'
            assert runtime.recorder.recorder.__class__.__name__ == 'TopicRecorder'
            assert runtime.platform.status()['auto_upload'] is False
            response = await client.post('/api/uploads/scan', json={})
            assert response.status == 200, await response.text()
            assert (await response.json())['enqueued'] == 0
            for path in ('/', '/static/upload_panel.js', '/static/upload_panel.css', '/static/simulation_viewer.js'):
                response = await client.get(path)
                assert response.status == 200
                if path.endswith('upload_panel.js'):
                    assert 'mountUploadPanel' in await response.text()
            routes = {route.resource.canonical for route in client.app.router.routes()}
            assert '/api/recordings/upload' in routes
        finally:
            await client.close()
        assert runtimes[0].uploads is None
        assert not upload_service.status()['running']

    asyncio.run(exercise())
