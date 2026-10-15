import asyncio
import importlib.util
import json
import os
from pathlib import Path

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from humanoid_upload.common import atomic_json
from test_client import enqueue


def test_optional_web_routes_use_installed_subprocess(tmp_path, cfg, queue):
    repository = Path(__file__).resolve().parents[2]
    workspace = repository.parents[1]
    executable = os.environ.get('HUMANOID_UPLOAD_TEST_CLI', str(workspace/"install/uploader/bin/humanoid-upload"))
    spec = importlib.util.spec_from_file_location("upload_hook", repository/"python/humanoid_manager/web/platform_upload_api.py")
    hook = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hook)
    config = tmp_path/"config.json"
    atomic_json(config, cfg)
    enqueue(queue, cfg)

    async def scenario():
        app = web.Application()
        hook.register_platform_upload_routes(app, executable=executable, config_path=config)
        async with TestClient(TestServer(app)) as client:
            response = await client.get("/platform-upload")
            assert response.status == 200
            assert 'Episode 数据包、质检及审核结果请进入平台查看' in await response.text()
            assert response.headers['Cache-Control']=='no-cache'
            jobs = await (await client.get("/api/platform-upload/queue")).json()
            assert jobs[0]["state"] == "queued"
            assert "frozen" not in jobs[0] and "manifest" not in jobs[0]
            response = await client.put("/api/platform-upload/config", json={**cfg, "token": "secret"})
            assert response.status == 400
            assert "secret" not in await response.text()
            assert json.loads(config.read_text()) == cfg
            response = await client.put("/api/platform-upload/config", json={**cfg, "task_id": "new-task"})
            assert response.status == 200
            response = await client.get("/api/platform-upload/config")
            assert (await response.json())["task_id"] == "new-task"
            response = await client.get("/api/platform-upload/queue")
            assert (await response.json())[0]["task_id"] == "synthetic-task-001"
    asyncio.run(scenario())
