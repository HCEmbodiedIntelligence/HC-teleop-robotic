import asyncio
import copy
import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from hc_platform.data import DataError, DataService
from hc_platform.data.upload import PlatformAdapter


class TestPlatform:
    __test__ = False

    def __init__(self):
        self.remote = None
        self.parts = {}
        self.puts = []
        self.lost_put = self.lost_commit = False
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def respond(self, value):
                body = json.dumps(value).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def disconnect(self):
                self.connection.shutdown(socket.SHUT_RDWR)
                self.connection.close()

            def do_POST(self):
                assert self.headers["Authorization"] == "Bearer test-token"
                data = json.loads(
                    self.rfile.read(int(self.headers.get("Content-Length", "0")))
                    or b"null"
                )
                suffix = self.path.removeprefix("/api/v1/robot-ingest/uploads")
                if suffix == "":
                    if owner.remote is None:
                        owner.remote = {
                            "upload_id": "remote",
                            "client_upload_id": data["client_upload_id"],
                            "authenticated_robot_id": data["robot_id"],
                            "request_robot_id": data["robot_id"],
                            "target": {
                                "collection_task_id": data["collection_task_id"]
                            },
                            "manifest": data,
                            "state": "UPLOADING",
                            "processing_status": "NOT_REQUESTED",
                            "quality_status": "PENDING",
                            "assets": [
                                {
                                    "asset_id": a["asset_id"],
                                    "path": a["path"],
                                    "state": "UPLOADING",
                                    **{
                                        "expected_" + k: a[k]
                                        for k in ("size_bytes", "sha256", "crc64")
                                    },
                                }
                                for a in data["assets"]
                            ],
                        }
                    else:
                        assert owner.remote["manifest"] == data
                    return self.respond({"data": owner.remote})
                if suffix.endswith(":authorize-parts"):
                    asset = suffix.split("/")[3].split(":")[0]
                    return self.respond(
                        {
                            "upload_id": "remote",
                            "asset_id": asset,
                            "uploaded_parts": list(owner.parts.get(asset, {}).values()),
                            "authorizations": [
                                {
                                    "part_number": n,
                                    "url": owner.base + f"/objects/{asset}/{n}",
                                }
                                for n in data["part_numbers"]
                            ],
                        }
                    )
                if suffix.endswith(":complete"):
                    asset = suffix.split("/")[3].split(":")[0]
                    for remote in owner.remote["assets"]:
                        if remote["asset_id"] == asset:
                            remote["state"] = "COMPLETED"
                    return self.respond({"data": owner.remote})
                if suffix.endswith(":commit"):
                    owner.remote.update(state="COMMITTED", raw_source_id="raw-one")
                    if not owner.lost_commit:
                        owner.lost_commit = True
                        return self.disconnect()
                    return self.respond({"data": owner.remote})
                self.send_error(404)

            def do_PUT(self):
                assert "Authorization" not in self.headers
                _, _, asset, number = self.path.split("/")
                number = int(number)
                body = self.rfile.read(int(self.headers["Content-Length"]))
                owner.puts.append((asset, number))
                owner.parts.setdefault(asset, {})[number] = {
                    "part_number": number,
                    "etag": f"etag-{number}",
                    "size_bytes": len(body),
                }
                if not owner.lost_put:
                    owner.lost_put = True
                    return self.disconnect()
                self.send_response(200)
                self.send_header("ETag", f"etag-{number}")
                self.send_header("Content-Length", "0")
                self.end_headers()

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.base = f"http://127.0.0.1:{self.server.server_port}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()


def test_real_http_reconciles_lost_put_and_lost_commit(tmp_path, monkeypatch):
    monkeypatch.setenv("HC_TEST_PLATFORM_TOKEN", "test-token")
    platform = TestPlatform()

    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session({"robot_id": "x1"}, "capture")
            await service.append_sample(
                session["id"], "/state", {"names": ["j"], "position": [1]}
            )
            result = await service.stop_session(session["id"])
            artifact = await service.get_dataset(result["artifact_id"])
            adapter = PlatformAdapter(
                {
                    "platform_base_url": platform.base,
                    "credential_env": "HC_TEST_PLATFORM_TOKEN",
                }
            )
            saved = {}

            async def checkpoint(value):
                saved.clear()
                saved.update(copy.deepcopy(value))

            async def cancelled():
                pass

            job_id = "9c8c7f8a-8338-4fe2-b0b5-842a1758fa0f"
            target = {"robot_id": "x1", "collection_task_id": "task-one"}
            for _ in range(3):
                try:
                    outcome = await adapter.upload(
                        artifact, job_id, target, dict(saved), checkpoint, cancelled
                    )
                    break
                except DataError as error:
                    assert error.retryable
            else:
                pytest.fail("Upload did not reconcile")
            assert outcome["remote_state"] == "COMMITTED"
            assert outcome["quality_status"] == "PENDING"
            assert len(platform.puts) == len(set(platform.puts))
            assert platform.lost_put and platform.lost_commit
            assert platform.remote["client_upload_id"] == job_id
        finally:
            await service.close()

    try:
        asyncio.run(scenario())
    finally:
        platform.close()
