import copy
import hashlib
import json
import os
import socket
import threading
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from humanoid_upload.common import DEFAULTS, canonical, validate
from humanoid_upload.queue import Queue

G0 = Path(os.environ.get("OPENARM_G0", "/home/hc_op/workspace/openarm-data-integration-plan/g0"))


class ContractServer:
    """HTTP contract service only. It does NOT represent C's worker or database."""
    def __init__(self):
        self.uploads, self.parts, self.bodies = {}, {}, {}
        self.calls, self.raws = Counter(), set()
        self.faults, self.statuses = {}, {}
        self.processing, self.quality = "PENDING", "PENDING"
        self.episode_result = None
        self.token = "synthetic-test-token"
        self.delay_put = 0
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                self.handle_request()

            def do_POST(self):
                self.handle_request()

            def do_PUT(self):
                self.handle_request()

            def handle_request(self):
                try:
                    owner.handle(self)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.base = "http://127.0.0.1:" + str(self.server.server_port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def send(self, request, payload, status=200, headers=None):
        data = canonical(payload)
        request.send_response(status)
        request.send_header("Content-Length", str(len(data)))
        for key, value in (headers or {}).items():
            request.send_header(key, value)
        request.end_headers()
        request.wfile.write(data)

    def handle(self, r):
        path = r.path.split("?")[0]
        action = ("put" if r.command == "PUT" else "create" if path.endswith("/uploads") else
                  "authorize" if path.endswith(":authorize-parts") else "complete" if path.endswith(":complete") else
                  "commit" if path.endswith(":commit") else "resume" if path.endswith(":resume") else
                  "retry-processing" if path.endswith(":retry-processing") else
                  "processing" if path.endswith("/processing") else "get")
        self.calls[action] += 1
        raw = r.rfile.read(int(r.headers.get("Content-Length", 0)))
        if len(raw) != int(r.headers.get("Content-Length", 0)):
            return  # An interrupted PUT is not a durable object-storage part.
        if action != "put" and r.headers.get("Authorization") != "Bearer " + self.token:
            return self.send(r, {"code": "ROBOT_CREDENTIAL_INVALID"}, 401)
        if action == "put":
            assert "Authorization" not in r.headers
        if self.statuses.get(action):
            status = self.statuses[action].pop(0)
            if status:
                return self.send(r, {"code": "INJECTED_FAILURE", "detail": "secret never echoed"}, status,
                                 {"Retry-After": "2"} if status == 429 else None)
        if action == "create":
            manifest = json.loads(raw)
            validate(manifest, "robot-ingest-manifest")
            uid = manifest["client_upload_id"]
            if uid not in self.uploads:
                upload = copy.deepcopy(json.loads((G0/"examples/get-upload-pending-pending.json").read_text())["data"])
                upload.update(manifest=manifest, client_upload_id=uid, upload_id=uid,
                    authenticated_robot_id=manifest["robot_id"], request_robot_id=manifest["robot_id"],
                    state="UPLOADING", raw_source_id=None,
                    total_bytes=sum(a["size_bytes"] for a in manifest["assets"]))
                upload["target"]["collection_task_id"] = manifest["collection_task_id"]
                upload["assets"] = [dict(asset_id=a["asset_id"], path=a["path"], role=a["role"], media_type=a["media_type"],
                     object_key=a["path"], multipart_upload_id="multipart-"+a["asset_id"], state="UPLOADING",
                     expected_size_bytes=a["size_bytes"], expected_sha256=a["sha256"], expected_crc64=a["crc64"]) for a in manifest["assets"]]
                self.uploads[uid] = upload
            else:
                assert self.uploads[uid]["manifest"] == manifest
            payload = {"data": self.uploads[uid], "resumed": self.calls["create"] > 1}
        elif action == "put":
            import time
            time.sleep(self.delay_put)
            _, _, uid, asset, number = path.split("/")
            number = int(number)
            self.calls[f"put:{asset}:{number}"] += 1
            self.bodies[(uid, asset, number)] = raw
            self.parts[(uid, asset, number)] = dict(part_number=number, size_bytes=len(raw), etag=hashlib.md5(raw).hexdigest())
            payload = {}
        else:
            uid = path.split("/uploads/")[1].split("/")[0].split(":")[0]
            upload = self.uploads[uid]
            if action == "get" and upload["state"] == "COMMITTED":
                upload.update(processing_status=self.processing, quality_status=self.quality)
            if action in {"authorize", "complete"}:
                asset_id = path.split("/assets/")[1].split(":")[0]
                asset = next(a for a in upload["assets"] if a["asset_id"] == asset_id)
                parts = [v for (u, a, _), v in self.parts.items() if (u, a) == (uid, asset_id)]
                if action == "authorize":
                    payload = dict(upload_id=uid, asset_id=asset_id, uploaded_parts=parts,
                        authorizations=[dict(part_number=n, url=self.base+f"/objects/{uid}/{asset_id}/{n}?signature=synthetic-secret",
                         expires_at="2099-01-01T00:00:00Z") for n in json.loads(raw)["part_numbers"]])
                else:
                    from humanoid_upload.bundle import CRC_TABLE
                    submitted = json.loads(raw)["parts"]
                    assert submitted == [dict(part_number=p["part_number"], etag=p["etag"]) for p in sorted(parts, key=lambda p:p["part_number"])]
                    data = b"".join(self.bodies[(uid, asset_id, p["part_number"])] for p in submitted)
                    assert len(data) == asset["expected_size_bytes"]
                    assert hashlib.sha256(data).hexdigest() == asset["expected_sha256"]
                    crc = 2**64-1
                    for b in data:
                        crc = CRC_TABLE[(crc ^ b)&255] ^ (crc>>8)
                    assert str(crc ^ (2**64-1)) == asset["expected_crc64"]
                    asset["state"] = "COMPLETED"
                    payload = {"data": upload, "resumed": False}
            elif action == "commit":
                assert all(a["state"] == "COMPLETED" for a in upload["assets"])
                self.raws.add(uid)
                upload.update(state="COMMITTED", raw_source_id="raw-"+uid)
                payload = {"data": upload, "resumed": False}
            elif action in {"processing", "retry-processing"}:
                if action == "retry-processing":
                    from uuid import UUID
                    request_id = str(UUID(json.loads(raw)["request_id"]))
                    if hasattr(self, "retry_request_id"):
                        assert self.retry_request_id == request_id
                    self.retry_request_id = request_id
                payload = copy.deepcopy(self.episode_result or json.loads((G0/"examples/processing-result-proposed.json").read_text()))
                payload.update(upload_id=uid, raw_source_id="raw-"+uid)
            else:
                if action == "resume":
                    upload["state"] = "UPLOADING"
                payload = {"data": upload, "resumed": False}
        if self.faults.get(action, 0):
            self.faults[action] -= 1
            r.connection.shutdown(socket.SHUT_RDWR)
            r.connection.close()
            return
        self.send(r, payload, headers={"ETag": "synthetic-etag"} if action == "put" else None)


@pytest.fixture
def server():
    instance = ContractServer()
    yield instance
    instance.close()


@pytest.fixture
def cfg(tmp_path, server, monkeypatch):
    monkeypatch.setenv("UPLOAD_TEST_TOKEN", server.token)
    monkeypatch.setenv("NO_PROXY", "127.0.0.1,localhost")
    monkeypatch.setenv("no_proxy", "127.0.0.1,localhost")
    return {**DEFAULTS, "platform_base_url": server.base, "credential_ref": "env:UPLOAD_TEST_TOKEN",
            "robot_id": "synthetic-platform-robot-01", "local_robot_id": "synthetic-openarmx-01",
            "task_id": "synthetic-task-001", "data_directory": str(G0/"fixtures"),
            "queue_directory": str(tmp_path/"queue"), "read_bytes_per_second": 1e9,
            "bandwidth_bytes_per_second": 1e9, "timeout_seconds": 2}


def due(queue):
    for job in queue.jobs():
        if not job["terminal"]:
            job["next_at"] = 0
            queue.save(job)


@pytest.fixture
def queue(cfg):
    return Queue(cfg["queue_directory"])
