"""A live infinite HTTP stream must not postpone the actuator stop or cleanup."""

import asyncio
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import httpx
import pytest

from hc_platform.ipc import CoreClient

REPO = Path(__file__).resolve().parents[2]
SERVER = r"""
import asyncio, sys
from pathlib import Path
from fastapi.responses import StreamingResponse
import uvicorn
from hc_platform.api import create_app, PlatformServer
from hc_platform.runtime import Runtime
runtime = Runtime(Path(sys.argv[1]), Path(sys.argv[2]), groups=['right_arm'], vr_port=int(sys.argv[4]), discovery_port=int(sys.argv[5]))
app = create_app(runtime)
@app.get('/test/infinite')
async def infinite():
    async def chunks():
        while True:
            yield b'--frame\r\n' + b'x'*1024 + b'\r\n'
            await asyncio.sleep(.02)
    return StreamingResponse(chunks(), media_type='multipart/x-mixed-replace; boundary=frame')
app.router.routes.insert(0, app.router.routes.pop())  # Before the console's catch-all mount.
PlatformServer(uvicorn.Config(app, host='127.0.0.1', port=int(sys.argv[3]), access_log=False), runtime).run()
"""


def free_port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


@pytest.mark.skipif(
    not (REPO / "platform/control/build/hc-control").exists(),
    reason="Build C++ core first",
)
def test_signal_stops_core_before_long_http_stream_drains_and_reaps_children(tmp_path):
    pytest.importorskip("rclpy")
    pytest.importorskip("pybullet")
    port = free_port()
    environment = os.environ.copy()
    environment.pop("HC_PLATFORM_TOKEN", None)
    log = (tmp_path / "server.log").open("w")
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            SERVER,
            str(REPO),
            str(tmp_path),
            str(port),
            str(free_port(socket.SOCK_DGRAM)),
            str(free_port(socket.SOCK_DGRAM)),
        ],
        env=environment,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    children = []
    try:
        with httpx.Client(base_url=f"http://127.0.0.1:{port}", timeout=2) as client:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path / "server.log").read_text()
                try:
                    response = client.get("/api/status")
                    state = response.json()
                    children = [item["pid"] for item in state["processes"]]
                    if state["control"].get("backend_ready"):
                        break
                except (httpx.RequestError, ValueError, KeyError):
                    pass
                time.sleep(0.05)
            else:
                pytest.fail((tmp_path / "server.log").read_text())
            assert (
                client.post(
                    "/api/control/operations", json={"op": "recover"}
                ).status_code
                == 202
            )
            assert (
                client.post(
                    "/api/control/operations",
                    json={"op": "session_start", "mode": "program"},
                ).status_code
                == 202
            )
            suffix = hashlib.sha256(str(tmp_path.resolve()).encode()).hexdigest()[:10]
            path = f"/tmp/hc-platform-{os.getuid()}-{suffix}/control.sock"

            async def observe_stop():
                core = CoreClient(path)
                try:
                    await core.connect()
                    assert (await core.request("status"))["state"]["safety"] == "ready"
                    started = time.monotonic()
                    process.send_signal(signal.SIGTERM)
                    while time.monotonic() - started < 1.5:
                        state = (await core.request("status"))["state"]
                        if state["safety"] == "stopped":
                            assert process.poll() is None  # HTTP is still draining.
                            return
                        await asyncio.sleep(0.01)
                    pytest.fail("Core did not stop before HTTP graceful drain")
                finally:
                    await core.close()

            with client.stream("GET", "/test/infinite") as stream:
                assert stream.status_code == 200
                assert next(stream.iter_bytes()).startswith(b"--frame")
                asyncio.run(observe_stop())
                process.wait(timeout=20)
        assert all(not Path(f"/proc/{pid}").exists() for pid in children)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        for pid in children:
            if Path(f"/proc/{pid}").exists():
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        log.close()
