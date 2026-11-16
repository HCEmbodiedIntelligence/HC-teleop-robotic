"""Actual process/IPC/UDP/URDF/motor integration, without external hardware."""

import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time

import pytest

from hc_platform.configuration import compile_package
from hc_platform.adapters.protocol import V2


class Client:
    def __init__(self, path, role="api"):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket.settimeout(2)
        self.socket.connect(str(path))
        self.sequence = 0
        assert self.request("hello", role=role)["ok"]

    def request(self, op, **fields):
        self.sequence += 1
        ident = str(self.sequence)
        self.socket.send(json.dumps(dict(op=op, id=ident, **fields)).encode())
        while True:
            data = self.socket.recv(65536)
            if not data:
                raise ConnectionError("core closed test client")
            result = json.loads(data)
            if result.get("type") == "response" and result.get("id") == ident:
                return result

    def state(self):
        return self.request("status")["state"]

    def wait(self, predicate, timeout=5):
        end = time.monotonic() + timeout
        state = {}
        while time.monotonic() < end:
            state = self.state()
            if predicate(state):
                return state
            time.sleep(0.02)
        raise AssertionError("core condition timed out: " + json.dumps(state))


class Headset:
    def __init__(self, port):
        self.port = port
        self.grip = 0.0
        self.offset = 0.0
        self.freeze = False
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run)

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        start = time.monotonic()
        sample = 0.0
        sequence = 0
        while not self.done.is_set():
            if not self.freeze:
                sample = time.monotonic() - start
            grip = self.grip
            poses = [
                0,
                1.5,
                0,
                0,
                0,
                0,
                1,
                -0.2,
                1,
                0.3,
                0,
                0,
                0,
                1,
                0.2,
                1 + self.offset,
                0.3,
                0,
                0,
                0,
                1,
            ]
            left = [0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            right = [4 if grip else 0, 0, 0, 0.25, grip, 0.0, 0.0, 0.0, 0.0]
            sock.sendto(
                V2.pack(b"PICO", 2, sequence, sample, 7, *poses, *left, *right),
                ("127.0.0.1", self.port),
            )
            sequence += 1
            self.done.wait(0.01)
        sock.close()


def free_udp_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def test_real_core_vr_ik_and_pybullet_pipeline():
    pytest.importorskip("pybullet")
    repo = Path(__file__).resolve().parents[2]
    binary = repo / "platform/control/build/hc-control"
    if not binary.exists():
        pytest.skip("build control core before process integration test")
    children = []
    streams = []
    api = None
    headset = None
    with tempfile.TemporaryDirectory(prefix="hc-adapter-e2e-") as directory:
        root = Path(directory)
        config = compile_package(repo / "adapters/robots/x1")
        config.update(
            allowed_groups=["right_arm"],
            enable_base=False,
            enable_grippers=False,
            vr_udp_port=free_udp_port(),
            discovery_port=free_udp_port(),
            status_port=free_udp_port(),
            vr_bind_host="127.0.0.1",
            runtime_state_dir=str(root),
        )
        path = root / "config.json"
        path.write_text(json.dumps(config))
        sock = root / "control.sock"
        env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
        env["PYTHONPATH"] = (
            str(repo / ".platform-deps")
            + os.pathsep
            + str(repo / "platform/python")
            + os.pathsep
            + os.environ.get("PYTHONPATH", "")
        )

        def launch(name, args):
            stream = (root / (name + ".log")).open("w+")
            streams.append(stream)
            process = subprocess.Popen(
                args, cwd=repo, env=env, stdout=stream, stderr=subprocess.STDOUT
            )
            children.append(process)

        try:
            launch("core", [str(binary), "--socket", str(sock), "--config", str(path)])
            deadline = time.monotonic() + 5
            while not sock.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert sock.exists()
            api = Client(sock)
            for role in ("backend", "ik", "vr"):
                launch(
                    role,
                    [
                        sys.executable,
                        "-m",
                        "hc_platform.adapters.runtime",
                        "--role",
                        role,
                        "--config",
                        str(path),
                        "--socket",
                        str(sock),
                        "--state-dir",
                        str(root),
                    ],
                )
            api.wait(
                lambda s: s["backend_ready"] and s["ik_connected"] and s["vr_connected"]
            )
            assert api.state()["safety"] == "stopped"
            assert api.request("recover")["ok"]
            api.wait(lambda s: s["safety"] == "ready" and not s["transition_pending"])
            assert api.request("session_start", mode="teleop", groups=["right_arm"])[
                "ok"
            ]
            api.wait(lambda s: s["owner"] == "vr" and not s["transition_pending"])
            headset = Headset(config["vr_udp_port"])
            headset.thread.start()
            api.wait(lambda s: not s["release_required"]["right"])
            initial = api.state()
            initial_q = dict(zip(initial["joint_names"], initial["positions"]))
            headset.grip = 0.8
            api.wait(lambda s: s["grip"]["right"])
            headset.offset = 0.1
            moved = api.wait(
                lambda s: s["accepted_commands"] > 5
                and any(
                    abs(q - initial_q[n]) > 0.015
                    for n, q in zip(s["joint_names"], s["positions"])
                ),
                timeout=7,
            )
            assert moved["safety"] == "ready"
            # Transport continues at 100 Hz with increasing uint32 sequence, but
            # the producer timestamp freezes: this MUST stop the source.
            headset.freeze = True
            stopped = api.wait(lambda s: s["safety"] in ("stopped", "fault"), timeout=2)
            assert stopped["owner"] == "none"
            assert (
                "STALE" in stopped["reason"]
                or "TIMEOUT" in stopped["reason"]
                or "INPUT" in stopped["reason"]
                or stopped["reason"] == "SOFTWARE_STOP"
            )
            headset.freeze = False
            time.sleep(0.2)
            assert api.state()["safety"] in ("stopped", "fault")
        except Exception:
            for stream in streams:
                stream.flush()
                stream.seek(0)
                print(stream.name + "\n" + stream.read())
            for status_path in root.glob("*.json"):
                if status_path.name != "config.json":
                    print(status_path.name + " " + status_path.read_text())
            raise
        finally:
            if headset:
                headset.done.set()
                headset.thread.join(2)
            if api:
                api.socket.close()
            for process in reversed(children):
                process.terminate()
            for process in reversed(children):
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            for stream in streams:
                stream.close()
