"""Acceptance checks against the real process topology, without robot hardware."""

import asyncio
import copy
import io
import json
import os
from pathlib import Path
import socket
import time
import zipfile

import pytest
from fastapi.testclient import TestClient

from hc_platform.api import create_app
from hc_platform.configuration import ConfigurationError, PackageRegistry
from hc_platform.runtime import Runtime

REPO = Path(__file__).resolve().parents[2]


def port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_builtin_packages_and_effective_configuration_hash(tmp_path):
    runtime = Runtime(REPO, tmp_path)
    for name, joints in (("x1", 17), ("openarmx", 14), ("HC_REALMAN", 12)):
        config = runtime.compile(name)
        assert len(config["joint_names"]) == joints
        assert len(config["joint_units"]) == joints
        assert config["command_interface_version"] == "hc-control-ipc/v1"
    original = runtime.compile("x1")
    changed = runtime.compile("x1", {"teleop": {"control": {"position_scale": 0.5}}})
    assert changed["config_hash"] != original["config_hash"]
    assert changed["package_hash"] == original["package_hash"]
    with pytest.raises(ConfigurationError):
        runtime.compile("x1", {"home": [1000] * 17})
    with pytest.raises(ConfigurationError):
        runtime.compile("x1", {"hardware": {"allowed_groups": ["invented_joint"]}})
    with pytest.raises(ConfigurationError):
        runtime.compile(
            "x1", {"teleop": {"control": {"axis_mapping": [[1, 0, 0]] * 3}}}
        )
    physical = Runtime(REPO, tmp_path / "physical", mode="teleop")
    with pytest.raises(ConfigurationError, match="X1 hardware adapter"):
        physical.compile("openarmx")


def test_package_zip_cannot_escape(tmp_path):
    registry = PackageRegistry(REPO / "adapters/robots", tmp_path)
    archive = tmp_path / "attack.zip"
    with zipfile.ZipFile(archive, "w") as package:
        package.writestr("../outside", "bad")
    with pytest.raises(ConfigurationError, match="Unsafe archive"):
        registry.import_zip(archive, "untrusted")
    assert not (tmp_path.parent / "outside").exists()


def wait_for(client, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    value = {}
    while time.monotonic() < deadline:
        response = client.get("/api/status")
        assert response.status_code == 200, response.text
        value = response.json()
        if predicate(value):
            return value
        time.sleep(0.05)
    raise AssertionError(json.dumps(value, indent=2))


@pytest.mark.skipif(
    not (REPO / "platform/control/build/hc-control").exists(),
    reason="Build C++ control first",
)
def test_real_api_recording_auth_stop_and_config_transaction(tmp_path, monkeypatch):
    pytest.importorskip("rclpy")
    pytest.importorskip("pybullet")
    monkeypatch.setenv("HC_PLATFORM_TOKEN", "isolated-test-token")
    runtime = Runtime(
        REPO, tmp_path, groups=["right_arm"], vr_port=port(), discovery_port=port()
    )
    with TestClient(create_app(runtime)) as client:
        state = wait_for(
            client,
            lambda value: value["control"].get("backend_ready")
            and value["backend"].get("process_alive"),
        )
        assert state["control"]["safety"] == "stopped"
        response = client.post("/api/control/operations", json={"op": "recover"})
        assert response.status_code == 401
        client.headers["Authorization"] = "Bearer isolated-test-token"
        assert (
            client.post(
                "/api/control/operations",
                json={"op": "recover", "request_id": "recover-test"},
            ).status_code
            == 202
        )
        wait_for(client, lambda value: value["control"].get("safety") == "ready")
        response = client.post(
            "/api/recordings",
            json={
                "topics": [],
                "format": "mcap",
                "request_id": "real-recording",
                "filename": "acceptance.mcap",
            },
        )
        assert response.status_code == 201, response.text
        assert (
            client.post(
                "/api/control/operations",
                json={
                    "op": "session_start",
                    "mode": "program",
                    "groups": ["right_arm"],
                },
            ).status_code
            == 202
        )
        for sequence in range(1, 31):
            current = client.get("/api/status").json()["control"]
            names = runtime.config["groups"]["right_arm"]
            positions = dict(zip(current["joint_names"], current["positions"]))
            payload = {
                key: current[key]
                for key in ("boot_id", "epoch", "lease_id", "reference_epoch")
            }
            payload.update(
                source="program",
                seq=sequence,
                ttl_ms=150,
                joint_names=names,
                positions=[
                    positions[name] + (0.005 if i == 0 else 0)
                    for i, name in enumerate(names)
                ],
            )
            response = client.post("/api/control/command", json=payload)
            assert response.status_code == 200, response.text
            time.sleep(0.015)
        response = client.post("/api/recordings/stop")
        assert response.status_code == 200, response.text
        artifact = response.json()["artifact_id"]
        dataset = client.get("/api/datasets/" + artifact).json()
        assert dataset["state"] == "READY"
        assert any(
            asset["path"].endswith(".mcap") for asset in dataset["manifest"]["assets"]
        )
        assert client.get("/api/datasets/" + artifact + "/download").status_code == 200
        assert (
            client.post("/api/control/operations", json={"op": "stop"}).status_code
            == 202
        )
        before = client.get("/api/config").json()["applied"]
        response = client.put(
            "/api/config", json={"home": [10000] * len(before["home"])}
        )
        assert response.status_code == 422, response.text
        assert (
            client.get("/api/config").json()["applied"]["config_hash"]
            == before["config_hash"]
        )
        response = client.put(
            "/api/config", json={"teleop": {"control": {"position_scale": 0.42}}}
        )
        assert response.status_code == 200, response.text
        after = wait_for(
            client,
            lambda value: value["control"].get("config_hash") != before["config_hash"],
        )
        assert after["control"]["safety"] == "stopped"
        assert after["control"]["boot_id"] != current["boot_id"]
        assert (tmp_path / "settings/x1.json").exists()
    assert all(
        child.process.returncode is not None
        for child in runtime.supervisor.children.values()
    )
    assert list((tmp_path / "logs").glob("*.jsonl"))
