import copy
import hashlib
import json
import os
import shutil
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

import pytest

from humanoid_upload.bundle import checksum
from humanoid_upload.cli import scan
from humanoid_upload.common import UploadError, atomic_json, canonical, credential, load_config
from humanoid_upload.http import retry_after
from humanoid_upload.queue import Queue, public_job
from humanoid_upload.worker import Worker, state_for
from conftest import G0, due


def enqueue(queue, cfg):
    return queue.enqueue(G0/"fixtures/valid-openarm", cfg)


def tick(queue, cfg):
    with queue.worker_lock():
        Worker(queue, cfg).tick()
    return queue.jobs()[0]


def copied(tmp_path, cfg):
    root = tmp_path/"exports"/"sample"
    shutil.copytree(G0/"fixtures/valid-openarm", root)
    cfg["data_directory"] = str(root.parent)
    return root


def reseal(root):
    marker = json.loads((root/"export-complete.json").read_text())
    marker["assets"] = [{"path": p.relative_to(root).as_posix(), **checksum(p)} for p in sorted(root.rglob("*"))
                        if p.is_file() and p.name != "export-complete.json"]
    marker["content_sha256"] = hashlib.sha256(canonical(marker["assets"])).hexdigest()
    context = json.loads((root/"capture-context.json").read_text())
    for key in ("export_id", "collection_task_id"):
        marker[key] = context[key]
    atomic_json(root/"export-complete.json", marker)


def test_schema_provenance():
    from humanoid_upload import common
    directory = Path(common.__file__).parent/"schemas"
    provenance = json.loads((directory/"provenance.json").read_text())
    assert provenance["release"] == "hc-shared-capture-20261021.2"
    assert provenance["upstream_release"] == "g0-20260922.1"
    for name, sha in provenance["files"].items():
        assert hashlib.sha256((directory/Path(name).name).read_bytes()).hexdigest() == sha


def test_crc64_vector(tmp_path):
    path = tmp_path/"vector"
    path.write_bytes(b"123456789")
    assert checksum(path)["crc64"] == str(0x995DC9BBDF1939FA)


@pytest.mark.parametrize("fixture", ["missing-video", "wrong-hash", "incomplete-export", "invalid-segment",
                                      "wrong-dimension", "future-action", "single-arm-paused"])
def test_public_negative_fixtures_rejected(fixture, queue, cfg):
    with pytest.raises(UploadError):
        queue.enqueue(G0/"fixtures"/fixture, cfg)
    assert queue.jobs() == []


def test_generic_fixture(queue, cfg):
    context = json.loads((G0/"fixtures/valid-generic/capture-context.json").read_text())
    cfg.update(local_robot_id=context["local_robot_id"], robot_id=context["robot_id"])
    job = queue.enqueue(G0/"fixtures/valid-generic", cfg)
    assert len(job["manifest"]["cameras"]) == 1
    assert tick(queue, cfg)["state"] == "raw_committed"


def test_duplicate_notices_and_complete_http_inventory(queue, cfg, server):
    job = enqueue(queue, cfg)
    assert enqueue(queue, cfg)["id"] == job["id"]
    assert Queue(cfg["queue_directory"]).jobs()[0]["id"] == job["id"]
    assert not server.calls
    result = tick(queue, cfg)
    assert result["state"] == "raw_committed"
    assert len(server.raws) == 1
    for asset in job["manifest"]["assets"]:
        assert server.bodies[(job["id"], asset["asset_id"], 1)] == (Path(job["root"])/asset["path"]).read_bytes()
    assert result["raw_source_id"]
    assert len(queue.jobs()) == 1
    text = canonical(queue.jobs()).decode()
    assert "synthetic-test-token" not in text and "signature=" not in text


@pytest.mark.parametrize("action", ["create", "put", "complete", "commit"])
def test_response_lost_restarts_real_cli(action, queue, cfg, server, tmp_path):
    job = enqueue(queue, cfg)
    path = tmp_path/"config.json"
    atomic_json(path, cfg)
    server.faults[action] = 1
    def step():
        result = subprocess.run([sys.executable, "-m", "humanoid_upload.cli", "--config", str(path), "step"],
                                capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout+result.stderr
    step()
    assert queue.jobs()[0]["state"] == "retry_wait"
    assert queue.jobs()[0]["client_upload_id"] == job["id"]
    due(queue)
    step()
    assert queue.jobs()[0]["state"] == "raw_committed"
    assert len(server.raws) == len(server.uploads) == 1
    assert server.calls["put"] == len(job["manifest"]["assets"])
    if action == "commit":
        assert server.calls["commit"] == 1


def test_reconciled_response_accepts_equivalent_server_utc_timestamps(queue,cfg,server):
    job=enqueue(queue,cfg)
    assert tick(queue,cfg)['state']=='raw_committed'
    remote=copy.deepcopy(server.uploads[job['id']])
    def canonical_server_dates(value):
        if isinstance(value,dict):
            for key,item in value.items():
                if key.endswith('_at') and isinstance(item,str) and item.endswith('Z'):
                    value[key]=item[:-1]+'+00:00'
                else:
                    canonical_server_dates(item)
        elif isinstance(value,list):
            for item in value:canonical_server_dates(item)
    canonical_server_dates(remote['manifest'])
    assert Worker(queue,cfg).facts(queue.jobs()[0],{'data':remote})['state']=='COMMITTED'
    assert len(server.raws)==len(server.uploads)==1
    assert queue.jobs()[0]['id']==job['id']


def test_repeated_disconnect_backoff_and_url_expiry(queue, cfg, server):
    enqueue(queue, cfg)
    server.faults["create"] = 3
    delays = []
    for _ in range(3):
        before = time.time()
        job = tick(queue, cfg)
        delays.append(job["next_at"]-before)
        before_calls = dict(server.calls)
        tick(queue, cfg)
        assert dict(server.calls) == before_calls
        due(queue)
    assert 1 <= delays[0] < 2 and 2 <= delays[1] < 3 and 4 <= delays[2] < 5.5
    server.statuses["put"] = [403]
    assert tick(queue, cfg)["state"] == "retry_wait"
    due(queue)
    assert tick(queue, cfg)["state"] == "raw_committed"
    assert len(server.raws) == 1


def test_retry_after_and_credential_recovery(queue, cfg, server, monkeypatch):
    job = enqueue(queue, cfg)
    server.statuses["create"] = [429]
    before = time.time()
    assert tick(queue, cfg)["next_at"] >= before + 2
    due(queue)
    monkeypatch.setenv("UPLOAD_TEST_TOKEN", "bad-token")
    assert tick(queue, cfg)["terminal"] is True
    assert queue.jobs()[0]["code"] == "ROBOT_CREDENTIAL_INVALID"
    calls = dict(server.calls)
    tick(queue, cfg)
    assert dict(server.calls) == calls
    monkeypatch.setenv("UPLOAD_TEST_TOKEN", server.token)
    queue.retry(job["id"])
    assert tick(queue, cfg)["state"] == "raw_committed"


@pytest.mark.parametrize("state", ["PAUSED", "CANCELLED", "FAILED"])
def test_server_terminal_states(queue, cfg, server, state):
    job = enqueue(queue, cfg)
    server.faults["create"] = 1
    tick(queue, cfg)
    server.uploads[job["id"]]["state"] = state
    due(queue)
    assert tick(queue, cfg)["state"] == "needs_attention"
    assert not server.raws
    calls = dict(server.calls)
    tick(queue, cfg)
    assert dict(server.calls) == calls
    if state == "PAUSED":
        queue.retry(job["id"], resume=True)
        assert tick(queue, cfg)["state"] == "raw_committed"
    else:
        with pytest.raises(UploadError, match="UPLOAD_TERMINAL"):
            queue.retry(job["id"])


def test_expired_session_and_conflict(queue, cfg, server):
    enqueue(queue, cfg)
    server.statuses["create"] = [410]
    assert tick(queue, cfg)["terminal"]
    assert not server.raws


def test_conflict_reconciles_then_stops_and_422_is_terminal(queue, cfg, server):
    job = enqueue(queue, cfg)
    server.statuses["create"] = [409, 409, 409]
    for i in range(3):
        result = tick(queue, cfg)
        assert result["terminal"] == (i == 2)
        due(queue)
    assert not server.raws
    queue.retry(job["id"])
    server.statuses["create"] = [422]
    assert tick(queue, cfg)["terminal"]


def test_server_identity_change_stops(queue, cfg, server):
    job = enqueue(queue, cfg)
    server.faults["create"] = 1
    tick(queue, cfg)
    server.uploads[job["id"]]["authenticated_robot_id"] = "wrong-robot"
    due(queue)
    result = tick(queue, cfg)
    assert result["code"] == "SERVER_IDENTITY_MISMATCH" and result["terminal"]
    assert not server.raws


def test_public_upload_url_not_rewritten(cfg):
    import io
    from humanoid_upload.http import Http
    cfg["platform_base_url"] = "https://platform.example.test"
    with pytest.raises(UploadError, match="UPLOAD_URL_NOT_EXTERNAL"):
        Http(cfg).put("http://127.0.0.1:9000/object?signature=secret", io.BytesIO(b"a"), 1)


def test_ready_with_pending_quality_needs_attention():
    assert state_for("COMMITTED", "READY", "PENDING")[:2] == ("needs_attention", True)


@pytest.mark.parametrize("key,value", [("robot_id", "other"), ("local_robot_id", "other"), ("task_id", "other")])
def test_identity_reject_before_enqueue(queue, cfg, key, value):
    cfg[key] = value
    with pytest.raises(UploadError):
        enqueue(queue, cfg)
    assert queue.jobs() == []


def test_target_change_frozen_and_new_export(queue, cfg, server, tmp_path):
    root = copied(tmp_path, cfg)
    first = queue.enqueue(root, cfg)
    cfg["task_id"] = "new-task"
    # The worker uses the frozen task, even with a changed global default.
    assert tick(queue, cfg)["state"] == "raw_committed"
    assert server.uploads[first["id"]]["target"]["collection_task_id"] == "synthetic-task-001"
    new = root.parent/"new"
    shutil.copytree(root, new)
    context = json.loads((new/"capture-context.json").read_text())
    context.update(collection_task_id="new-task", export_id="new-export")
    atomic_json(new/"capture-context.json", context)
    reseal(new)
    second = queue.enqueue(new, cfg)
    assert first["id"] != second["id"]


def test_changed_bytes_never_reuse_uuid(queue, cfg, tmp_path, server):
    root = copied(tmp_path, cfg)
    job = queue.enqueue(root, cfg)
    (root/"export.json").write_text("changed")
    assert tick(queue, cfg)["state"] == "needs_attention"
    assert not server.calls
    reseal(root)
    with pytest.raises(UploadError, match="EXPORT_ID_REUSED"):
        queue.enqueue(root, cfg)
    assert queue.jobs()[0]["id"] == job["id"]


@pytest.mark.parametrize("kind", ["symlink", "extra", "half_mp4", "unsafe"])
def test_bad_inventory_and_half_mp4(kind, queue, cfg, tmp_path):
    root = copied(tmp_path, cfg)
    if kind == "symlink":
        (root/"link").symlink_to(root/"export.json")
    elif kind == "extra":
        (root/"extra.txt").write_text("extra")
    elif kind == "half_mp4":
        path = next(root.rglob("*.mp4"))
        path.write_bytes(path.read_bytes()[:64])
        reseal(root)
    else:
        marker = json.loads((root/"export-complete.json").read_text())
        marker["assets"][0]["path"] = "../outside"
        atomic_json(root/"export-complete.json", marker)
    with pytest.raises(UploadError):
        queue.enqueue(root, cfg)


def test_scan_compensates_crash_before_enqueue(queue, cfg, tmp_path):
    root = copied(tmp_path, cfg)
    failed = root.parent/".export-failed"
    shutil.copytree(root, failed)
    assert len(scan(queue, cfg)) == 1
    assert scan(queue, cfg) == []
    assert len(queue.jobs()) == 1


def test_concurrent_worker_and_enqueue(queue, cfg):
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=4) as pool:
        ids = list(pool.map(lambda _: enqueue(Queue(cfg["queue_directory"]), cfg)["id"], range(4)))
    assert len(set(ids)) == 1
    with queue.worker_lock():
        with pytest.raises(UploadError, match="WORKER_ALREADY_RUNNING"):
            with Queue(cfg["queue_directory"]).worker_lock():
                pytest.fail("second worker entered")
        with pytest.raises(UploadError, match="WORKER_ALREADY_RUNNING"):
            queue.retry(ids[0])


@pytest.mark.parametrize("example", sorted((G0/"examples").glob("get-upload-*.json")))
def test_public_status_examples_stop_polling(example, queue, cfg, server):
    enqueue(queue, cfg)
    tick(queue, cfg)
    data = json.loads(example.read_text())["data"]
    server.processing, server.quality = data["processing_status"], data["quality_status"]
    due(queue)
    job = tick(queue, cfg)
    expected, terminal, _ = state_for("COMMITTED", server.processing, server.quality)
    assert (job["state"], job["terminal"]) == (expected, terminal)
    if terminal:
        calls = dict(server.calls)
        tick(queue, cfg)
        assert dict(server.calls) == calls


def test_episode_partial_failure_and_retry_never_second_raw(queue, cfg, server):
    cfg["processing_endpoint"] = True
    enqueue(queue, cfg)
    job = tick(queue, cfg)
    assert job["state"] == "processing_failed"
    assert job["episodes"][0]["next_action"] == "open_episode"
    assert job["episodes"][1]["error_code"] == "OPENARM_VIDEO_MISSING"
    calls = dict(server.calls)
    tick(queue, cfg)
    assert dict(server.calls) == calls
    queue.retry(job["id"])
    tick(queue, cfg)
    assert server.calls["commit"] == 1 and len(server.raws) == 1


def test_lost_processing_response_keeps_polling(queue, cfg, server):
    cfg["processing_endpoint"] = True
    enqueue(queue, cfg)
    server.faults["processing"] = 1
    assert tick(queue, cfg)["state"] == "retry_wait"
    due(queue)
    assert tick(queue, cfg)["state"] == "processing_failed"
    assert server.calls["commit"] == 1


def test_explicit_processing_retry_is_idempotent(queue, cfg, server):
    cfg["processing_endpoint"] = True
    enqueue(queue, cfg)
    job = tick(queue, cfg)
    queue.retry(job["id"], processing=True)
    request_id = queue.jobs()[0]["processing_retry_request"]
    server.faults["retry-processing"] = 1
    assert tick(queue, cfg)["state"] == "retry_wait"
    due(queue)
    assert tick(queue, cfg)["state"] == "processing_failed"
    assert server.retry_request_id == request_id
    assert server.calls["commit"] == 1 and len(server.raws) == 1


def test_read_only_queue_failure_does_not_send(queue, cfg, monkeypatch, server):
    enqueue(queue, cfg)
    def fail(*_):
        raise sqlite3.OperationalError("database or disk is full")
    monkeypatch.setattr(queue, "save", fail)
    with pytest.raises(sqlite3.Error):
        tick(queue, cfg)
    assert server.calls["put"] == 0 and not server.raws


def test_credentials_and_config_permissions(tmp_path, cfg):
    secret = tmp_path/"secret"
    secret.write_text("only-test-secret")
    secret.chmod(0o644)
    with pytest.raises(UploadError):
        credential("file:"+str(secret))
    secret.chmod(0o600)
    assert credential("file:"+str(secret)) == "only-test-secret"
    cfg["token"] = "not-allowed"
    path = tmp_path/"config.json"
    atomic_json(path, cfg)
    with pytest.raises(UploadError):
        load_config(path)
    assert retry_after("Thu, 01 Jan 1970 00:00:00 GMT") == 0
