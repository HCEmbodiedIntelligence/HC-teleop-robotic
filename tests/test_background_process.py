"""Actual child lifecycle and IPC regressions, without ROS or hardware."""
from concurrent.futures import ThreadPoolExecutor
import os
import json
import threading
import subprocess
import sys
from pathlib import Path
import time

import pytest

from middleware.core.background_process import (
    BackgroundProcess, BackgroundProcessError, BackgroundProcessStopped,
)
from humanoid_upload.common import UploadError


def _work(delay):
    started = time.monotonic()
    time.sleep(delay)
    return os.getpid(), started, time.monotonic()


def _public_error():
    raise UploadError('LOCAL_CONTENT_CHANGED', '保留原文件', retry=True, after=7,
                      conflict=True, platform_retry=True)


def _private_error():
    raise ValueError('never expose a transport body or credential')


def test_reuses_one_child_and_serializes_concurrent_jobs():
    worker = BackgroundProcess('test-sealer')
    try:
        assert worker.status()['pid'] is None  # No process until work arrives.
        with ThreadPoolExecutor(max_workers=4) as callers:
            results = list(callers.map(worker.call, [_work]*4, [.03]*4))
        assert len({r[0] for r in results}) == 1
        assert results[0][0] != os.getpid()
        intervals = sorted((r[1], r[2]) for r in results)
        assert all(a[1] <= b[0] for a, b in zip(intervals, intervals[1:]))
        assert worker.call(os.getenv, 'OMP_NUM_THREADS') == '1'
        assert worker.call(os.getenv, 'OPENBLAS_NUM_THREADS') == '1'
        if os.name == 'posix':
            assert worker.call(os.getpriority, os.PRIO_PROCESS, 0) >= 10
    finally:
        worker.close()
    assert not worker.status()['running']
    with pytest.raises(BackgroundProcessStopped):
        worker.call(os.getpid)


def test_preserves_public_errors_and_worker_survives_task_failure():
    worker = BackgroundProcess('test-errors')
    try:
        pid = worker.call(os.getpid)
        with pytest.raises(UploadError) as caught:
            worker.call(_public_error)
        error = caught.value
        assert (error.code, error.action, error.retry, error.after, error.conflict) == (
            'LOCAL_CONTENT_CHANGED', '保留原文件', True, 7, True)
        assert error.platform_retry is True
        with pytest.raises(BackgroundProcessError, match='ValueError') as caught:
            worker.call(_private_error)
        assert 'credential' not in str(caught.value)
        assert worker.call(os.getpid) == pid
    finally:
        worker.close()


def test_dead_child_fails_current_call_and_restarts_on_next_job():
    worker = BackgroundProcess('test-recovery')
    try:
        original = worker.call(os.getpid)
        with pytest.raises(BackgroundProcessError):
            worker.call(os._exit, 17)
        assert worker.call(os.getpid) != original
    finally:
        worker.close()


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux process lifecycle')
def test_child_exits_if_middleware_parent_dies():
    parent = subprocess.Popen([sys.executable, '-c',
        "import os,sys; from middleware.core.background_process import BackgroundProcess; "
        "worker=BackgroundProcess('test-parent-exit'); "
        "print(worker.call(os.getpid),flush=True); sys.stdin.readline()"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    child = None
    try:
        child = int(parent.stdout.readline().strip())
        parent.terminate()
        parent.wait(timeout=3)
        deadline = time.monotonic()+3
        while True:
            try:
                state = Path(f'/proc/{child}/stat').read_text().split(') ', 1)[1].split()[0]
            except FileNotFoundError:
                break
            if state == 'Z':
                break
            assert time.monotonic() < deadline, 'background work survived its parent'
            time.sleep(.02)
    finally:
        if parent.poll() is None:
            parent.terminate()
        parent.communicate(timeout=3)


def test_stop_interrupts_blocked_child_and_releases_callers():
    worker = BackgroundProcess('test-stop')
    worker.call(os.getpid)
    with ThreadPoolExecutor(max_workers=2) as callers:
        active = callers.submit(worker.call, time.sleep, 60)
        deadline = time.monotonic()+3
        while not worker.status()['busy']:
            assert time.monotonic() < deadline
            time.sleep(.01)
        queued = callers.submit(worker.call, os.getpid)
        started = time.monotonic()
        worker.close(timeout=.1)
        assert time.monotonic()-started < 3
        for future in (active, queued):
            with pytest.raises(BackgroundProcessStopped):
                future.result(timeout=2)
    worker.close()  # Shutdown is idempotent.
    assert not worker.status()['running']


def _upload_fixture(tmp_path, monkeypatch, base='http://127.0.0.1:1'):
    from humanoid_upload.bundle import checksum
    from humanoid_upload.common import atomic_json
    from middleware.core.upload_service import UploadService
    data, ready = tmp_path/'data', tmp_path/'ready'
    data.mkdir()
    monkeypatch.setenv('HC_BACKGROUND_TEST_TOKEN', 'synthetic-token')
    service = UploadService(dict(enabled=True, auto_enqueue=True, poll_seconds=.1,
        queue_directory=str(tmp_path/'queue'), platform_base_url=base,
        credential_ref='env:HC_BACKGROUND_TEST_TOKEN', robot_id='test-robot',
        local_robot_id='test-local', task_id='task'), artifact_state_dir=ready, data_directory=data)
    path = data/'capture.mcap'
    path.write_bytes(b'\x89MCAP0\r\nsynthetic-data\x89MCAP0\r\n')
    descriptor = dict(schema_version='hc-artifact-ready/v1', recording_id='capture',
        artifact_id='capture', kind='hc_mcap', root=str(data),
        files=[{'path': path.name, **checksum(path)}], profile_id='test',
        config_fingerprint='a'*64, source_artifact_id=None, native_session_id=None,
        native_completion_marker=None, binding_id=service.default_binding_id,
        capture_started_at='2026-10-22T00:00:00+00:00',
        capture_ended_at='2026-10-22T00:00:01+00:00',
        completed_at='2026-10-22T00:00:02+00:00')
    atomic_json(ready/'capture.json', descriptor)
    return service, path


def test_periodic_scan_skips_queued_bytes_but_manual_enqueue_revalidates(tmp_path, monkeypatch):
    service, source = _upload_fixture(tmp_path, monkeypatch)
    try:
        assert service.scan() == {'enqueued': 1, 'errors': {}}
        assert service.artifact_process.status()['pid'] != os.getpid()
        with monkeypatch.context() as patch:
            patch.setattr(service.artifact_process, 'call', lambda *a, **kw: pytest.fail('historical data rehashed'))
            assert service.scan() == {'enqueued': 0, 'errors': {}}
        source.write_bytes(source.read_bytes()+b'changed')
        with pytest.raises(UploadError, match='ASSET_INTEGRITY'):
            service.enqueue('capture')
        assert len(service.jobs()) == 1
    finally:
        service.close()


def test_upload_stop_releases_blocked_http_and_restart_keeps_job_identity(tmp_path, monkeypatch):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    entered, release = threading.Event(), threading.Event()
    requests = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            requests.append(request)
            entered.set()
            release.wait(10)
            body = b'{"code":"TEST_UNAVAILABLE"}'
            try:
                self.send_response(503)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except OSError:
                pass  # The stopped child deliberately closed this connection.

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    service, _ = _upload_fixture(tmp_path, monkeypatch, 'http://127.0.0.1:'+str(server.server_port))
    try:
        job = service.enqueue('capture')
        with ThreadPoolExecutor(max_workers=4) as callers:
            list(callers.map(lambda _: service.start(), range(8)))
        assert entered.wait(5)
        original = service.upload_process.status()['pid']
        assert original and original != os.getpid()
        started = time.monotonic()
        assert service.stop()['stopped']
        assert time.monotonic()-started < 6
        assert not service.upload_process.status()['running']
        release.set()
        entered.clear()
        service.start()
        assert entered.wait(5)
        deadline = time.monotonic()+5
        while service.jobs()[0]['state'] != 'retry_wait':
            assert time.monotonic() < deadline
            time.sleep(.01)
        assert service.upload_process.status()['pid'] != original
        assert len(service.jobs()) == 1 and service.jobs()[0]['id'] == job['id']
        assert len(requests) >= 2 and requests[0] == requests[1]
    finally:
        release.set()
        service.close()
        server.shutdown()
        server.server_close()
        thread.join()
