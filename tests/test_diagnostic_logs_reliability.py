"""Logging must survive bad data and preserve safe, complete view cursors."""
import asyncio
import json
import logging
from pathlib import Path
import threading
import time

from aiohttp import web
from aiohttp.test_utils import make_mocked_request
import pytest

from middleware.core.diagnostic_logs import (
    DiagnosticLogHandler, DiagnosticLogs, _text, register_log_routes,
)


def wait_for(service, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = service.snapshot(limit=500, priority=False)
        if predicate(result):
            return result
        time.sleep(.01)
    raise AssertionError(result)


@pytest.mark.parametrize('text,secret', [
    ('password=not-for-log', 'not-for-log'),
    ('{"token":"not-for-log"}', 'not-for-log'),
    ('Authorization: Bearer not-for-log', 'not-for-log'),
    ('Authorization: Basic dXNlcjpwYXNz', 'dXNlcjpwYXNz'),
    ('--control-token not-for-log', 'not-for-log'),
    ('https://username:not-for-log@example.com/path', 'not-for-log'),
])
def test_secrets_are_redacted_without_formatting_errors(text, secret):
    clean = _text(text)
    assert secret not in clean
    assert '[REDACTED]' in clean


def test_secret_in_saved_log_keeps_worker_alive_and_never_reaches_disk(tmp_path):
    folder = tmp_path / 'runtime/teleop_logs/session_test/middleware'
    folder.mkdir(parents=True)
    (folder / 'middleware.log').write_text(
        '[ERROR] Authorization: Bearer never-write-this\n[INFO] tail ready\n')
    service = DiagnosticLogs(tmp_path, folder, poll_interval=.02).start()
    try:
        wait_for(service, lambda r: len(r['entries']) >= 2)
        service.log('error', 'new error after credential line')
        result = wait_for(service, lambda r: any('new error after' in e['message'] for e in r['entries']))
        assert service._thread.is_alive()
        assert 'never-write-this' not in json.dumps(result)
        assert 'never-write-this' not in service.journal.read_text()
    finally:
        service.close()


def test_malformed_saved_record_does_not_kill_worker_or_erase_error(tmp_path, monkeypatch):
    folder = tmp_path / 'runtime/teleop_logs/session_test/middleware'
    folder.mkdir(parents=True)
    (folder / 'middleware.log').write_text('[ERROR] bad record\n[INFO] good record\n')
    service = DiagnosticLogs(tmp_path, folder, poll_interval=.02)
    original = service._raw_entry

    def parse(text, *args, **kwargs):
        if 'bad record' in text:
            raise ValueError('unreadable record token=private-value')
        return original(text, *args, **kwargs)

    monkeypatch.setattr(service, '_raw_entry', parse)
    service.start()
    try:
        wait_for(service, lambda r: r['processing_failures'] > 0)
        service.log('info', 'subsequent write succeeded')
        result = wait_for(service, lambda r: any('subsequent write' in e['message'] for e in r['entries']))
        assert service._thread.is_alive()
        assert result['persistence_error']
        assert result['processing_error']
        assert 'private-value' not in result['processing_error']
        assert service.persistence_error == ''  # Last disk write succeeded.
    finally:
        service.close()


def test_python_warning_and_standard_runtime_warning_keep_severity(tmp_path):
    service = DiagnosticLogs(tmp_path, poll_interval=.02).start()
    try:
        handler = DiagnosticLogHandler(service)
        handler.handle(logging.LogRecord('example', logging.WARNING, __file__, 1,
                                         'python warning', (), None))
        service.log('warning', 'runtime warning')
        wait_for(service, lambda r: len(r['entries']) >= 2)
        rows = service.snapshot(level='warning')['entries']
        assert len(rows) == 2 and all(r['level'] == 'warning' for r in rows)
    finally:
        service.close()


def test_unwritable_log_directory_keeps_in_memory_service_available(tmp_path, monkeypatch):
    folder = tmp_path / 'runtime/middleware_logs/denied_session'
    service = DiagnosticLogs(tmp_path, folder, poll_interval=.02)
    original_mkdir = Path.mkdir

    def deny_log_directory(path, *args, **kwargs):
        if path == folder:
            raise PermissionError('log directory is read-only')
        return original_mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'mkdir', deny_log_directory)
    assert service.start() is service
    try:
        assert service.persistence_error
        assert service.log('error', 'available without a writable journal')
        result = wait_for(service, lambda r: bool(r['entries']))
        assert service._thread.is_alive()
        assert result['entries'][0]['message'] == 'available without a writable journal'
        assert result['persistence_error']
        assert not service.journal.exists()
    finally:
        service.close()


def test_blocked_file_writer_cannot_block_producers_and_overflow_is_visible(tmp_path, monkeypatch):
    service = DiagnosticLogs(tmp_path, queue_size=1, poll_interval=.02)
    entered, release = threading.Event(), threading.Event()
    original = service._write

    def blocked_write(entry):
        if not entered.is_set():
            entered.set()
            assert release.wait(2)
        original(entry)

    monkeypatch.setattr(service, '_write', blocked_write)
    service.start()
    try:
        assert service.log('error', 'first')
        assert entered.wait(1)
        began = time.monotonic()
        assert service.log('info', 'second')
        assert not service.log('warning', 'third')
        assert time.monotonic() - began < .1
        assert service._dropped == 1
        release.set()
        result = wait_for(service, lambda r: any(e['kind'] == 'log_gap' for e in r['entries']))
        assert result['dropped'] == 1
    finally:
        release.set()
        service.close()


def test_historical_multiple_files_keep_monotonic_unique_cursor_ids(tmp_path):
    root = tmp_path / 'runtime/teleop_logs/session_old'
    root.mkdir(parents=True)
    (root / 'a.log').write_text(''.join(f'[ERROR] a{i}\n' for i in range(30)))
    (root / 'b.log').write_text(''.join(f'[ERROR] b{i}\n' for i in range(5)))
    service = DiagnosticLogs(tmp_path, max_entries=10)
    session = service._session_id(root)
    cursor, entries = 0, []
    while True:
        page = service.snapshot(session, after=cursor, limit=3, priority=False)
        entries.extend(page['entries'])
        assert page['cursor'] > cursor
        cursor = page['cursor']
        if not page['has_more']:
            break
    assert [e['id'] for e in entries] == list(range(26, 36))
    assert [e['message'] for e in entries][-5:] == [f'[ERROR] b{i}' for i in range(5)]
    exported = [json.loads(line) for line in service.export(session).splitlines()]
    assert [e['id'] for e in exported] == list(range(26, 36))


def test_error_preview_is_global_filtered_and_never_skips_cursor(tmp_path):
    service = DiagnosticLogs(tmp_path, max_entries=500)
    for i in range(250):
        event = {'kind': 'log', 'source': 'control' if i == 249 else 'middleware',
                 'payload': {'level': 'error' if i == 249 else 'info', 'message': f'row-{i}'}}
        service._append(service._normalize(event), persist=False)
    first = service.snapshot(limit=10)
    assert first['cursor'] == 10 and first['has_more']
    assert [e['id'] for e in first['priority_entries']] == [250]
    assert service.snapshot(after=first['cursor'], limit=10)['cursor'] == 20
    assert service.snapshot(q='does not exist')['priority_entries'] == []
    assert service.snapshot(subsystem='hardware')['priority_entries'] == []
    assert service.snapshot(level='critical')['priority_entries'] == []


def test_rotation_restart_recovers_sanitized_journal_and_next_id(tmp_path):
    folder = tmp_path / 'runtime/middleware_logs/session_one'
    service = DiagnosticLogs(tmp_path, folder, max_bytes=512, backups=3, poll_interval=.02).start()
    for i in range(15):
        service.log('info', f'line {i} password=never-store-this')
    service.close()
    paths = sorted(folder.glob('diagnostic_events.jsonl*'))
    assert 1 < len(paths) <= 4
    assert all('never-store-this' not in path.read_text() for path in paths)
    service = DiagnosticLogs(tmp_path, folder, max_bytes=512, backups=3, poll_interval=.02).start()
    try:
        before = service.snapshot()['cursor']
        assert before == 15
        service.log('error', 'after restart')
        result = wait_for(service, lambda r: any(e['message'] == 'after restart' for e in r['entries']))
        assert result['cursor'] == 16
        assert len({e['id'] for e in result['entries']}) == len(result['entries'])
    finally:
        service.close()


def test_api_rejects_path_queries_and_symlink_source(tmp_path):
    async def exercise():
        root = tmp_path / 'runtime/teleop_logs/session_history'
        root.mkdir(parents=True)
        outside = tmp_path / 'private.log'
        outside.write_text('[ERROR] do-not-read-private-source\n')
        (root / 'escape.log').symlink_to(outside)
        service = DiagnosticLogs(tmp_path)
        app = web.Application()
        register_log_routes(app, service)
        # Exercise actual registered route handlers without opening a listener;
        # the logger remains a read-only application component in this test.
        async def get(url):
            request = make_mocked_request('GET', url)
            route = await app.router.resolve(request)
            return await route.handler(request)

        with pytest.raises(web.HTTPBadRequest):
            await get('/api/logs?session=../../private.log')
        with pytest.raises(web.HTTPBadRequest):
            await get('/api/logs/export?session=' + str(outside))
        response = await get('/api/logs?session=' + service._session_id(root))
        assert response.status == 200
        body = json.loads(response.text)
        assert body['entries'] == []
        assert 'do-not-read-private-source' not in json.dumps(body)
    asyncio.run(exercise())
