import io
import os
import time
from urllib.error import HTTPError

from humanoid_upload.common import UploadError, canonical
from humanoid_upload.http import Http
from humanoid_upload.queue import Queue
from humanoid_upload.worker import Worker


def add_job(queue, number, platform='https://platform.example'):
    job = {
        'id': f'job-{number}', 'client_upload_id': f'job-{number}',
        'root': '/unused', 'frozen': {'platform_base_url': platform},
        'manifest': {'assets': []}, 'state': 'queued', 'terminal': False,
        'next_at': 0, 'attempts': 0, 'upload_id': None, 'code': None,
        'next_action': '等待上传', 'created_at': time.time(), 'updated_at': time.time(),
    }
    with queue.connect() as db:
        db.execute('INSERT INTO jobs(id,dedup,export_key,payload,state) VALUES(?,?,?,?,?)',
                   (job['id'], f'dedup-{number}', f'export-{number}',
                    canonical(job).decode(), job['state']))
    return job


def test_platform_outage_stops_sibling_jobs_and_survives_restart(tmp_path, monkeypatch):
    queue = Queue(tmp_path/'queue')
    for number in range(3):
        add_job(queue, number)
    calls = []

    def offline(_worker, job):
        calls.append(job['id'])
        raise UploadError('NETWORK_DNS_FAILED', 'offline', retry=True, platform_retry=True)

    monkeypatch.setattr(Worker, 'process', offline)
    Worker(queue, {'retry_base_seconds': 1, 'retry_max_seconds': 60, 'max_attempts': 0}).tick()
    assert calls == ['job-0']
    jobs = queue.jobs()
    assert jobs[0]['state'] == 'retry_wait' and jobs[0]['attempts'] == 1
    assert all(job['state'] == 'queued' and job['attempts'] == 0 for job in jobs[1:])
    gate = queue.platform_backoff('https://platform.example')
    assert gate['code'] == 'NETWORK_DNS_FAILED' and gate['next_at'] > time.time()

    # A new Queue/Worker represents a restarted child or middleware process.
    restarted = Queue(tmp_path/'queue')
    Worker(restarted, {'retry_base_seconds': 1, 'retry_max_seconds': 60, 'max_attempts': 0}).tick()
    assert calls == ['job-0']

    restarted.set_platform_backoff('https://platform.example', gate['failures'], 0, gate['code'])
    restarted.retry('job-0')
    assert restarted.platform_backoff('https://platform.example') is None

    def online(_worker, job):
        calls.append(job['id'])
        job.update(state='raw_committed', terminal=True, next_at=0)

    monkeypatch.setattr(Worker, 'process', online)
    Worker(restarted, {'retry_base_seconds': 1, 'retry_max_seconds': 60, 'max_attempts': 0}).tick()
    assert calls[1:] == ['job-0', 'job-1', 'job-2']
    assert restarted.platform_backoff('https://platform.example') is None


def test_platform_outage_does_not_block_another_platform(tmp_path, monkeypatch):
    queue = Queue(tmp_path/'queue')
    add_job(queue, 0, 'https://offline.example')
    add_job(queue, 1, 'https://online.example')
    calls = []

    def process(_worker, job):
        calls.append(job['id'])
        if job['id'] == 'job-0':
            raise UploadError('HTTP_530', 'offline', retry=True, platform_retry=True)
        job.update(state='raw_committed', terminal=True, next_at=0)

    monkeypatch.setattr(Worker, 'process', process)
    Worker(queue, {'retry_base_seconds': 1, 'retry_max_seconds': 60, 'max_attempts': 0}).tick()
    assert calls == ['job-0', 'job-1']


def test_verified_file_is_not_rehashed_until_kernel_identity_changes(tmp_path, monkeypatch):
    import humanoid_upload.worker as worker_module

    path = tmp_path/'recording.mcap'
    path.write_bytes(b'first')
    manifest = {'assets': [{'path': path.name}]}
    job = {'id': 'job', 'root': str(tmp_path), 'manifest': manifest}

    class FakeQueue:
        def cancelled(self, _job_id): return False
        def save(self, _job): pass

    calls = []
    monkeypatch.setattr(worker_module, 'manifest_for',
                        lambda *_: (calls.append(True) or (tmp_path, {}, manifest)))
    worker = Worker(FakeQueue(), {})
    worker.verify(job, {})
    worker.verify(job, {})
    assert len(calls) == 1
    changed = path.stat().st_mtime_ns + 1_000_000_000
    path.write_bytes(b'other')
    os.utime(path, ns=(changed, changed))
    worker.verify(job, {})
    assert len(calls) == 2


def test_server_5xx_remains_platform_outage_when_json_overrides_code(tmp_path):
    secret = tmp_path/'token'
    secret.write_text('test-token')
    secret.chmod(0o600)
    http = Http({'platform_base_url': 'https://platform.example',
                 'credential_ref': 'file:'+str(secret), 'timeout_seconds': 1,
                 'bandwidth_bytes_per_second': 1_000_000})

    class Offline:
        def open(self, request, timeout):
            raise HTTPError(request.full_url, 503, 'unavailable', {},
                            io.BytesIO(b'{"code":"PLATFORM_OVERLOADED"}'))

    http.opener = Offline()
    try:
        http.api('POST', payload={})
    except UploadError as error:
        assert error.code == 'PLATFORM_OVERLOADED'
        assert error.retry is True and error.platform_retry is True
    else:
        raise AssertionError('expected platform outage')
