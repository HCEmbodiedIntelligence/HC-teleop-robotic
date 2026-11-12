import fcntl
import hashlib
import json
import os
import sqlite3
import time
from contextlib import contextmanager
from pathlib import Path
from uuid import uuid4

from .bundle import manifest_for
from .artifacts import ArtifactLocks, manifest_for_ready
from .common import UploadError, canonical


class Queue:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.directory.is_symlink():
            raise UploadError("QUEUE_PATH_INVALID")
        self.path = self.directory / "queue.sqlite3"
        with self.connect() as db:
            db.execute("PRAGMA journal_mode=WAL")
            db.execute("CREATE TABLE IF NOT EXISTS jobs (id TEXT PRIMARY KEY, dedup TEXT UNIQUE NOT NULL, "
                       "export_key TEXT UNIQUE NOT NULL, payload TEXT NOT NULL, state TEXT NOT NULL, "
                       "terminal INTEGER NOT NULL DEFAULT 0, next_at REAL NOT NULL DEFAULT 0)")
            db.execute("CREATE TABLE IF NOT EXISTS events (sequence INTEGER PRIMARY KEY, job_id TEXT, "
                       "at REAL, state TEXT, code TEXT)")
            db.execute("CREATE TABLE IF NOT EXISTS platform_backoff (platform TEXT PRIMARY KEY, "
                       "failures INTEGER NOT NULL, next_at REAL NOT NULL, code TEXT NOT NULL)")
        with self.connect() as db:
            db.execute("CREATE TABLE IF NOT EXISTS cancellations (job_id TEXT PRIMARY KEY)")
            db.execute("CREATE TABLE IF NOT EXISTS artifact_jobs (artifact_id TEXT PRIMARY KEY, job_id TEXT NOT NULL)")
            # Older cleanup paths could leave an artifact index after deleting
            # its job. Remove only those dangling rows so startup recovery can
            # safely enqueue the preserved descriptor again.
            db.execute("DELETE FROM artifact_jobs WHERE job_id NOT IN (SELECT id FROM jobs)")
            # Earlier HC queues predate the artifact index. Rebuild without changing jobs or bindings.
            for row in db.execute("SELECT id,payload FROM jobs"):
                job = json.loads(row['payload'])
                if job.get('artifact_id'):
                    old = db.execute("SELECT job_id FROM artifact_jobs WHERE artifact_id=?", (job['artifact_id'],)).fetchone()
                    if old and old[0] != row['id']:
                        raise UploadError('ARTIFACT_IDENTITY_CONFLICT', '旧队列存在重复产物身份；保留现场并人工核对')
                    db.execute("INSERT OR IGNORE INTO artifact_jobs VALUES(?,?)", (job['artifact_id'], row['id']))
        os.chmod(self.path, 0o600)

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=5)
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA synchronous=FULL")
        try:
            with db:
                yield db
        finally:
            db.close()

    @contextmanager
    def worker_lock(self):
        with (self.directory / "worker.lock").open("a") as stream:
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise UploadError("WORKER_ALREADY_RUNNING", "同一队列只能运行一个 worker") from None
            try:
                yield
            finally:
                fcntl.flock(stream, fcntl.LOCK_UN)

    def enqueue_artifact(self, descriptor, cfg):
        with ArtifactLocks(cfg['artifact_state_dir']).read(descriptor['artifact_id']):
            return self.enqueue(descriptor['root'], cfg, descriptor=descriptor)

    def enqueue(self, root, cfg, descriptor=None):
        client_id = str(uuid4())
        root, marker, manifest = (manifest_for_ready(descriptor, cfg, client_id) if descriptor is not None
                                  else manifest_for(root, cfg, client_id))
        frozen = {k: cfg[k] for k in ("platform_base_url", "credential_ref", "robot_id", "local_robot_id", "task_id", "part_size_bytes", "data_directory")}
        if descriptor is not None:
            frozen.update(artifact_state_dir=cfg['artifact_state_dir'], binding_id=cfg.get('binding_id'))
        # Credentials can rotate at the same reference. Never change the target of an old job.
        identity = [cfg["platform_base_url"], marker["robot_id"], marker["collection_task_id"], marker["export_id"]]
        export_key = hashlib.sha256(canonical(identity)).hexdigest()
        dedup = hashlib.sha256(canonical([identity, marker["content_sha256"], manifest["assets"]])).hexdigest()
        job = dict(id=client_id, client_upload_id=client_id, root=str(root), frozen=frozen, manifest=manifest,
                   export_id=marker["export_id"], session_id=marker["session_id"], content_sha256=marker["content_sha256"],
                   state="queued", terminal=False, next_at=0, attempts=0, upload_id=None,
                   raw_source_id=None, server_state=None, processing_status=None, quality_status=None,
                   episodes=[], code=None, next_action="等待上传", created_at=time.time(), updated_at=time.time())
        if descriptor is not None:
            job.update(artifact_id=descriptor['artifact_id'], local_recording_id=descriptor['recording_id'],
                       kind=descriptor['kind'], descriptor=descriptor)
            if descriptor.get('capture_snapshot'):
                job['capture_snapshot'] = descriptor['capture_snapshot']
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            if descriptor is not None:
                # Defend against a dangling index created after this Queue was
                # initialized (for example by an interrupted external repair).
                db.execute("DELETE FROM artifact_jobs WHERE artifact_id=? AND job_id NOT IN (SELECT id FROM jobs)",
                           (descriptor['artifact_id'],))
                bound = db.execute("SELECT j.payload FROM artifact_jobs a JOIN jobs j ON j.id=a.job_id WHERE a.artifact_id=?",
                                   (descriptor['artifact_id'],)).fetchone()
                if bound:
                    old_job = json.loads(bound[0])
                    if any(old_job['frozen'].get(k) != frozen.get(k) for k in
                           ('binding_id', 'platform_base_url', 'robot_id', 'local_robot_id', 'task_id')):
                        raise UploadError('TASK_BINDING_MISMATCH', '产物已经冻结绑定；新归属需要新的产物身份')
                    if old_job['export_id'] != marker['export_id']:
                        raise UploadError('EXPORT_ID_REUSED', '产物的原生身份已改变；需要新的产物身份')
            old = db.execute("SELECT * FROM jobs WHERE export_key=?", (export_key,)).fetchone()
            if old:
                if old["dedup"] != dedup:
                    raise UploadError("EXPORT_ID_REUSED", "内容已改变，须使用新的 export_id")
                previous = json.loads(old['payload'])
                if descriptor is not None and not previous.get('artifact_id'):
                    raise UploadError('LEGACY_JOB_REQUIRES_MIGRATION', '旧 CLI 队列没有产物读取锁；停旧 worker 后人工迁移，不能自动接管')
                if descriptor is not None and previous.get('artifact_id') != descriptor['artifact_id']:
                    raise UploadError('ARTIFACT_IDENTITY_CONFLICT', '同一原生封存不能使用两个 artifact_id')
                return previous
            db.execute("INSERT INTO jobs(id,dedup,export_key,payload,state) VALUES(?,?,?,?,?)",
                       (client_id, dedup, export_key, canonical(job).decode(), "queued"))
            if descriptor is not None:
                db.execute("INSERT INTO artifact_jobs VALUES(?,?)", (descriptor['artifact_id'], client_id))
            db.execute("INSERT INTO events(job_id,at,state) VALUES(?,?,?)", (client_id, time.time(), "queued"))
        return job

    def jobs(self, due=False):
        with self.connect() as db:
            query = "SELECT payload FROM jobs"
            args = ()
            if due:
                query += " WHERE terminal=0 AND next_at<=?"
                args = (time.time(),)
            query += " ORDER BY rowid"
            return [json.loads(r[0]) for r in db.execute(query, args)]

    def save(self, job):
        job["updated_at"] = time.time()
        with self.connect() as db:
            db.execute("UPDATE jobs SET payload=?,state=?,terminal=?,next_at=? WHERE id=?",
                       (canonical(job).decode(), job["state"], int(job["terminal"]), job["next_at"], job["id"]))
            db.execute("INSERT INTO events(job_id,at,state,code) VALUES(?,?,?,?)",
                       (job["id"], job["updated_at"], job["state"], job.get("code")))

    def platform_backoff(self, platform):
        with self.connect() as db:
            row = db.execute("SELECT failures,next_at,code FROM platform_backoff WHERE platform=?",
                             (platform,)).fetchone()
        return None if row is None else {'failures': row['failures'], 'next_at': row['next_at'],
                                         'code': row['code']}

    def platform_backoffs(self):
        with self.connect() as db:
            return {row['platform']: {'failures': row['failures'], 'next_at': row['next_at'],
                                      'code': row['code']}
                    for row in db.execute('SELECT platform,failures,next_at,code FROM platform_backoff')}

    def set_platform_backoff(self, platform, failures, next_at, code):
        with self.connect() as db:
            db.execute("INSERT INTO platform_backoff VALUES(?,?,?,?) "
                       "ON CONFLICT(platform) DO UPDATE SET failures=excluded.failures, "
                       "next_at=excluded.next_at, code=excluded.code",
                       (platform, failures, next_at, code))

    def clear_platform_backoff(self, platform):
        with self.connect() as db:
            db.execute("DELETE FROM platform_backoff WHERE platform=?", (platform,))

    def cancelled(self, job_id):
        with self.connect() as db:
            return db.execute("SELECT 1 FROM cancellations WHERE job_id=?", (job_id,)).fetchone() is not None

    def cancel(self, job_id):
        with self.connect() as db:
            if not db.execute("SELECT 1 FROM jobs WHERE id=?", (job_id,)).fetchone():
                raise UploadError('QUEUE_ITEM_NOT_FOUND')
            db.execute("INSERT OR IGNORE INTO cancellations VALUES(?)", (job_id,))
        return {'id': job_id, 'cancel_requested': True}

    def retry(self, job_id, resume=False, processing=False):
        # Serialize explicit operator commands with the worker, never overwrite a live update.
        with self.worker_lock():
            job = next((j for j in self.jobs() if j["id"] == job_id), None)
            if job is None:
                raise UploadError("QUEUE_ITEM_NOT_FOUND")
            if job.get("server_state") in {"CANCELLED", "FAILED"} and not job.get("raw_source_id"):
                raise UploadError("UPLOAD_TERMINAL", "平台取消/失效会话须人工处置；不能用 retry 创建另一份 Raw")
            if processing:
                if not job.get("raw_source_id") or job["processing_status"] not in {"PARTIALLY_FAILED", "FAILED"}:
                    raise UploadError("PROCESSING_RETRY_NOT_APPLICABLE", "只有已提交 Raw 的失败处理可以重试")
                job.setdefault("processing_retry_request", str(uuid4()))
            if not job.get("upload_id") and job.get("descriptor"):
                try:
                    from .artifacts import manifest_for_ready
                    from .common import DEFAULTS
                    cfg = {**DEFAULTS, **job.get("frozen", {})}
                    _, _, manifest = manifest_for_ready(job["descriptor"], cfg, job["id"])
                    if job['manifest'].get('capture_plan') and manifest != job['manifest']:
                        raise UploadError('CAPTURE_SNAPSHOT_MISMATCH', '重试不能更改已封存的上传合同')
                    job["manifest"] = manifest
                except Exception:
                    if job['manifest'].get('capture_plan'):
                        raise
                    pass
            with self.connect() as db:
                db.execute("DELETE FROM cancellations WHERE job_id=?", (job_id,))
            job.update(terminal=False, next_at=0, attempts=0, code=None, state="queued", resume_requested=resume)
            self.save(job)
            # An explicit operator retry is also an explicit request to test
            # the platform now instead of waiting for the shared circuit.
            self.clear_platform_backoff(job['frozen']['platform_base_url'])
            return job



def public_job(job):
    return {k: v for k, v in job.items() if k not in {"root", "frozen", "manifest", "resume_requested", "descriptor"}} | {
        "robot_id": job["frozen"]["robot_id"], "task_id": job["frozen"]["task_id"],
        "total_bytes": sum(a["size_bytes"] for a in job["manifest"]["assets"])}
