"""Pollable, cancellable dataset downloads, separate from robot control."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
from urllib.parse import quote
import uuid

from aiohttp import web

from .recording_datasets import validate_download_selection

TERMINAL = {'completed', 'cancelled', 'error'}
PUBLIC_FIELDS = {'state', 'stage', 'percent', 'files_done', 'files_total', 'bytes_done',
                 'bytes_total', 'current_file', 'archive_size', 'filename', 'error'}


@dataclass
class DownloadJob:
    id: str
    directory: Path
    filenames: tuple[str, ...]
    single: bool
    data: dict = field(default_factory=dict)
    task: asyncio.Task | None = None
    process: asyncio.subprocess.Process | None = None
    folder: Path | None = None
    path: Path | None = None
    identity: list | None = None
    content_type: str = 'application/zip'
    transport: object = None
    finished_at: float | None = None

    def snapshot(self):
        return dict(self.data)


class DownloadJobs:
    """One low-priority packer; bounded job retention and bounded-memory transfers."""
    def __init__(self, *, ttl=1800, max_active=8):
        self.jobs: dict[str, DownloadJob] = {}
        self.ttl = ttl
        self.max_active = max_active
        self.temp_root: Path | None = None
        self._packing_semaphore = asyncio.Semaphore(1)
        self._reaper: asyncio.Task | None = None
        self._closed = False

    async def _remove_artifact(self, job):
        if job.folder:
            await asyncio.to_thread(shutil.rmtree, job.folder, True)
        job.path = None

    async def _prune(self):
        now = time.monotonic()
        for job in list(self.jobs.values()):
            if job.data['state'] == 'ready' and now - job.data['_ready_at'] >= self.ttl:
                job.data.update(state='error', stage='下载已过期，请重试', error='download expired')
                job.finished_at = now
                await self._remove_artifact(job)
            if job.finished_at is not None and now - job.finished_at >= self.ttl:
                self.jobs.pop(job.id, None)
        # Keep recent terminal outcomes for refresh/retry, without unbounded growth.
        finished = sorted((j for j in self.jobs.values() if j.finished_at is not None), key=lambda j:j.finished_at)
        for job in finished[:-32]:
            self.jobs.pop(job.id, None)

    async def _reap(self):
        try:
            while not self._closed:
                await asyncio.sleep(min(30, max(.05, self.ttl)))
                await self._prune()
        except asyncio.CancelledError:
            pass

    async def create(self, directory, filenames, *, single=False):
        filenames = validate_download_selection(filenames)
        if any(len(name.encode('utf-8')) > 270 for name in filenames):
            raise ValueError('recording identifier is too long')
        if not isinstance(single, bool) or single and len(filenames) != 1:
            raise ValueError('single download requires exactly one dataset')
        if self._closed:
            raise RuntimeError('download service is stopping')
        await self._prune()
        if sum(j.data['state'] not in TERMINAL for j in self.jobs.values()) >= self.max_active:
            raise RuntimeError('too many download tasks; cancel or finish existing tasks first')
        if self.temp_root is None:
            self.temp_root = Path(tempfile.mkdtemp(prefix='hc-download-jobs-'))
        if self._reaper is None:
            self._reaper = asyncio.create_task(self._reap())
        key = uuid.uuid4().hex
        job = DownloadJob(key, Path(directory).resolve(), tuple(filenames), single)
        job.data = dict(id=key, state='queued', stage='等待打包', percent=0, selection_count=len(filenames),
                        files_done=0, files_total=0, bytes_done=0, bytes_total=0, current_file='',
                        bytes_sent=0, archive_size=0, filename='', error='', cancel_requested=False,
                        download_url=f'/api/recordings/download-jobs/{key}/download',
                        created_at=datetime.now().isoformat(timespec='seconds'))
        self.jobs[key] = job
        job.task = asyncio.create_task(self._pack(job))
        return job

    def get(self, key):
        try:
            return self.jobs[key]
        except KeyError:
            raise web.HTTPNotFound(text='download task not found') from None

    @staticmethod
    def public(job):
        return {k:v for k,v in job.data.items() if not k.startswith('_')}

    async def _stop_process(self, job):
        process = job.process
        if process is not None and process.returncode is None:
            try:
                process.terminate()
            except ProcessLookupError:
                return
            try:
                await asyncio.wait_for(process.wait(), timeout=2)
            except asyncio.TimeoutError:
                process.kill()
                await process.wait()

    async def _pack(self, job):
        stderr_task = None
        try:
            async with self._packing_semaphore:
                if job.data['cancel_requested']:
                    return
                job.data.update(state='scanning', stage='正在读取文件列表')
                job.folder = self.temp_root / job.id
                job.folder.mkdir()
                repo = Path(__file__).resolve().parents[3]
                env = dict(os.environ)
                env['PYTHONPATH'] = str(repo) + os.pathsep + env.get('PYTHONPATH', '')
                env['PYTHONUNBUFFERED'] = '1'
                job.process = await asyncio.create_subprocess_exec(
                    sys.executable, '-m', 'middleware.core.recording.recording_download_worker',
                    stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE, env=env, cwd=repo)
                stderr_task = asyncio.create_task(job.process.stderr.read())
                job.process.stdin.write(json.dumps(dict(directory=str(job.directory), filenames=job.filenames,
                                                       output=str(job.folder/'archive.zip'), single=job.single)).encode()+b'\n')
                await job.process.stdin.drain()
                job.process.stdin.close()
                if job.data['cancel_requested']:
                    await self._stop_process(job)
                ready = None
                async for line in job.process.stdout:
                    message = json.loads(line)
                    if job.data['cancel_requested']:
                        continue
                    if message.get('state') == 'ready':
                        ready = message
                    else:
                        job.data.update({k:v for k,v in message.items() if k in PUBLIC_FIELDS})
                code = await job.process.wait()
                stderr = await stderr_task
                if job.data['cancel_requested']:
                    job.data.update(state='cancelled', stage='已取消')
                elif code or ready is None:
                    job.data.update(state='error', stage='打包失败', error=job.data.get('error') or
                                    stderr.decode(errors='replace')[-2000:] or 'archive worker stopped unexpectedly')
                else:
                    job.path = Path(ready['path'])
                    job.identity = ready['identity']
                    job.content_type = ready['content_type']
                    job.data.update({k:v for k,v in ready.items() if k in PUBLIC_FIELDS})
                    job.data['_ready_at'] = time.monotonic()
        except asyncio.CancelledError:
            job.data.update(cancel_requested=True, state='cancelled', stage='已取消')
            await self._stop_process(job)
        except Exception as exception:
            job.data.update(state='error', stage='打包失败', error=str(exception)[:2000])
            await self._stop_process(job)
        finally:
            if stderr_task is not None and not stderr_task.done():
                await stderr_task
            if job.data['cancel_requested']:
                job.data.update(state='cancelled', stage='已取消')
            if job.data['state'] in TERMINAL:
                await self._remove_artifact(job)
                job.finished_at = time.monotonic()

    async def cancel(self, job):
        state = job.data['state']
        if state in TERMINAL or job.data['cancel_requested']:
            return
        job.data.update(cancel_requested=True, state='cancelling', stage='正在取消')
        if state == 'queued':
            job.task.cancel()  # No subprocess is being created while state is queued.
            await asyncio.gather(job.task, return_exceptions=True)
            job.data.update(state='cancelled', stage='已取消')
            job.finished_at = time.monotonic()
        elif state == 'ready':
            await self._remove_artifact(job)
            job.data.update(state='cancelled', stage='已取消')
            job.finished_at = time.monotonic()
        elif state == 'downloading':
            if job.transport is not None:
                job.transport.close()  # Interrupt this transfer even under TCP backpressure.
        else:
            await self._stop_process(job)
            # _pack completes cleanup and the cancelled state; do not claim done early.
            await asyncio.gather(job.task, return_exceptions=True)

    async def transfer(self, job, request):
        if job.data['state'] != 'ready' or job.path is None:
            raise web.HTTPConflict(text='download is not ready; retry the task if it has finished')
        fd = None
        try:
            fd = os.open(job.path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
            info = os.fstat(fd)
            if [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns] != job.identity:
                raise ValueError('download source changed; retry after recording has stopped')
        except (OSError, ValueError) as exception:
            if fd is not None:
                os.close(fd)
            job.data.update(state='error', stage='下载失败', error=str(exception))
            job.finished_at = time.monotonic()
            await self._remove_artifact(job)
            raise web.HTTPConflict(text=str(exception)) from exception
        job.data.update(state='downloading', stage='正在传输到浏览器', bytes_sent=0, percent=0)
        job.transport = request.transport
        response = web.StreamResponse(headers={
            'Content-Disposition': "attachment; filename*=UTF-8''"+quote(job.data['filename']),
            'Content-Type': job.content_type, 'Content-Length': str(info.st_size),
            'Cache-Control': 'no-store',
        })
        try:
            await response.prepare(request)
            with os.fdopen(fd, 'rb') as source:
                fd = None
                remaining = info.st_size
                while remaining:
                    chunk = await asyncio.to_thread(source.read, min(256*1024, remaining))
                    if not chunk:
                        raise ValueError('download source was truncated during transfer')
                    if job.data['cancel_requested']:
                        response.force_close()
                        break
                    await response.write(chunk)
                    remaining -= len(chunk)
                    job.data['bytes_sent'] += len(chunk)
                    job.data['percent'] = min(99.9, job.data['bytes_sent'] * 100 / info.st_size) if info.st_size else 100
                end = os.fstat(source.fileno())
                if not job.data['cancel_requested'] and [end.st_dev, end.st_ino, end.st_size, end.st_mtime_ns] != job.identity:
                    raise ValueError('download source changed during transfer')
            if not job.data['cancel_requested']:
                await response.write_eof()
                job.data.update(state='completed', stage='服务器传输完成，请查看浏览器下载列表', percent=100)
        except (ConnectionError, asyncio.CancelledError):
            job.data.update(cancel_requested=True)
        except Exception as exception:
            job.data.update(state='error', stage='下载失败', error=str(exception)[:2000])
            if job.transport is not None:
                job.transport.close()
        finally:
            if fd is not None:
                os.close(fd)
            job.transport = None
            if job.data['cancel_requested']:
                job.data.update(state='cancelled', stage='已取消')
            job.finished_at = time.monotonic()
            await self._remove_artifact(job)
        return response

    async def close(self):
        if self._closed:
            return
        self._closed = True
        if self._reaper:
            self._reaper.cancel()
            await asyncio.gather(self._reaper, return_exceptions=True)
        await asyncio.gather(*(self.cancel(job) for job in list(self.jobs.values())), return_exceptions=True)
        await asyncio.gather(*(j.task for j in self.jobs.values() if j.task is not None), return_exceptions=True)
        if self.temp_root:
            await asyncio.to_thread(shutil.rmtree, self.temp_root, True)


DOWNLOAD_JOBS = web.AppKey('recording_download_jobs', DownloadJobs)


def register_download_routes(app, get_recorder):
    manager = DownloadJobs()
    app[DOWNLOAD_JOBS] = manager

    def check_recording(directory, filenames):
        recorder = get_recorder()
        if recorder is None:
            raise web.HTTPServiceUnavailable(text='recorder not available')
        if (recorder.is_recording() and recorder.path is not None
                and recorder.path.parent.resolve() == Path(directory).resolve()
                and recorder.path.name in filenames):
            raise web.HTTPConflict(text='recording is still being written')
        return recorder

    async def create(request):
        try:
            data = await request.json()
            if not isinstance(data, dict):
                raise ValueError('invalid download request')
            filenames = validate_download_selection(data.get('filenames'))
            recorder = get_recorder()
            if recorder is None:
                raise web.HTTPServiceUnavailable(text='recorder not available')
            check_recording(recorder.directory, filenames)
            job = await manager.create(recorder.directory, filenames, single=data.get('single', False))
            return web.json_response(dict(ok=True, job=manager.public(job)), status=202)
        except (ValueError, TypeError) as exception:
            raise web.HTTPBadRequest(text=str(exception)) from exception
        except RuntimeError as exception:
            raise web.HTTPTooManyRequests(text=str(exception)) from exception

    async def listing(_request):
        await manager._prune()
        return web.json_response(dict(ok=True, jobs=[manager.public(job) for job in manager.jobs.values()]))

    async def status(request):
        await manager._prune()
        return web.json_response(dict(ok=True, job=manager.public(manager.get(request.match_info['job_id']))))

    async def cancel(request):
        job = manager.get(request.match_info['job_id'])
        await manager.cancel(job)
        return web.json_response(dict(ok=True, job=manager.public(job)))

    async def retry(request):
        previous = manager.get(request.match_info['job_id'])
        if previous.data['state'] not in TERMINAL:
            raise web.HTTPConflict(text='cancel or finish the current download before retrying')
        check_recording(previous.directory, previous.filenames)
        try:
            job = await manager.create(previous.directory, list(previous.filenames), single=previous.single)
        except RuntimeError as exception:
            raise web.HTTPTooManyRequests(text=str(exception)) from exception
        return web.json_response(dict(ok=True, job=manager.public(job)), status=202)

    async def download(request):
        return await manager.transfer(manager.get(request.match_info['job_id']), request)

    async def close(_app):
        await manager.close()

    prefix = '/api/recordings/download-jobs'
    app.router.add_post(prefix, create)
    app.router.add_get(prefix, listing)
    app.router.add_get(prefix+'/{job_id}', status)
    app.router.add_post(prefix+'/{job_id}/cancel', cancel)
    app.router.add_post(prefix+'/{job_id}/retry', retry)
    app.router.add_get(prefix+'/{job_id}/download', download, allow_head=False)
    app.on_shutdown.append(close)
    app.on_cleanup.append(close)
