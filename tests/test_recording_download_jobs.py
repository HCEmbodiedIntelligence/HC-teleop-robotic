"""Local API coverage for asynchronous recording download preparation."""
import asyncio
import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path

from aiohttp.test_utils import TestClient, TestServer

from middleware.core.config import ConfigStore
from middleware.core.recording_datasets import MAX_BATCH_DOWNLOAD_SELECTION


class RecordingDownloadJobApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        from middleware.core.app import create_app

        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.recordings = self.root / 'recordings'
        self.recordings.mkdir()
        store = ConfigStore(self.root / 'middleware.yaml')
        config = store.load()
        config['robot_profiles'] = {'root': str(self.root / 'profiles'), 'active': ''}
        for section in ('ros', 'vr', 'camera', 'safety'):
            config[section]['enabled'] = False
        config['safety']['stop_on_startup'] = False
        config['ros']['recording']['directory'] = str(self.recordings)
        store.save(config)
        self.app = create_app(store)
        self.client = TestClient(TestServer(self.app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.directory.cleanup()

    def manager(self):
        from middleware.core.recording_downloads import DOWNLOAD_JOBS
        return self.app[DOWNLOAD_JOBS]

    def make_dataset(self):
        dataset = self.recordings / 'lerobot' / 'episode'
        (dataset / 'meta').mkdir(parents=True)
        (dataset / 'meta/info.json').write_text(json.dumps({
            'total_frames': 1, 'total_episodes': 1, 'fps': 15,
            'robot_type': 'test', 'features': {},
        }))
        (dataset / 'data').mkdir()
        (dataset / 'data/file.parquet').write_bytes(b'parquet payload')
        return dataset

    async def create_job(self, filenames, **options):
        response = await self.client.post('/api/recordings/download-jobs', json={
            'filenames': filenames, **options,
        })
        self.assertEqual(response.status, 202, await response.text())
        result = await response.json()
        self.assertTrue(result['ok'])
        job = result['job']
        self.assertTrue(job['id'])
        self.assertEqual(job['selection_count'], len(filenames))
        return job

    async def status(self, job_id):
        response = await self.client.get('/api/recordings/download-jobs/' + job_id)
        self.assertEqual(response.status, 200, await response.text())
        result = await response.json()
        self.assertTrue(result['ok'])
        self.assertEqual(result['job']['id'], job_id)
        return result['job']

    async def wait_for(self, job_id, state):
        deadline = asyncio.get_running_loop().time() + 5
        while True:
            job = await self.status(job_id)
            if job['state'] == state:
                return job
            if job['state'] in {'error', 'cancelled', 'completed'}:
                self.fail(f'expected {state}, received {job}')
            if asyncio.get_running_loop().time() >= deadline:
                self.fail(f'timed out waiting for {state}: {job}')
            await asyncio.sleep(0.02)

    async def wait_removed(self, path):
        deadline = asyncio.get_running_loop().time() + 5
        while path.exists() and asyncio.get_running_loop().time() < deadline:
            await asyncio.sleep(0.02)
        self.assertFalse(path.exists(), f'temporary download artifact leaked: {path}')

    def assert_ready_progress(self, job, sources):
        self.assertEqual(job['files_total'], len(sources))
        self.assertEqual(job['files_done'], len(sources))
        self.assertEqual(job['bytes_total'], sum(path.stat().st_size for path in sources))
        self.assertEqual(job['bytes_done'], job['bytes_total'])
        self.assertEqual(job['bytes_sent'], 0)
        self.assertGreater(job['archive_size'], 0)
        self.assertTrue(job['filename'])
        self.assertTrue(job['download_url'].endswith('/' + job['id'] + '/download'))
        self.assertGreaterEqual(job['percent'], 0)
        self.assertLessEqual(job['percent'], 100)

    async def test_mixed_archive_progress_listing_stream_completion_and_retry(self):
        raw = self.recordings / 'capture.mcap'
        raw.write_bytes(b'mcap payload')
        dataset = self.make_dataset()
        bundle = self.recordings / 'bundles' / 'session'
        bundle.mkdir(parents=True)
        (bundle / 'session.json').write_text('{}')
        (bundle / 'raw.mcap').write_bytes(b'bundle payload')
        sources = [raw, *(path for path in dataset.rglob('*') if path.is_file()),
                   bundle / 'session.json', bundle / 'raw.mcap']
        job = await self.create_job(['capture.mcap', 'lerobot:episode', 'bundle:session'])
        ready = await self.wait_for(job['id'], 'ready')
        self.assert_ready_progress(ready, sources)
        listing = await (await self.client.get('/api/recordings/download-jobs')).json()
        self.assertTrue(listing['ok'])
        self.assertIn(job['id'], [item['id'] for item in listing['jobs']])
        folder = self.manager().jobs[job['id']].folder
        self.assertTrue(folder.exists())

        response = await self.client.get(ready['download_url'])
        self.assertEqual(response.status, 200)
        self.assertEqual(response.headers['Content-Type'], 'application/zip')
        data = await response.read()
        self.assertEqual(len(data), ready['archive_size'])
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(archive.read('mcap/capture.mcap'), b'mcap payload')
            self.assertEqual(archive.read('lerobot/episode/data/file.parquet'), b'parquet payload')
            self.assertEqual(archive.read('capture_bundle/session/raw.mcap'), b'bundle payload')
        completed = await self.wait_for(job['id'], 'completed')
        self.assertEqual(completed['bytes_sent'], len(data))
        self.assertEqual(completed['percent'], 100)
        await self.wait_removed(folder)
        self.assertTrue(raw.exists())

        response = await self.client.post('/api/recordings/download-jobs/' + job['id'] + '/retry')
        self.assertEqual(response.status, 202, await response.text())
        retried = (await response.json())['job']
        self.assertNotEqual(retried['id'], job['id'])
        self.assertEqual(retried['selection_count'], 3)
        self.assert_ready_progress(await self.wait_for(retried['id'], 'ready'), sources)

    async def test_single_plain_mcap_download_is_raw(self):
        raw = self.recordings / 'capture.mcap'
        raw.write_bytes(b'raw mcap payload')
        job = await self.create_job(['capture.mcap'], single=True)
        ready = await self.wait_for(job['id'], 'ready')
        self.assert_ready_progress(ready, [raw])
        self.assertEqual(ready['filename'], 'capture.mcap')
        response = await self.client.get(ready['download_url'])
        self.assertEqual(response.status, 200)
        self.assertEqual(await response.read(), raw.read_bytes())
        await self.wait_for(job['id'], 'completed')

    async def test_single_tagged_mcap_archive_preserves_companion(self):
        raw = self.recordings / 'capture.mcap'
        raw.write_bytes(b'tagged mcap payload')
        tags = raw.with_suffix('.takeovers.json')
        tags.write_text('{"intervals":[]}')
        job = await self.create_job(['capture.mcap'], single=True)
        ready = await self.wait_for(job['id'], 'ready')
        self.assert_ready_progress(ready, [raw, tags])
        response = await self.client.get(ready['download_url'])
        self.assertEqual(response.status, 200)
        with zipfile.ZipFile(io.BytesIO(await response.read())) as archive:
            self.assertEqual(archive.read('mcap/capture.mcap'), raw.read_bytes())
            self.assertEqual(json.loads(archive.read('mcap/capture.takeovers.json')), {'intervals': []})

    async def test_single_lerobot_archive_retains_dataset_root(self):
        dataset = self.make_dataset()
        job = await self.create_job(['lerobot:episode'], single=True)
        ready = await self.wait_for(job['id'], 'ready')
        self.assert_ready_progress(ready, [path for path in dataset.rglob('*') if path.is_file()])
        response = await self.client.get(ready['download_url'])
        self.assertEqual(response.status, 200)
        with zipfile.ZipFile(io.BytesIO(await response.read())) as archive:
            self.assertEqual(set(archive.namelist()), {'episode/meta/info.json', 'episode/data/file.parquet'})
            self.assertEqual(archive.read('episode/data/file.parquet'), b'parquet payload')

    async def test_cancel_queued_job_and_ready_archive(self):
        raw = self.recordings / 'capture.mcap'
        raw.write_bytes(b'mcap payload')
        manager = self.manager()
        await manager._packing_semaphore.acquire()
        try:
            job = await self.create_job(['capture.mcap'])
            self.assertIn((await self.status(job['id']))['state'], {'queued', 'scanning'})
            response = await self.client.post('/api/recordings/download-jobs/' + job['id'] + '/cancel')
            self.assertEqual(response.status, 200, await response.text())
            cancelled = await self.wait_for(job['id'], 'cancelled')
            self.assertTrue(cancelled['cancel_requested'])
            self.assertIsNone(manager.jobs[job['id']].process)
            self.assertIsNone(manager.jobs[job['id']].folder)
        finally:
            manager._packing_semaphore.release()
        job = await self.create_job(['capture.mcap'])
        ready = await self.wait_for(job['id'], 'ready')
        folder = manager.jobs[job['id']].folder
        response = await self.client.post('/api/recordings/download-jobs/' + job['id'] + '/cancel')
        self.assertEqual(response.status, 200, await response.text())
        await self.wait_for(job['id'], 'cancelled')
        self.assertFalse(folder.exists())
        response = await self.client.get(ready['download_url'])
        self.assertGreaterEqual(response.status, 400)
        await response.read()
        self.assertEqual(raw.read_bytes(), b'mcap payload')

    async def test_request_validation_and_active_recording_conflict(self):
        invalid = [None, [], 'capture.mcap', [1], ['capture.mcap', 'capture.mcap'],
                   [f'capture_{i}.mcap' for i in range(MAX_BATCH_DOWNLOAD_SELECTION + 1)]]
        for filenames in invalid:
            with self.subTest(filenames=str(filenames)[:80]):
                response = await self.client.post('/api/recordings/download-jobs', json={'filenames': filenames})
                self.assertEqual(response.status, 400, await response.text())
        response = await self.client.post('/api/recording/start', json={
            'filename': 'active.mcap', 'force': True, 'selection': {'mode': 'local'},
        })
        self.assertEqual(response.status, 200, await response.text())
        try:
            response = await self.client.post('/api/recordings/download-jobs', json={'filenames': ['active.mcap']})
            self.assertEqual(response.status, 409, await response.text())
        finally:
            await self.client.post('/api/recording/stop')

    async def test_traversal_and_symlink_sources_fail_without_download(self):
        outside = self.root / 'outside.mcap'
        outside.write_bytes(b'private outside data')
        (self.recordings / 'alias.mcap').symlink_to(outside)
        dataset = self.make_dataset()
        (dataset / 'data/leak.mcap').symlink_to(outside)
        for identifier in ('../outside.mcap', 'alias.mcap', 'lerobot:episode'):
            with self.subTest(identifier=identifier):
                job = await self.create_job([identifier])
                error = await self.wait_for(job['id'], 'error')
                self.assertTrue(error['error'])
                self.assertEqual(error['bytes_sent'], 0)
                folder = self.manager().jobs[job['id']].folder
                self.assertIsNotNone(folder)
                await self.wait_removed(folder)
                response = await self.client.get('/api/recordings/download-jobs/' + job['id'] + '/download')
                self.assertGreaterEqual(response.status, 400)
                self.assertNotIn(outside.read_bytes(), await response.read())

    async def test_server_shutdown_removes_prepared_archives(self):
        raw = self.recordings / 'capture.mcap'
        raw.write_bytes(b'mcap payload')
        job = await self.create_job(['capture.mcap'])
        await self.wait_for(job['id'], 'ready')
        manager = self.manager()
        folder = manager.jobs[job['id']].folder
        temp_root = manager.temp_root
        self.assertTrue(folder.exists())
        await self.client.close()
        self.assertFalse(folder.exists())
        self.assertFalse(temp_root.exists())
        self.assertEqual(raw.read_bytes(), b'mcap payload')


    async def test_cancel_running_packer_terminates_process_and_cleans_partial_zip(self):
        from unittest.mock import patch
        import sys
        original = asyncio.create_subprocess_exec
        script = ("import sys,json,time; from pathlib import Path; "
                  "c=json.loads(sys.stdin.readline()); Path(c['output']).write_bytes(b'partial'); "
                  "print(json.dumps(dict(state='packing',stage='正在打包',bytes_done=7,bytes_total=100)),flush=True); "
                  "time.sleep(30)")
        async def slow_worker(*args, **kwargs):
            return await original(sys.executable, '-c', script, **kwargs)
        raw = self.recordings / 'capture.mcap'
        raw.write_bytes(b'original source')
        with patch('middleware.core.recording_downloads.asyncio.create_subprocess_exec', slow_worker):
            job = await self.create_job(['capture.mcap'])
            await self.wait_for(job['id'], 'packing')
            actual = self.manager().jobs[job['id']]
            process, folder = actual.process, actual.folder
            self.assertIsNotNone(process)
            response = await self.client.post('/api/recordings/download-jobs/'+job['id']+'/cancel')
            self.assertEqual(response.status, 200)
            self.assertEqual((await response.json())['job']['state'], 'cancelled')
            self.assertIsNotNone(process.returncode)
            self.assertFalse(folder.exists())
            self.assertEqual(raw.read_bytes(), b'original source')

    async def test_cancel_active_transfer_closes_only_download_and_keeps_source(self):
        import aiohttp
        raw = self.recordings / 'large.mcap'
        with raw.open('wb') as stream:
            stream.truncate(16*1024*1024)
        job = await self.create_job(['large.mcap'], single=True)
        await self.wait_for(job['id'], 'ready')
        response = await self.client.get(job['download_url'])
        self.assertEqual(response.status, 200)
        await response.content.read(1024)
        cancelled = await self.client.post('/api/recordings/download-jobs/'+job['id']+'/cancel')
        self.assertEqual(cancelled.status, 200)
        try:
            await response.read()
        except (aiohttp.ClientPayloadError, aiohttp.ClientConnectionError):
            pass
        finally:
            response.close()
        await self.wait_for(job['id'], 'cancelled')
        state = await self.status(job['id'])
        self.assertLess(state['bytes_sent'], state['archive_size'])
        self.assertEqual(raw.stat().st_size,16*1024*1024)
        self.assertEqual((await self.client.get('/health')).status, 200)

    async def test_expired_ready_archive_is_removed_and_retry_recreates_task(self):
        import time
        raw = self.recordings / 'capture.mcap'
        raw.write_bytes(b'payload')
        job = await self.create_job(['capture.mcap'])
        await self.wait_for(job['id'], 'ready')
        actual = self.manager().jobs[job['id']]
        folder = actual.folder
        actual.data['_ready_at'] = time.monotonic()-self.manager().ttl-1
        await self.manager()._prune()
        self.assertFalse(folder.exists())
        self.assertEqual((await self.status(job['id']))['state'], 'error')
        response=await self.client.post('/api/recordings/download-jobs/'+job['id']+'/retry')
        self.assertEqual(response.status,202)
        retry=(await response.json())['job']
        self.assertNotEqual(retry['id'],job['id'])
        await self.wait_for(retry['id'],'ready')
        self.assertEqual(raw.read_bytes(),b'payload')


if __name__ == '__main__':
    unittest.main()
