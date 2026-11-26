import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path
from urllib.parse import quote

from aiohttp.test_utils import TestClient, TestServer
from middleware.core.config import ConfigStore
from middleware.core.recording.topic_recorder import TopicRecorder
from middleware.core.recording.recording_datasets import (
    archive_datasets, resolve_dataset, validate_download_selection, MAX_BATCH_DOWNLOAD_SELECTION,
)


def make_dataset(root, name='episode'):
    dataset = root / 'lerobot' / name
    (dataset / 'meta').mkdir(parents=True)
    (dataset / 'meta/info.json').write_text(json.dumps({
        'total_frames': 3, 'total_episodes': 2, 'fps': 15, 'robot_type': 'x1',
        'features': {'observation.state': {'dtype': 'float32', 'shape': [21]},
                     'observation.images.eye': {'dtype': 'video', 'shape': [400, 640, 3]}}}))
    (dataset / 'meta/teleop_sync.json').write_text(json.dumps({'task': '搬箱子'}))
    video = dataset / 'videos/observation.images.eye/chunk-000/file-000.mp4'
    video.parent.mkdir(parents=True)
    video.write_bytes(b'video payload')
    (dataset / 'data').mkdir()
    (dataset / 'data/file.parquet').write_bytes(b'parquet')
    return dataset, video


class DatasetFilesTests(unittest.TestCase):
    def test_archive_mixed_dataset_selection(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            make_dataset(root)
            (root / 'capture.mcap').write_bytes(b'mcap payload')
            archive = archive_datasets(root, ['capture.mcap', 'lerobot:episode'])
            try:
                with zipfile.ZipFile(archive) as output:
                    self.assertEqual(output.read('mcap/capture.mcap'), b'mcap payload')
                    self.assertEqual(
                        output.read('lerobot/episode/videos/observation.images.eye/chunk-000/file-000.mp4'),
                        b'video payload')
            finally:
                archive.unlink(missing_ok=True)

    def test_archive_whole_collection_above_old_200_limit(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            names = [f'capture_{index:04d}.mcap' for index in range(1508)]
            for index, name in enumerate(names):
                (root / name).write_bytes(f'original {index}'.encode())
            archive = archive_datasets(root, names)
            try:
                with zipfile.ZipFile(archive) as output:
                    self.assertEqual(output.namelist(), ['mcap/' + name for name in names])
                    self.assertIsNone(output.testzip())
                    self.assertEqual(output.read('mcap/' + names[-1]), b'original 1507')
            finally:
                archive.unlink(missing_ok=True)

    def test_batch_download_limits_and_invalid_selections_are_explicit(self):
        names = [f'capture_{index}.mcap' for index in range(MAX_BATCH_DOWNLOAD_SELECTION)]
        self.assertEqual(validate_download_selection(names), names)
        with self.assertRaisesRegex(ValueError, 'at most 10000 datasets'):
            validate_download_selection(names + ['one_more.mcap'])
        for selection in (None, 'capture.mcap', [], ['capture.mcap', 'capture.mcap'],
                          [1], [{'file': 'capture.mcap'}]):
            with self.subTest(selection=selection), self.assertRaises(ValueError):
                validate_download_selection(selection)

    def test_list_and_delete_dataset_keeps_source_and_other_formats(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            dataset, _ = make_dataset(root)
            source = root / '.lerobot_sources/episode.mcap'
            source.parent.mkdir()
            source.write_bytes(b'original')
            (root / 'episode.jsonl').write_text('{}')
            recorder = TopicRecorder({'directory': str(root)}, root)
            listed = recorder.list_recordings()
            self.assertEqual(len(listed), 2)
            entry = next(item for item in listed if item['format'] == 'LeRobot')
            self.assertEqual((entry['frames'], entry['total_episodes'], entry['camera_count']), (3, 2, 1))
            self.assertEqual(entry['task'], '搬箱子')
            self.assertEqual(entry['size_bytes'], sum(p.stat().st_size for p in dataset.rglob('*') if p.is_file()))
            recorder.delete_recording(entry['id'])
            self.assertFalse(dataset.exists())
            self.assertTrue(source.exists())
            self.assertTrue((root / 'episode.jsonl').exists())

    def test_invalid_and_symlink_paths_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            dataset, _ = make_dataset(root)
            for identifier in ('../episode', 'lerobot:../episode', 'lerobot:', 'lerobot:.hidden'):
                with self.assertRaises(ValueError):
                    resolve_dataset(root, identifier)
            (root / 'lerobot/alias').symlink_to(dataset, target_is_directory=True)
            with self.assertRaises(ValueError):
                resolve_dataset(root, 'lerobot:alias')


class DatasetApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        from middleware.core.app import create_app
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        store = ConfigStore(self.root / 'middleware.yaml')
        config = store.load()
        config['robot_profiles'] = {'root': str(self.root / 'profiles'), 'active': ''}
        for section in ('ros', 'vr', 'camera', 'safety'):
            config[section]['enabled'] = False
        config['safety']['stop_on_startup'] = False
        config['ros']['recording']['directory'] = str(self.root / 'recordings')
        store.save(config)
        self.dataset, self.video = make_dataset(self.root / 'recordings')
        self.client = TestClient(TestServer(create_app(store)))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.directory.cleanup()

    async def test_tagged_mcap_download_includes_companion_without_changing_raw_api(self):
        source = self.root / 'recordings/capture.mcap'
        source.write_bytes(b'raw mcap')
        url = '/api/recordings/capture.mcap/download'
        self.assertEqual(await (await self.client.get(url + '?with_tags=1')).read(), b'raw mcap')
        source.with_suffix('.takeovers.json').write_text('{"tag":"dagger","intervals":[]}')
        self.assertEqual(await (await self.client.get(url)).read(), b'raw mcap')
        data = await (await self.client.get(url + '?with_tags=1')).read()
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            self.assertEqual(archive.read('mcap/capture.mcap'), b'raw mcap')
            self.assertEqual(json.loads(archive.read('mcap/capture.takeovers.json'))['intervals'], [])

    async def test_format_selection_saves_through_page_api_and_survives_reload(self):
        for mode in ('mcap', 'mcap_mp4', 'lerobot', 'both'):
            response = await self.client.get('/api/config')
            config = await response.json()
            config['ros']['recording']['format'] = mode
            saved = await self.client.put('/api/config', json=config)
            self.assertEqual(saved.status, 200, await saved.text())
            reloaded = await (await self.client.get('/api/config')).json()
            self.assertEqual(reloaded['ros']['recording']['format'], mode)
            self.assertEqual(ConfigStore(self.root/'middleware.yaml').load()['ros']['recording']['format'], mode)

    async def test_saving_format_during_capture_keeps_the_active_configuration(self):
        from unittest.mock import patch
        config = await (await self.client.get('/api/config')).json()
        original = config['ros']['recording']['format']
        config['ros']['recording']['format'] = 'mcap_mp4'
        with patch.object(TopicRecorder, 'is_recording', return_value=True):
            response = await self.client.put('/api/config', json=config)
            self.assertEqual(response.status, 409)
            self.assertIn('请先停止录制', await response.text())
        self.assertEqual(ConfigStore(self.root/'middleware.yaml').load()['ros']['recording']['format'], original)

    async def test_list_zip_video_range_and_batch_delete(self):
        response = await self.client.get('/api/recordings')
        data = await response.json()
        self.assertEqual(data['total_count'], 1)
        item = data['files'][0]
        self.assertEqual(item['format'], 'LeRobot')
        url = '/api/recordings/' + quote(item['id'], safe='')
        response = await self.client.get(url + '/download')
        self.assertEqual(response.status, 200)
        self.assertEqual(response.headers['Content-Type'], 'application/zip')
        with zipfile.ZipFile(io.BytesIO(await response.read())) as archive:
            self.assertEqual(archive.read('episode/videos/observation.images.eye/chunk-000/file-000.mp4'), b'video payload')
            self.assertIn('episode/meta/info.json', archive.namelist())
        response = await self.client.get(item['videos'][0]['url'], headers={'Range': 'bytes=0-4'})
        self.assertEqual(response.status, 206)
        self.assertEqual(await response.read(), b'video')
        response = await self.client.get(url + '/video?path=../../middleware.yaml')
        self.assertEqual(response.status, 404)
        response = await self.client.post('/api/recordings/batch-delete', json={'filenames': [item['id']]})
        result = await response.json()
        self.assertEqual(result['deleted'], [item['id']])
        self.assertFalse(self.dataset.exists())

    async def test_batch_download_returns_one_mixed_zip(self):
        raw = self.root / 'recordings/capture.mcap'
        raw.write_bytes(b'mcap payload')
        response = await self.client.post('/api/recordings/batch-download', json={
            'filenames': ['capture.mcap', 'lerobot:episode'],
        })
        self.assertEqual(response.status, 200)
        self.assertTrue(response.headers['Content-Disposition'].endswith('.zip"'))
        with zipfile.ZipFile(io.BytesIO(await response.read())) as archive:
            self.assertEqual(archive.read('mcap/capture.mcap'), b'mcap payload')
            self.assertIn('lerobot/episode/meta/info.json', archive.namelist())

        response = await self.client.post('/api/recordings/batch-download', data={
            'filenames': json.dumps(['capture.mcap']),
        })
        self.assertEqual(response.status, 200)
        with zipfile.ZipFile(io.BytesIO(await response.read())) as archive:
            self.assertEqual(archive.read('mcap/capture.mcap'), b'mcap payload')

    async def test_batch_download_over_200_works_for_json_and_form(self):
        names = [f'capture_{index:03d}.mcap' for index in range(201)]
        for index, name in enumerate(names):
            (self.root / 'recordings' / name).write_bytes(f'payload {index}'.encode())
        for body in ({'json': {'filenames': names}},
                     {'data': {'filenames': json.dumps(names)}}):
            with self.subTest(encoding=next(iter(body))):
                response = await self.client.post('/api/recordings/batch-download', **body)
                self.assertEqual(response.status, 200, await response.text() if response.status != 200 else '')
                with zipfile.ZipFile(io.BytesIO(await response.read())) as archive:
                    self.assertEqual(len(archive.namelist()), 201)
                    self.assertIsNone(archive.testzip())
                    self.assertEqual(archive.read('mcap/' + names[-1]), b'payload 200')

    async def test_batch_download_reports_new_bound_without_obscure_selection_error(self):
        response = await self.client.post('/api/recordings/batch-download', json={
            'filenames': [f'capture_{index}.mcap' for index in range(MAX_BATCH_DOWNLOAD_SELECTION + 1)]})
        self.assertEqual(response.status, 400)
        self.assertIn('at most 10000 datasets', await response.text())

    async def test_batch_download_rejects_file_still_being_recorded(self):
        response = await self.client.post('/api/recording/start', json={
            'filename': 'active.mcap', 'force': True, 'selection': {'mode': 'local'}})
        self.assertEqual(response.status, 200, await response.text())
        try:
            response = await self.client.post('/api/recordings/batch-download', json={
                'filenames': ['active.mcap']})
            self.assertEqual(response.status, 409)
            self.assertIn('still being written', await response.text())
        finally:
            await self.client.post('/api/recording/stop')

    async def test_recording_list_filters_formats_and_compacts_rows(self):
        raw = self.root / 'recordings/capture.mcap'
        raw.write_bytes(b'mcap payload')
        response = await self.client.get('/api/recordings?format=mcap&compact=1')
        self.assertEqual(response.status, 200)
        result = await response.json()
        self.assertEqual([item['id'] for item in result['files']], ['capture.mcap'])
        self.assertNotIn('channels', result['files'][0])
        self.assertEqual(result['format_counts'], {'mcap': 1, 'lerobot': 1, 'bundle': 0})

        result = await (await self.client.get('/api/recordings?format=lerobot')).json()
        self.assertEqual([item['id'] for item in result['files']], ['lerobot:episode'])
        self.assertIn('channels', result['files'][0])

    async def test_patch_recording_updates_metadata_and_renames_mcap_with_companions(self):
        raw = self.root / 'recordings/capture.mcap'
        raw.write_bytes(b'mcap payload')
        raw.with_suffix('.takeovers.json').write_text(json.dumps({'tag': 'dagger', 'recording': 'capture.mcap', 'intervals': []}))

        # Update metadata without renaming.
        response = await self.client.patch('/api/recordings/capture.mcap', json={
            'task': '双臂搬运箱子',
            'robot_label': 'X1-一号机',
            'tags': ['高质量', '双臂', '高质量'],
            'notes': '动作流畅',
            'quality': 'good',
            'marked': True,
        })
        self.assertEqual(response.status, 200, await response.text())
        body = await response.json()
        self.assertFalse(body['renamed'])
        self.assertEqual(body['id'], 'capture.mcap')
        self.assertEqual(body['item']['task'], '双臂搬运箱子')
        self.assertEqual(body['item']['profile_display_name'], 'X1-一号机')
        self.assertEqual(body['item']['tags'], ['高质量', '双臂'])
        self.assertEqual(body['item']['notes'], '动作流畅')
        self.assertEqual(body['item']['quality'], 'good')
        self.assertTrue(body['item']['marked'])

        # Rename MCAP (omitting .mcap automatically preserves extension) and verify companions move.
        response = await self.client.patch('/api/recordings/capture.mcap', json={
            'filename': 'pick_box_01',
            'marked': False,
        })
        self.assertEqual(response.status, 200, await response.text())
        body = await response.json()
        self.assertTrue(body['renamed'])
        self.assertEqual(body['id'], 'pick_box_01.mcap')
        self.assertFalse(raw.exists())
        renamed = self.root / 'recordings/pick_box_01.mcap'
        self.assertTrue(renamed.is_file())
        self.assertTrue(renamed.with_suffix('.takeovers.json').is_file())
        self.assertEqual(json.loads(renamed.with_suffix('.takeovers.json').read_text())['recording'], 'pick_box_01.mcap')
        self.assertFalse(body['item']['marked'])
        self.assertEqual(body['item']['task'], '双臂搬运箱子')
        self.assertEqual(body['item']['tags'], ['高质量', '双臂'])

        # Duplicate filename conflict -> 409.
        (self.root / 'recordings/existing.mcap').write_bytes(b'other')
        conflict = await self.client.patch('/api/recordings/pick_box_01.mcap', json={'filename': 'existing.mcap'})
        self.assertEqual(conflict.status, 409)

        # Invalid filename / quality -> 400.
        bad_name = await self.client.patch('/api/recordings/pick_box_01.mcap', json={'filename': '../escape.mcap'})
        self.assertEqual(bad_name.status, 400)
        bad_quality = await self.client.patch('/api/recordings/pick_box_01.mcap', json={'quality': 'invalid'})
        self.assertEqual(bad_quality.status, 400)

        # Deleting the dataset also removes its hidden metadata sidecar.
        meta_file = self.root / 'recordings/.pick_box_01.mcap.meta.json'
        self.assertTrue(meta_file.is_file())
        deleted = await self.client.delete('/api/recordings/pick_box_01.mcap')
        self.assertEqual(deleted.status, 200)
        self.assertFalse(meta_file.exists())

    async def test_patch_lerobot_dataset_preserves_sealed_payload_and_supports_rename(self):
        info_before = (self.dataset / 'meta/info.json').read_bytes()
        sync_before = (self.dataset / 'meta/teleop_sync.json').read_bytes()
        url = '/api/recordings/' + quote('lerobot:episode', safe='')

        # Metadata override does not mutate meta/info.json or meta/teleop_sync.json.
        response = await self.client.patch(url, json={
            'task': '整理桌面',
            'robot_label': '自定义X1',
            'tags': ['精选'],
            'notes': '已人工复核',
            'quality': 'review',
            'marked': True,
        })
        self.assertEqual(response.status, 200, await response.text())
        body = await response.json()
        self.assertEqual(body['item']['task'], '整理桌面')
        self.assertEqual(body['item']['profile_display_name'], '自定义X1')
        self.assertTrue(body['item']['marked'])
        self.assertEqual((self.dataset / 'meta/info.json').read_bytes(), info_before)
        self.assertEqual((self.dataset / 'meta/teleop_sync.json').read_bytes(), sync_before)

        # Clearing robot_label restores auto-detected robot_type from meta/info.json.
        response = await self.client.patch(url, json={'robot_label': ''})
        self.assertEqual(response.status, 200)
        body = await response.json()
        self.assertEqual(body['item']['profile_display_name'], 'x1')

        # Rename LeRobot dataset directory.
        response = await self.client.patch(url, json={'filename': 'tidy_desk_01'})
        self.assertEqual(response.status, 200, await response.text())
        body = await response.json()
        self.assertTrue(body['renamed'])
        self.assertEqual(body['id'], 'lerobot:tidy_desk_01')
        self.assertFalse(self.dataset.exists())
        self.assertTrue((self.root / 'recordings/lerobot/tidy_desk_01/meta/info.json').is_file())
        listed = await (await self.client.get('/api/recordings?format=lerobot')).json()
        self.assertEqual([item['id'] for item in listed['files']], ['lerobot:tidy_desk_01'])
        self.assertEqual(listed['files'][0]['task'], '整理桌面')
        self.assertEqual(listed['files'][0]['tags'], ['精选'])
        self.assertTrue(listed['files'][0]['marked'])

    async def test_batch_metadata_updates_multiple_datasets_and_rejects_rename(self):
        (self.root / 'recordings/one.mcap').write_bytes(b'one')
        (self.root / 'recordings/two.mcap').write_bytes(b'two')
        await self.client.patch('/api/recordings/one.mcap', json={'tags': ['旧标签', '保留']})

        bad = await self.client.post('/api/recordings/batch-metadata', json={
            'filenames': ['one.mcap'], 'updates': {'filename': 'nope.mcap'},
        })
        self.assertEqual(bad.status, 400)

        res = await self.client.post('/api/recordings/batch-metadata', json={
            'filenames': ['one.mcap', 'two.mcap'],
            'updates': {
                'task': '批量任务',
                'quality': 'good',
                'marked': True,
                'add_tags': ['新标签'],
                'remove_tags': ['旧标签'],
            },
        })
        self.assertEqual(res.status, 200, await res.text())
        body = await res.json()
        self.assertEqual(body['updated'], ['one.mcap', 'two.mcap'])
        self.assertEqual(body['failed'], [])

        listed = await (await self.client.get('/api/recordings?format=mcap')).json()
        by_id = {item['id']: item for item in listed['files']}
        self.assertEqual(by_id['one.mcap']['tags'], ['保留', '新标签'])
        self.assertEqual(by_id['two.mcap']['tags'], ['新标签'])
        self.assertEqual(by_id['one.mcap']['task'], '批量任务')
        self.assertEqual(by_id['two.mcap']['quality'], 'good')
        self.assertTrue(by_id['two.mcap']['marked'])

    async def test_patch_rejects_active_recording_and_active_replay_rename(self):
        start = await self.client.post('/api/recording/start', json={
            'filename': 'live.mcap', 'force': True, 'selection': {'mode': 'local'}})
        self.assertEqual(start.status, 200, await start.text())
        try:
            conflict = await self.client.patch('/api/recordings/live.mcap', json={'notes': 'while recording'})
            self.assertEqual(conflict.status, 409)
        finally:
            await self.client.post('/api/recording/stop')

        # Active replay blocks renaming the file being played, while allowing non-rename metadata edits.
        from unittest.mock import patch
        raw = self.root / 'recordings/replaying.mcap'
        raw.write_bytes(b'mcap payload')
        from middleware.core.recording.topic_player import TopicPlayer
        with patch.object(TopicPlayer, 'status', return_value={'state': 'playing', 'is_active': True, 'filename': 'replaying.mcap'}):
            rename_res = await self.client.patch('/api/recordings/replaying.mcap', json={'filename': 'moved.mcap'})
            self.assertEqual(rename_res.status, 409)
            meta_res = await self.client.patch('/api/recordings/replaying.mcap', json={'notes': 'ok during replay'})
            self.assertEqual(meta_res.status, 200)

    def test_task_recorder_blocks_rename_when_bound_but_allows_metadata_edit(self):
        from types import SimpleNamespace
        from middleware.core.uploads.task_recording import TaskRecorder
        from middleware.core.recording.dataset_metadata import RecordingConflictError

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rec_dir = root / 'recordings'
            rec_dir.mkdir()
            bound_file = rec_dir / 'bound.mcap'
            bound_file.write_bytes(b'bound payload')
            local_file = rec_dir / 'local.mcap'
            local_file.write_bytes(b'local payload')

            recorder = TopicRecorder({'directory': str(rec_dir)}, root)
            fake_platform = SimpleNamespace(
                directory=root / 'platform',
                upload=SimpleNamespace(
                    artifact_process=SimpleNamespace(closed=False, close=lambda: None),
                    state_dir=root / 'state',
                    queue=SimpleNamespace(jobs=lambda: []),
                ),
                config={},
                state={'uploads_enabled': False, 'upload_format': 'mcap'},
                recording_selection=lambda: {'mode': 'local'},
            )
            fake_platform.upload.state_dir.mkdir(parents=True)
            task_rec = TaskRecorder(recorder, fake_platform)
            try:
                task_rec.jobs['job-bound'] = {
                    'recording_id': 'job-bound',
                    'path': str(bound_file),
                    'chosen_filename': 'bound.mcap',
                    'state': 'ready',
                    'format': 'mcap',
                    'binding': {'binding_id': 'bind-1', 'task': {'name': '平台任务'}},
                    'artifacts': {},
                }
                task_rec.jobs['job-local'] = {
                    'recording_id': 'job-local',
                    'path': str(local_file),
                    'chosen_filename': 'local.mcap',
                    'state': 'ready',
                    'format': 'mcap',
                    'binding': None,
                    'artifacts': {},
                }
                # Bound recording refuses rename, but accepts metadata updates.
                with self.assertRaises(RecordingConflictError):
                    task_rec.update_recording('bound.mcap', {'filename': 'renamed_bound.mcap'})
                meta_res = task_rec.update_recording('bound.mcap', {'tags': ['精选'], 'notes': '已绑定任务仍可备注'})
                self.assertEqual(meta_res['item']['tags'], ['精选'])
                self.assertEqual(meta_res['item']['notes'], '已绑定任务仍可备注')

                # Local unbound recording can be renamed and updates job path cleanly.
                rename_res = task_rec.update_recording('local.mcap', {'filename': 'local_renamed.mcap', 'task': '本地抓取'})
                self.assertTrue(rename_res['renamed'])
                self.assertEqual(rename_res['id'], 'local_renamed.mcap')
                self.assertEqual(Path(task_rec.jobs['job-local']['path']).name, 'local_renamed.mcap')
                self.assertEqual(task_rec.jobs['job-local']['chosen_filename'], 'local_renamed.mcap')
            finally:
                task_rec.close()
