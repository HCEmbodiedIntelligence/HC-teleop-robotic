import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path
from urllib.parse import quote

from aiohttp.test_utils import TestClient, TestServer
from middleware.core.config import ConfigStore
from middleware.core.topic_recorder import TopicRecorder
from middleware.core.recording_datasets import resolve_dataset


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
            self.assertEqual((await response.json())['code'], 'RECORDING_ACTIVE')
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
