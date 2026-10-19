"""Optional upload integration must preserve main's local recording lifecycle."""
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

from aiohttp.test_utils import TestClient, TestServer
from middleware.core.app import MiddlewareRuntime, create_app
from middleware.core.config import ConfigError, ConfigStore, validate_config


class UploadConfigTests(unittest.TestCase):
    def test_defaults_keep_upload_disabled_and_local_recording(self):
        config = validate_config({})
        self.assertFalse(config['uploads']['enabled'])
        self.assertFalse(config['uploads']['auto_enqueue'])
        self.assertEqual(config['ros']['recording']['format'], 'mcap')
        self.assertNotIn('workflow', config['ros']['recording'])
        self.assertEqual(config['ros']['command_mux']['source'], 'vr')

    def test_credentials_are_references_and_flags_boolean(self):
        for values in ({'enabled': 'false'}, {'auto_enqueue': 1},
                       {'credential_ref': 'secret-token'}, {'queue_directory': ''}):
            with self.subTest(values=values), self.assertRaises(ConfigError):
                validate_config({'uploads': values})
        self.assertEqual(validate_config({'uploads': {'credential_ref': 'env:UPLOAD_TOKEN'}})
                         ['uploads']['credential_ref'], 'env:UPLOAD_TOKEN')


class UploadRuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.runtime = MiddlewareRuntime(validate_config({}), self.root)
        self.runtime.recorder = SimpleNamespace(
            artifact_state_dir=self.root / 'artifacts', directory=self.root / 'recordings',
            binding_snapshot=None, upload_service=None, shutdown=Mock())

    async def asyncTearDown(self):
        self.temp.cleanup()

    async def test_missing_optional_dependency_does_not_stop_recording(self):
        with patch.dict(sys.modules, {'middleware.core.upload_service': None}):
            await self.runtime._start_uploads()
        self.assertIsNone(self.runtime.uploads)
        self.assertTrue(self.runtime.upload_error)
        self.assertIsNone(self.runtime.recorder.binding_snapshot)

    async def test_binding_is_only_attached_when_enabled_and_automatic(self):
        for enabled, automatic in ((False, True), (True, False), (True, True)):
            service = SimpleNamespace(start=Mock(), stop=Mock(), binding_snapshot=Mock())
            factory = Mock(return_value=service)
            self.runtime.config['uploads'].update(enabled=enabled, auto_enqueue=automatic)
            self.runtime.recorder.binding_snapshot = None
            with patch.dict(sys.modules, {'middleware.core.upload_service': SimpleNamespace(UploadService=factory)}):
                await self.runtime._start_uploads()
            self.assertEqual(factory.call_args.kwargs, {
                'artifact_state_dir': self.runtime.recorder.artifact_state_dir,
                'data_directory': self.runtime.recorder.directory})
            self.assertEqual(Path(factory.call_args.args[0]['queue_directory']),
                             (self.root / '../runtime/upload_queue').resolve())
            self.assertEqual(self.runtime.recorder.binding_snapshot,
                             service.binding_snapshot if enabled and automatic else None)
            service.start.assert_called_once()

    async def test_shutdown_finishes_recording_before_stopping_uploader(self):
        order = []
        self.runtime.recorder.shutdown.side_effect = lambda: order.append('recorder')
        self.runtime.uploads = SimpleNamespace(stop=lambda: order.append('uploads'))
        await self.runtime.stop()
        self.assertEqual(order, ['recorder', 'uploads'])
        self.assertIsNone(self.runtime.uploads)

    async def test_unavailable_upload_api_still_serves_main(self):
        store = ConfigStore(self.root / 'config.yaml')
        with patch.dict(sys.modules, {'middleware.core.upload_api': None}):
            app = create_app(store)
        with patch.object(MiddlewareRuntime, 'start', new=AsyncMock()), \
             patch.object(MiddlewareRuntime, 'stop', new=AsyncMock()):
            async with TestClient(TestServer(app)) as client:
                response = await client.get('/api/uploads')
                self.assertEqual(response.status, 503)
                self.assertEqual((await response.json())['state'], 'unavailable')
                response = await client.post('/api/uploads/enqueue', json={})
                self.assertEqual(response.status, 503)
                response = await client.get('/api/config')
                self.assertEqual(response.status, 200)

    async def test_installed_but_disabled_uploader_exposes_read_only_status(self):
        client_source = str(Path(__file__).resolve().parents[1] / 'upload_client')
        with patch.object(sys, 'path', [client_source, *sys.path]):
            from middleware.core.upload_service import UploadService
            from middleware.core.upload_api import register_upload_routes
        from aiohttp import web
        service = UploadService({'enabled': False}, artifact_state_dir=self.root / 'state',
                                data_directory=self.root / 'recordings')
        app = web.Application()
        register_upload_routes(app, service)
        async with TestClient(TestServer(app)) as client:
            response = await client.get('/api/uploads')
            self.assertEqual(response.status, 200)
            status = await response.json()
            self.assertFalse(status['enabled'])
            self.assertFalse(status['running'])
            response = await client.post('/api/uploads/enqueue', json={'artifact_id': 'unused'})
            self.assertEqual(response.status, 400)
            self.assertEqual((await response.json())['code'], 'UPLOAD_DISABLED')
        self.assertFalse((self.root / 'state').exists())

    async def test_upload_read_lease_blocks_delete_and_preserves_batch_failures(self):
        client_source = str(Path(__file__).resolve().parents[1] / 'upload_client')
        with patch.object(sys, 'path', [client_source, *sys.path]):
            from humanoid_upload.artifacts import ArtifactLocks
        from middleware.core.topic_recorder import TopicRecorder
        recorder = TopicRecorder({'directory': str(self.root / 'recordings'),
                                  'artifact_state_dir': str(self.root / 'artifacts'),
                                  'upload_job_state_dir': str(self.root / 'jobs')}, self.root)
        self.runtime.recorder = recorder
        recorder.directory.mkdir()
        blocked = recorder.directory / 'blocked.mcap'
        free = recorder.directory / 'free.mcap'
        blocked.write_bytes(b'capture')
        free.write_bytes(b'capture')
        identity = recorder._artifacts.begin(blocked, 'mcap', {})
        store = ConfigStore(self.root / 'config.yaml')
        with patch('middleware.core.app.MiddlewareRuntime', return_value=self.runtime):
            app = create_app(store)
        with patch.object(self.runtime, 'start', new=AsyncMock()), \
             patch.object(self.runtime, 'stop', new=AsyncMock()):
            async with TestClient(TestServer(app)) as client:
                with ArtifactLocks(recorder.artifact_state_dir).read(identity + '-mcap'):
                    response = await client.delete('/api/recordings/blocked.mcap')
                    self.assertEqual(response.status, 409)
                    self.assertEqual((await response.json())['code'], 'ARTIFACT_BUSY')
                    self.assertTrue(blocked.exists())
                    response = await client.post('/api/recordings/batch-delete',
                                                 json={'filenames': ['blocked.mcap', 'free.mcap']})
                    self.assertEqual(response.status, 200)
                    result = await response.json()
                    self.assertEqual(result['deleted'], ['free.mcap'])
                    self.assertEqual(result['failed'][0]['code'], 'ARTIFACT_BUSY')
                response = await client.delete('/api/recordings/blocked.mcap')
                self.assertEqual(response.status, 200)
                self.assertFalse(blocked.exists())
        recorder.shutdown()

    async def test_auto_upload_without_binding_reports_http_and_vr_error(self):
        client_source = str(Path(__file__).resolve().parents[1] / 'upload_client')
        with patch.object(sys, 'path', [client_source, *sys.path]):
            from middleware.core.upload_service import UploadService
        from middleware.core.topic_recorder import TopicRecorder
        recorder = TopicRecorder({'directory': str(self.root / 'recordings'),
                                  'artifact_state_dir': str(self.root / 'artifacts'),
                                  'upload_job_state_dir': str(self.root / 'jobs')}, self.root)
        service = UploadService({'enabled': True, 'queue_directory': str(self.root / 'queue')},
                                artifact_state_dir=recorder.artifact_state_dir,
                                data_directory=recorder.directory)
        recorder.binding_snapshot = service.binding_snapshot
        self.runtime.recorder = recorder
        store = ConfigStore(self.root / 'config.yaml')
        with patch('middleware.core.app.MiddlewareRuntime', return_value=self.runtime):
            app = create_app(store)
        with patch.object(self.runtime, 'start', new=AsyncMock()), \
             patch.object(self.runtime, 'stop', new=AsyncMock()):
            async with TestClient(TestServer(app)) as client:
                response = await client.post('/api/recording/start', json={'force': True})
                self.assertEqual(response.status, 409)
                result = await response.json()
                self.assertEqual(result['code'], 'BINDING_REQUIRED')
                self.assertIn('任务', result['reason'])
                self.assertFalse(recorder.is_recording())
            with patch.object(self.runtime, 'emit') as emit:
                self.runtime._handle_vr_record_start()
                event = next(call.args[0] for call in emit.call_args_list
                             if call.args[0].get('kind') == 'recording_error')
                self.assertEqual(event['payload']['code'], 'BINDING_REQUIRED')
                self.assertIn('任务', event['payload']['reason'])
        recorder.shutdown()
        service.stop()
