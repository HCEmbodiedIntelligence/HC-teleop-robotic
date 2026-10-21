"""Run with HC_PLATFORM_SOURCE=<platform>/backend/src and platform dependencies.

Real robot recorder, client HTTP/multipart transport, platform routes/service and
in-memory object storage. PostgreSQL durability is tested in the platform suite.
"""
import json
import os
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import pytest

if not os.environ.get('HC_PLATFORM_SOURCE'):
    pytest.skip('HC_PLATFORM_SOURCE required for cross-repository HTTP integration', allow_module_level=True)
sys.path.insert(0, os.environ['HC_PLATFORM_SOURCE'])
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'upload_client'))
sys.path.insert(0, str(Path(os.environ['HC_PLATFORM_SOURCE']).parent))

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.testclient import TestClient
from hc_data_platform.core.errors import ProblemException
from hc_data_platform.robot_ingest.connection_api import router as connections_router
from hc_data_platform.robot_ingest.router import router, get_robot_ingest_service
from hc_data_platform.robot_ingest.connections import LocalRobot
from hc_data_platform.robot_ingest.onboarding_models import OnboardingFacts
from hc_data_platform.robot_ingest.models import CreateRobotIngestIdentity, UploadTarget
from hc_data_platform.robot_ingest.repository import InMemoryRobotIngestRepository
from hc_data_platform.robot_ingest.service import RobotIngestService
from hc_data_platform.security.auth import AuthContext
from datetime import datetime, timezone

from middleware.core.platform_connection import PlatformConnection
from middleware.core.task_recording import TaskRecorder
from middleware.core.topic_recorder import TopicRecorder
from humanoid_upload.common import UploadError
from test_platform_task_recording import LOCAL, capture, choice, finish
from capture_contract_support import task_document, snapshot_document
from types import SimpleNamespace


class MemoryCapturePlans:
    """Implements the platform RecordingPlans port; durable PostgreSQL remains a separate gate."""
    def __init__(self):
        self.snapshots = {}

    def preview(self, scope, task_id, robot_id, source_format):
        task = task_document(task_id, source_format, version=1 if task_id == 'A' else 2)
        return SimpleNamespace(plan=task['capture_plan'], processing_config={'source': 'TASK_PLAN'})

    def prepare(self, scope, task_id, robot_id, command, actor):
        payload = command.model_dump(mode='json')
        task = task_document(task_id, payload['source_format'], version=1 if task_id == 'A' else 2)
        return self.snapshots.setdefault(payload['recording_id'], snapshot_document(task, payload, robot_id))


def test_single_link_real_http_a_b_offline_restart_and_scope(tmp_path, monkeypatch):
    now = datetime.now(timezone.utc)
    repo = InMemoryRobotIngestRepository(targets=tuple(UploadTarget(
        collection_task_id=task, organization_id='foreign' if task == 'X' else 'org',
        project_id='project-'+task, dataset_id='dataset-'+task, region_code='region', task_status='ACTIVE') for task in 'ABX'))
    service = RobotIngestService.in_memory(repository=repo)
    service.connections.capture_plans = MemoryCapturePlans()
    auth = AuthContext(subject_id='test-admin', organization_ids=frozenset({'org'}),
        project_ids=frozenset({'project-A'}), region_codes=frozenset({'region'}),
        organization_scope_triples=frozenset({('org','project-A','region')}),
        organization_scoped_capabilities=frozenset({('org','project-A','ingest_source.manage')}))
    identity = service.create_identity(auth=auth,organization_id='org',project_id='project-A',
        command=CreateRobotIngestIdentity(robot_id='robot',allowed_formats=('MCAP',))).data
    repo.robot_facts[('org','robot')] = OnboardingFacts(robot_id='robot',display_name='Test Robot',serial_no='test',
        lifecycle_status='ACTIVE',model_version_id='test-model',updated_at=now)
    app = FastAPI()
    app.include_router(router); app.include_router(connections_router)
    app.dependency_overrides[get_robot_ingest_service] = lambda: service
    @app.exception_handler(ProblemException)
    async def error(request, exc):
        return JSONResponse(exc.problem.model_dump(mode='json'), status_code=exc.problem.status)
    client = TestClient(app)
    faults = {'redeem': True, 'commit': True, 'offline': False}
    puts = []
    protocol_errors = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def do_GET(self): self.api()
        def do_POST(self): self.api()
        def do_PUT(self):
            assert not self.headers.get('Authorization')
            _, _, multipart, number = urlsplit(self.path).path.split('/')
            part = service.storage.upload_part(multipart,int(number),self.rfile.read(int(self.headers['Content-Length'])))
            puts.append((multipart,number))
            self.send_response(200); self.send_header('ETag',part.etag); self.send_header('Content-Length','0'); self.end_headers()
        def api(self):
            if faults['offline']:
                self.connection.shutdown(socket.SHUT_RDWR); self.connection.close(); return
            raw = self.rfile.read(int(self.headers.get('Content-Length',0)))
            result = client.request(self.command,self.path,content=raw or None,
                headers={k:v for k,v in self.headers.items() if k.lower() in ('content-type','authorization')})
            data = result.json()
            if isinstance(data.get('data'), dict) and 'manifest' in data['data']:
                from humanoid_upload.common import validator
                protocol_errors.extend((list(error.path), error.message) for error in
                                       validator('robot-ingest-upload-response').iter_errors(data))
            if 'authorizations' in data:
                for item in data['authorizations']:
                    item['url'] = base+'/objects'+urlsplit(item['url']).path
            key = 'redeem' if self.path.endswith('connections:redeem') else 'commit' if self.path.endswith(':commit') else ''
            if key and faults.get(key) and result.status_code < 300:
                faults[key] = False; self.connection.shutdown(socket.SHUT_RDWR); self.connection.close(); return
            payload = json.dumps(data).encode()
            self.send_response(result.status_code); self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(payload))); self.end_headers(); self.wfile.write(payload)
    server = ThreadingHTTPServer(('127.0.0.1',0),Handler)
    base = f'http://127.0.0.1:{server.server_port}'
    thread = threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    cfg = {'ros': {'recording': {'format':'mcap','directory':str(tmp_path/'data')}},
           'uploads': {'queue_directory':str(tmp_path/'queue'),'timeout_seconds':2,'poll_seconds':1}}
    cfg['ros']['subscriptions'] = [{'topic':'/test', 'type':'sensor_msgs/msg/JointState', 'outputs':['record']}]
    monkeypatch.setenv('no_proxy','127.0.0.1')
    def local_service():
        value = PlatformConnection(cfg,tmp_path,tmp_path/'data',identity_provider=lambda:dict(LOCAL))
        monkeypatch.setattr(value.upload,'start',lambda:None)
        value.capture_health = lambda: {'/test': {'has_data':True, 'hz':15, 'state':'ok'}}
        return value
    platform = local_service(); recorder = None
    try:
        ticket = service.connections.issue(auth,'org','project-A',identity.ingest_identity_id)
        link = base+'/#robot-connect='+ticket['ticket']
        with pytest.raises(UploadError,match='NETWORK_UNCERTAIN'): platform.connect(link)
        platform = local_service()  # Lost connection response, then service restart.
        platform.connect(link)
        assert len(repo.list_credentials('org',identity.ingest_identity_id)) == 1
        assert {t['collection_task_id'] for t in platform.tasks()['items']} == {'A','B'}
        recorder = TaskRecorder(TopicRecorder(cfg['ros']['recording'],tmp_path),platform)
        capture(recorder,choice(platform,'A'))
        recorder.start(selection=choice(platform,'B'))
        faults['offline'] = True
        recorder.record({'topic':'/test','payload':{'disconnected':True}})
        recorder.stop(); finish(recorder)
        platform.upload.step(scan=False)
        assert platform.upload.queue.jobs()[0]['state'] == 'retry_wait'
        with pytest.raises(UploadError): recorder.start(selection=choice(platform,'A'))
        capture(recorder,{'mode':'local'})
        recorder.close(); recorder = None
        faults['offline'] = False
        platform = local_service()
        # Reconcile the lost commit response, never resend committed bytes.
        for _ in range(4):
            for job in platform.upload.queue.jobs():
                if not job['terminal']:
                    job['next_at'] = 0; platform.upload.queue.save(job)
            platform.upload.step(scan=False)
        jobs = platform.upload.queue.jobs()
        assert not protocol_errors, protocol_errors
        assert len(jobs) == 2 and {j['server_state'] for j in jobs} == {'COMMITTED'}
        assert len(puts) == 2 and len(repo._uploads) == 2
        assert [j['manifest']['collection_task_id'] for j in jobs] == ['A','B']
        assert all(j['quality_status'] == 'PENDING' for j in jobs)
        with pytest.raises(UploadError): platform.freeze(choice(platform,'X'))
        recorder = TaskRecorder(TopicRecorder(cfg['ros']['recording'],tmp_path),platform)
        local = next(j for j in recorder.records() if not j['binding'] and j['intent']['mode'] == 'local')
        recorder.upload_recording(local['recording_id'],choice(platform,'A')); finish(recorder)
        repo._targets['A'] = repo._targets['A'].model_copy(update={'dataset_id':'changed'})
        platform.upload.step(scan=False)
        job = platform.upload.queue.jobs()[-1]
        assert job['code'] == 'TASK_OWNERSHIP_CHANGED' and job['terminal']
        assert Path(local['path']).exists() and len(repo._uploads) == 2
    finally:
        if recorder: recorder.close()
        platform.stop(); server.shutdown(); server.server_close(); thread.join(); client.close()
