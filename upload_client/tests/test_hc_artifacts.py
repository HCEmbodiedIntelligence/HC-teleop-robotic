"""Offline transport tests; synthetic files do not certify media decoding."""
import asyncio
import copy
import hashlib
import shutil
import threading
from pathlib import Path
import pytest
from humanoid_upload.artifacts import ArtifactLocks
from humanoid_upload.bundle import checksum
from humanoid_upload.common import DEFAULTS, UploadError, atomic_json, read_json
from humanoid_upload.queue import Queue
from humanoid_upload.worker import Worker
from core.upload_service import UploadService

@pytest.fixture
def hc_cfg(tmp_path):
    return dict(DEFAULTS, platform_base_url='http://127.0.0.1:1', credential_ref='env:HC_TEST_TOKEN',
        robot_id='robot-1', local_robot_id='local-1', task_id='task-1', data_directory=str(tmp_path/'data'),
        queue_directory=str(tmp_path/'queue'), artifact_state_dir=str(tmp_path/'ready'), binding_id='binding-1',
        read_bytes_per_second=1e9, bandwidth_bytes_per_second=1e9)

def ready(cfg, kind='hc_mcap', identity='artifact-1'):
    root=Path(cfg['data_directory'])/identity; root.mkdir(parents=True)
    if kind=='hc_mcap': (root/'capture.mcap').write_bytes(b'\x89MCAP0\r\nfixture\x89MCAP0\r\n')
    elif kind=='hc_lerobot':
        atomic_json(root/'meta/info.json',dict(codebase_version='v3.0',features={'action':{}},total_frames=1))
        (root/'data.parquet').write_bytes(b'PAR1fixturePAR1')
    else:
        shutil.copytree(Path(__file__).parent/'fixtures/openarm-raw-v1',root,dirs_exist_ok=True)
        marker=read_json(root/'recording-complete.json')
        cfg.update(robot_id=marker['robot_id'],local_robot_id=marker['local_robot_id'],task_id=marker['collection_task_id'])
    d=dict(schema_version='hc-artifact-ready/v1',recording_id='recording-1',artifact_id=identity,kind=kind,root=str(root),
        profile_id='test-profile',config_fingerprint='offline',source_artifact_id=None,
        native_session_id=marker['session_id'] if kind=='platform_raw' else None,
        native_completion_marker='recording-complete.json' if kind=='platform_raw' else None,
        binding_id=cfg.get('binding_id'),completed_at='2026-10-15T00:01:00Z',
        capture_started_at='2026-10-15T00:00:00Z',capture_ended_at='2026-10-15T00:00:01Z',
        files=[dict(path=p.relative_to(root).as_posix(),**{k:v for k,v in checksum(p).items() if k!='crc64'})
               for p in sorted(root.rglob('*')) if p.is_file()])
    atomic_json(Path(cfg['artifact_state_dir'])/(identity+'.json'),d)
    return d

class OfflinePlatform:
    def __init__(self):
        self.upload=None; self.parts={}; self.fail_commit=False; self.put_hook=None; self.creates=0; self.puts=0
    def api(self,method,suffix='',payload=None):
        if not suffix:
            self.creates+=1; m=payload
            if self.upload is None:
                self.upload=dict(upload_id=m['client_upload_id'],client_upload_id=m['client_upload_id'],ingest_identity_id='test',
                    authenticated_robot_id=m['robot_id'],request_robot_id=m['robot_id'],credential_id='test',credential_version=1,
                    collection_job_id='test', target=dict(collection_task_id=m['collection_task_id'],organization_id='test',
                    project_id='test',dataset_id='test',region_code='test',task_status='ACTIVE'),
                    **{k:m[k] for k in ('capture_mode','source_format','source_format_version','capture_started_at','capture_ended_at')},
                    total_bytes=sum(a['size_bytes'] for a in m['assets']),state='UPLOADING',manifest_fingerprint='0'*64,
                    manifest=copy.deepcopy(m),cameras=[],assets=[dict(asset_id=a['asset_id'],path=a['path'],role=a['role'],
                    media_type=a['media_type'],object_key=a['path'],multipart_upload_id='test',state='UPLOADING',
                    **{'expected_'+k:a[k] for k in ('size_bytes','sha256','crc64')}) for a in m['assets']],
                    created_at=m['capture_ended_at'],expires_at='2099-01-01T00:00:00Z',updated_at=m['capture_ended_at'],
                    processing_status='PENDING',quality_status='PENDING',raw_source_id=None)
        elif suffix.endswith(':authorize-parts'):
            aid=suffix.split('/assets/')[1].split(':')[0]
            return dict(upload_id=self.upload['upload_id'],asset_id=aid,uploaded_parts=list(self.parts.get(aid,{}).values()),
                authorizations=[dict(part_number=n,url=f'{aid}/{n}') for n in payload['part_numbers']])
        elif suffix.endswith(':complete'):
            aid=suffix.split('/assets/')[1].split(':')[0]
            next(a for a in self.upload['assets'] if a['asset_id']==aid)['state']='COMPLETED'
        elif suffix.endswith(':commit'):
            self.upload.update(state='COMMITTED',raw_source_id='raw-test')
            if self.fail_commit:
                self.fail_commit=False; raise UploadError('NETWORK_UNCERTAIN',retry=True)
        elif suffix.endswith('/recording'):
            return dict(upload_id=self.upload['upload_id'],raw_source_id='raw-test',status='AWAITING_SLICE',terminal=True,
                recording_id='remote-recording',page_path='/recordings/remote-recording/slice',poll_after_seconds=0)
        return {'data':copy.deepcopy(self.upload)}
    def put(self,url,stream,size):
        self.puts+=1
        if self.put_hook: self.put_hook()
        data=stream.read(size); aid,number=url.split('/')
        expected=next(a for a in self.upload['assets'] if a['asset_id']==aid)
        assert hashlib.sha256(data).hexdigest()==expected['expected_sha256']
        self.parts.setdefault(aid,{})[int(number)]=dict(part_number=int(number),size_bytes=len(data),etag='test')

@pytest.mark.parametrize('kind',['hc_mcap','hc_lerobot','platform_raw'])
def test_worker_restart_commit_reconciliation(kind,hc_cfg,monkeypatch):
    d=ready(hc_cfg,kind); q=Queue(hc_cfg['queue_directory']); job=q.enqueue_artifact(d,hc_cfg)
    assert q.enqueue_artifact(d,hc_cfg)['id']==job['id']
    api=OfflinePlatform(); api.fail_commit=True; monkeypatch.setattr('humanoid_upload.worker.Http',lambda cfg:api)
    Worker(q,hc_cfg).tick(); assert q.jobs()[0]['state']=='retry_wait'; puts=api.puts
    q=Queue(hc_cfg['queue_directory']); q.retry(job['id']); Worker(q,dict(hc_cfg,task_id='new-default')).tick()
    result=q.jobs()[0]
    assert api.creates==1 and api.puts==puts and result['frozen']['task_id']==hc_cfg['task_id']
    assert result['client_upload_id']==job['id'] and result['server_state']=='COMMITTED'
    assert result['quality_status']=='PENDING'
    assert result['state']==('awaiting_slice' if kind=='platform_raw' else 'raw_committed')
    assert result['manifest']['source_format']=={'hc_mcap':'MCAP','hc_lerobot':'LEROBOT_V3','platform_raw':'CAPTURE_BUNDLE'}[kind]

@pytest.mark.parametrize('mutation',['hash','missing','truncated','escape','symlink','duplicate','time','raw_marker'])
def test_reject_incomplete_unsafe(mutation,hc_cfg):
    d=ready(hc_cfg,'platform_raw' if mutation=='raw_marker' else 'hc_mcap'); path=Path(d['root'])/d['files'][0]['path']
    if mutation=='hash': d['files'][0]['sha256']='0'*64
    elif mutation=='missing': path.unlink()
    elif mutation=='truncated':
        path.write_bytes(b'not closed'); d['files'][0].update({k:v for k,v in checksum(path).items() if k!='crc64'})
    elif mutation=='escape': d['files'][0]['path']='../capture.mcap'
    elif mutation=='symlink':
        other=path.with_name('other'); path.rename(other); path.symlink_to(other)
    elif mutation=='duplicate': d['files'].append(d['files'][0])
    elif mutation=='time': d.pop('capture_started_at')
    else: d['native_completion_marker']=None
    with pytest.raises(UploadError): Queue(hc_cfg['queue_directory']).enqueue_artifact(d,hc_cfg)

def test_worker_revalidates_before_network(hc_cfg,monkeypatch):
    d=ready(hc_cfg); q=Queue(hc_cfg['queue_directory']); q.enqueue_artifact(d,hc_cfg)
    (Path(d['root'])/'capture.mcap').write_bytes(b'changed')
    api=OfflinePlatform(); monkeypatch.setattr('humanoid_upload.worker.Http',lambda cfg:api); Worker(q,hc_cfg).tick()
    assert api.creates==0 and q.jobs()[0]['code']=='ASSET_INTEGRITY'

def test_cancel_read_delete_race(hc_cfg,monkeypatch):
    d=ready(hc_cfg); q=Queue(hc_cfg['queue_directory']); job=q.enqueue_artifact(d,hc_cfg)
    api=OfflinePlatform(); entered=threading.Event(); release=threading.Event()
    api.put_hook=lambda:(entered.set(),release.wait(5)); monkeypatch.setattr('humanoid_upload.worker.Http',lambda cfg:api)
    worker=threading.Thread(target=Worker(q,hc_cfg).tick); worker.start(); assert entered.wait(3)
    q.cancel(job['id']); locks=ArtifactLocks(hc_cfg['artifact_state_dir']); assert locks.usage(d['artifact_id'])['reading']
    with pytest.raises(UploadError,match='ARTIFACT_BUSY'):
        with locks.delete(d['artifact_id']): pass
    release.set(); worker.join(5); assert not worker.is_alive(); assert q.jobs()[0]['state']=='cancelled'
    with locks.delete(d['artifact_id']): shutil.rmtree(d['root'])
    with pytest.raises(UploadError,match='ARTIFACT_DELETED'):
        with locks.read(d['artifact_id']): pass
    q.retry(job['id']); Worker(q,hc_cfg).tick(); assert q.jobs()[0]['code']=='ARTIFACT_DELETED'

def service(cfg):
    return UploadService({k:v for k,v in dict(cfg,enabled=True).items() if k not in ('artifact_state_dir','binding_id')},
        artifact_state_dir=cfg['artifact_state_dir'],data_directory=cfg['data_directory'])

def test_binding_scan_restart(hc_cfg):
    svc=service(hc_cfg); snapshot=svc.binding_snapshot(); hc_cfg['binding_id']=snapshot['binding_id']; ready(hc_cfg)
    svc.scan(); first=svc.jobs()[0]; svc.set_binding({**svc.bindings[snapshot['binding_id']],'task_id':'task-2'})
    svc.scan(); assert len(svc.jobs())==1 and svc.jobs()[0]['task_id']=='task-1'
    ready(dict(hc_cfg,binding_id=None),identity='historical'); svc.scan(); assert len(svc.jobs())==1
    historical = svc.enqueue('historical')
    assert historical['task_id']=='task-2'
    svc.set_binding({**svc.bindings[snapshot['binding_id']], 'task_id':'task-3'})
    assert svc.enqueue('historical')['id']==historical['id']
    svc.set_binding({**svc.bindings[snapshot['binding_id']], 'task_id':'task-2'})
    restarted=service(hc_cfg); assert restarted.jobs()[0]['id']==first['id']
    assert restarted.binding_snapshot()['collection_task_id']=='task-2'
    assert 'credential_ref' not in restarted.status()['default_binding']

def test_api_secret_rejection(hc_cfg):
    from aiohttp import web
    from aiohttp.test_utils import TestClient,TestServer
    from core.upload_api import register_upload_routes
    async def run():
        svc=service(hc_cfg); app=web.Application(); register_upload_routes(app,svc)
        async with TestClient(TestServer(app)) as client:
            response=await client.post('/api/uploads/binding',json={'token':'private-secret'})
            assert response.status==400 and 'private-secret' not in await response.text()
            assert (await client.get('/api/uploads')).status==200
            assert '/api/recordings/upload' not in [r.resource.canonical for r in app.router.routes()]
    asyncio.run(run())

@pytest.mark.parametrize('kind',['hc_mcap','hc_lerobot','platform_raw'])
def test_real_http_transport_on_loopback(kind,hc_cfg,monkeypatch):
    """Exercise Http, actual multipart byte reads and lost commit response offline."""
    import io
    import json
    import socket
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    platform=OfflinePlatform(); platform.fail_commit=True
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_): pass
        def do_GET(self): self.handle_api()
        def do_POST(self): self.handle_api()
        def do_PUT(self):
            assert self.headers.get('Authorization') is None
            data=self.rfile.read(int(self.headers['Content-Length']))
            platform.put(self.path.removeprefix('/objects/'),io.BytesIO(data),len(data))
            self.send_response(200); self.send_header('ETag','test'); self.send_header('Content-Length','0'); self.end_headers()
        def handle_api(self):
            assert self.headers['Authorization']=='Bearer synthetic-hc-test'
            raw=self.rfile.read(int(self.headers.get('Content-Length',0)))
            suffix=self.path.removeprefix('/api/v1/robot-ingest/uploads')
            try: result=platform.api(self.command,suffix,json.loads(raw) if raw else None)
            except UploadError:
                self.connection.shutdown(socket.SHUT_RDWR); self.connection.close(); return
            if 'authorizations' in result:
                for grant in result['authorizations']: grant['url']=base+'/objects/'+grant['url']
            body=json.dumps(result).encode()
            self.send_response(200); self.send_header('Content-Length',str(len(body))); self.end_headers(); self.wfile.write(body)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler); base='http://127.0.0.1:'+str(server.server_port)
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    monkeypatch.setenv('HC_TEST_TOKEN','synthetic-hc-test'); monkeypatch.setenv('no_proxy','127.0.0.1')
    try:
        hc_cfg['platform_base_url']=base
        d=ready(hc_cfg,kind); q=Queue(hc_cfg['queue_directory']); job=q.enqueue_artifact(d,hc_cfg)
        Worker(q,hc_cfg).tick(); assert q.jobs()[0]['state']=='retry_wait'
        q=Queue(hc_cfg['queue_directory']); q.retry(job['id']); Worker(q,hc_cfg).tick()
        result=q.jobs()[0]
        assert result['server_state']=='COMMITTED' and result['quality_status']=='PENDING'
        assert platform.creates==1 and platform.puts==len(job['manifest']['assets'])
        assert result['uploaded_bytes']==sum(a['size_bytes'] for a in job['manifest']['assets'])
    finally:
        server.shutdown(); server.server_close(); thread.join()


def test_same_identity_changed_content_rejected(hc_cfg):
    d=ready(hc_cfg); q=Queue(hc_cfg['queue_directory']); q.enqueue_artifact(d,hc_cfg)
    path=Path(d['root'])/'capture.mcap'; path.write_bytes(b'\x89MCAP0\r\nchanged\x89MCAP0\r\n')
    d['files'][0].update({k:v for k,v in checksum(path).items() if k!='crc64'})
    with pytest.raises(UploadError,match='EXPORT_ID_REUSED'): q.enqueue_artifact(d,hc_cfg)


def test_raw_seal_only_submits_callback(hc_cfg):
    svc=service(hc_cfg); calls=[]
    svc.request_raw_seal=lambda rid,binding:(calls.append((rid,binding)) or {'job_id':'seal-1','state':'sealing'})
    result=svc.seal_raw('recording-1')
    assert result['job_id']=='seal-1' and svc.jobs()==[]
    assert set(calls[0][1])=={'binding_id','platform_base_url','robot_id','collection_task_id'}


def test_concurrent_duplicate_enqueue(hc_cfg):
    from concurrent.futures import ThreadPoolExecutor
    d=ready(hc_cfg); q=Queue(hc_cfg['queue_directory'])
    with ThreadPoolExecutor(max_workers=4) as pool:
        jobs=list(pool.map(lambda _:q.enqueue_artifact(d,hc_cfg),range(8)))
    assert len({j['id'] for j in jobs})==1 and len(q.jobs())==1


def test_cross_process_read_lease(hc_cfg):
    import os
    import subprocess
    import sys
    d=ready(hc_cfg); locks=ArtifactLocks(hc_cfg['artifact_state_dir'])
    code="from humanoid_upload.artifacts import ArtifactLocks; import sys;\nwith ArtifactLocks(sys.argv[1]).read(sys.argv[2]):\n print('held',flush=True)\n sys.stdin.readline()"
    child=subprocess.Popen([sys.executable,'-c',code,hc_cfg['artifact_state_dir'],d['artifact_id']],
                           stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,env=os.environ.copy())
    try:
        assert child.stdout.readline().strip()=='held'
        with pytest.raises(UploadError,match='ARTIFACT_BUSY'):
            with locks.delete(d['artifact_id']): pass
    finally:
        child.communicate('\n',timeout=5)
    with locks.delete(d['artifact_id']): pass
    with pytest.raises(UploadError,match='ARTIFACT_DELETED'):
        with locks.read(d['artifact_id']): pass


def test_platform_status_separation(hc_cfg):
    svc=service(hc_cfg); hc_cfg['binding_id']=svc.binding_snapshot()['binding_id']; d=ready(hc_cfg)
    svc.scan(); job=svc.queue.jobs()[0]
    job.update(server_state='COMMITTED',state='awaiting_slice',recording_status='AWAITING_SLICE',quality_status='PASS')
    svc.queue.save(job); result=svc.jobs()[0]
    assert result['upload_state']=='submitted' and result['processing_state']=='AWAITING_SLICE'
    assert result['quality_state']=='NOT_EVALUATED'


def test_artifact_binding_cannot_be_reassigned(hc_cfg):
    d=ready(hc_cfg); d['binding_id']=None
    q=Queue(hc_cfg['queue_directory']); q.enqueue_artifact(d,hc_cfg)
    with pytest.raises(UploadError,match='TASK_BINDING_MISMATCH'):
        q.enqueue_artifact(d,dict(hc_cfg,task_id='task-2',binding_id='binding-2'))
    assert len(q.jobs())==1
