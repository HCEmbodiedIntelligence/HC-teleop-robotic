"""Transport contracts only; real MCAP/QC acceptance is in E's platform evidence."""
import hashlib
import shutil
from uuid import uuid4

import pytest

from pathlib import Path
from humanoid_upload.bundle import checksum
from humanoid_upload.cli import scan, exports
from humanoid_upload.common import UploadError, atomic_json, canonical, read_json
from humanoid_upload.worker import Worker


@pytest.fixture
def recording(tmp_path,cfg):
    root=tmp_path/'data/recording'
    root.mkdir(parents=True)
    cfg['data_directory']=str(root.parent)
    shutil.copyfile(Path(__file__).parent/'fixtures/openarm-raw-v1/videos/camera_left/000000.mp4',root/'front.mp4')
    (root/'sensors.mcap').write_bytes(b'protocol-only-test-fixture')
    config={'schema_version':'recording-config/v1','recorder_version':'openarm-session/v2',
        'primary_clock_domain':'recording_grid','cameras':[{'camera_id':'front','topic':'/camera/front/image_raw',
        'fps':30,'width':640,'height':480,'codec':'h264','clock_domain':'recording_grid',
        'time_base_numerator':1,'time_base_denominator':15360,'capture_start_offset_ns':'0'}],
        'sensors':[{'topic':topic,'clock_domain':'recording_grid','timestamp_mode':'RECORDING_OFFSET_NS','required':True}
            for topic in ('/action','/observation/state','/openarm/capture_validity')]}
    atomic_json(root/'recording-config.json',config)
    roles={'front.mp4':('RAW_VIDEO','front','video/mp4'),'sensors.mcap':('SENSOR_DATA',None,'application/mcap'),
           'recording-config.json':('RECORDING_CONFIG',None,'application/json')}
    assets=[]
    for path in sorted(root.iterdir()):
        facts=checksum(path)
        role,camera,media=roles[path.name]
        assets.append({'path':path.name,'role':role,'camera_id':camera,'media_type':media,'size':facts['size_bytes'],
            'sha256':facts['sha256'],'crc64':int(facts['crc64']),'part_count':1})
    command={'schema_version':'continuous-recording-upload/v2','recording_id':'recording-test',
        'rollout_id':'capture-test','data_package_id':'capture-test','collection_job_id':'capture-test',
        'collection_task_id':cfg['task_id'],'robot_id':cfg['robot_id'],'device_id':cfg['local_robot_id'],
        'capture_started_at':'2026-09-23T00:00:00Z','capture_ended_at':'2026-09-23T00:00:01Z',
        'recording_config':config,'assets':assets}
    marker={'schema_version':'openarm-recording-complete/v1','export_id':str(uuid4()),'session_id':'test',
        'robot_id':cfg['robot_id'],'local_robot_id':cfg['local_robot_id'],'collection_task_id':cfg['task_id'],
        'synthetic':True,'content_sha256':hashlib.sha256(canonical(assets)).hexdigest(),'recording_upload':command}
    atomic_json(root/'recording-complete.json',marker)
    return root


def test_manual_policy_keeps_local_package_until_explicit_enqueue(recording,cfg,queue):
    cfg['auto_enqueue']=False
    assert scan(queue,cfg)==[] and queue.jobs()==[]
    assert exports(cfg)[0]['root']==str(recording)
    job=queue.enqueue(recording,cfg)
    assert job['manifest']['capture_mode']=='CONTINUOUS'
    assert job['manifest']['declared_episode_count'] is None
    assert queue.enqueue(recording,cfg)['id']==job['id']
    cfg['auto_enqueue']=True
    assert scan(queue,cfg)==[]


@pytest.mark.parametrize('mutation',['tamper','symlink','schema','task'])
def test_invalid_or_rebound_recording_is_never_enqueued(recording,cfg,queue,mutation):
    if mutation=='tamper':(recording/'sensors.mcap').write_bytes(b'tampered')
    elif mutation=='symlink':(recording/'alias').symlink_to(recording/'sensors.mcap')
    elif mutation=='task':cfg['task_id']='different-task'
    else:
        marker=read_json(recording/'recording-complete.json')
        marker['recording_upload']['assets'][0]['role']='TRAINING_DATA'
        atomic_json(recording/'recording-complete.json',marker)
    with pytest.raises(UploadError):queue.enqueue(recording,cfg)
    assert queue.jobs()==[]


def test_recording_receipt_is_not_training_ready(recording,cfg,queue):
    job=queue.enqueue(recording,cfg)
    job.update(upload_id='upload',raw_source_id='raw')
    class API:
        def api(self,*args):
            return {'upload_id':'upload','raw_source_id':'raw','status':'AWAITING_SLICE','terminal':True,
                    'recording_id':'recording-test','page_path':'/recordings/recording-test/slice','poll_after_seconds':0}
    Worker(queue,cfg).processing(job,API(),'/uploads/upload')
    assert queue.jobs()[0]['state']=='awaiting_slice'
    assert queue.jobs()[0]['episodes']==[]
    class InvalidAPI(API):
        def api(self,*args):return {**super().api(*args),'terminal':False}
    with pytest.raises(UploadError,match='PROCESSING_RESULT_INCONSISTENT'):
        Worker(queue,cfg).processing(job,InvalidAPI(),'/uploads/upload')


def test_confirmed_slices_stay_terminal_and_can_be_explicitly_refreshed(recording,cfg,queue):
    job=queue.enqueue(recording,cfg)
    job.update(upload_id='upload',raw_source_id='raw',server_state='COMMITTED')
    class API:
        def api(self,*args):
            return {'upload_id':'upload','raw_source_id':'raw','status':'SLICED','terminal':True,
                    'recording_id':'recording-test','page_path':'/recordings/recording-test/slice','poll_after_seconds':0}
    Worker(queue,cfg).processing(job,API(),'/uploads/upload')
    saved=queue.jobs()[0]
    assert saved['state']=='sliced' and saved['recording_status']=='SLICED'
    assert saved['terminal'] and saved['episodes']==[] and queue.jobs(due=True)==[]
    frozen=saved['frozen'].copy()
    refreshed=queue.retry(job['id'])
    assert not refreshed['terminal'] and refreshed['upload_id']=='upload'
    assert refreshed['raw_source_id']=='raw' and refreshed['frozen']==frozen
    assert 'processing_retry_request' not in refreshed


@pytest.mark.parametrize('raw',[False,True])
def test_more_than_256_assets_preserve_complete_inventory(recording,cfg,queue,tmp_path,raw):
    if raw:
        from pathlib import Path
        root=tmp_path/'data/raw-capacity'
        shutil.copytree(Path(__file__).parent/'fixtures/openarm-raw-v1',root)
        recording=root
        marker=read_json(root/'recording-complete.json')
        cfg.update(robot_id=marker['robot_id'],local_robot_id=marker['local_robot_id'],task_id=marker['collection_task_id'])
    marker=read_json(recording/'recording-complete.json')
    original_count=len(marker['recording_upload']['assets'])
    for index in range(300):
        path=recording/f'aux-{index:04d}.bin'
        path.write_bytes(f'software-capacity-{index}'.encode())
        facts=checksum(path)
        marker['recording_upload']['assets'].append({
            'path':path.name,'role':'AUXILIARY','camera_id':None,
            'media_type':'application/octet-stream','size':facts['size_bytes'],
            'sha256':facts['sha256'],'crc64':int(facts['crc64']),'part_count':1})
    marker['recording_upload']['assets'].sort(key=lambda a:a['path'])
    marker['content_sha256']=hashlib.sha256(canonical(marker['recording_upload']['assets'])).hexdigest()
    atomic_json(recording/'recording-complete.json',marker)
    job=queue.enqueue(recording,cfg)
    assert len(job['manifest']['assets'])==original_count+301  # Includes the completion marker.
    assert len({a['path'] for a in job['manifest']['assets']})==original_count+301
    assert queue.enqueue(recording,cfg)['id']==job['id']
    for a in job['manifest']['assets']:
        assert checksum(recording/a['path'])['sha256']==a['sha256']


@pytest.mark.parametrize('case,code',[
    ('old','RECORDING_CAPACITY_UNSUPPORTED'),('limited','RECORDING_CAPACITY_EXCEEDED'),
    ('wrong_robot','UPLOAD_CAPABILITIES_INVALID'),('invalid','UPLOAD_CAPABILITIES_INVALID'),
    ('allowed',None),
])
def test_large_upload_requires_matching_server_capability_before_create(case,code,cfg,monkeypatch):
    calls=[]
    class API:
        def __init__(self,*args):pass
        def api(self,method,path='',payload=None):
            calls.append((method,path))
            assert (method,path)==('GET',':capabilities')
            if case=='old':raise UploadError('HTTP_404')
            return {'schema_version':'robot-ingest-capabilities/v1',
                'robot_id':'foreign' if case=='wrong_robot' else cfg['robot_id'],
                'max_total_assets':256 if case=='limited' else 4096,
                'max_recording_assets':True if case=='invalid' else 255 if case=='limited' else 4095}
    monkeypatch.setattr('humanoid_upload.worker.Http',API)
    worker=Worker(None,cfg)
    job={'manifest':{'source_format':'CAPTURE_BUNDLE','assets':[{}]*304,'robot_id':cfg['robot_id']},
         'frozen':{},'upload_id':None}
    if code:
        with pytest.raises(UploadError,match=code):worker.process(job)
    else:
        worker.recording_capacity(job,API())
    assert calls==[('GET',':capabilities')]


def test_failed_preparation_retries_only_processing(recording,cfg,queue):
    job=queue.enqueue(recording,cfg)
    job.update(upload_id='upload',raw_source_id='raw')
    class API:
        calls=[]
        def api(self,*args):
            self.calls.append(args)
            return {'upload_id':'upload','raw_source_id':'raw','status':'FAILED','terminal':True,
                    'recording_id':None,'page_path':None,'poll_after_seconds':0,
                    'error_code':'RAW_RECORDING_PREPARATION_FAILED'}
    api=API()
    Worker(queue,cfg).processing(job,api,'/uploads/upload')
    assert queue.jobs()[0]['state']=='processing_failed'
    request_id=str(uuid4())
    job['processing_retry_request']=request_id
    Worker(queue,cfg).processing(job,api,'/uploads/upload')
    assert api.calls[-1]==('POST','/uploads/upload/recording:retry',{'request_id':request_id})
    assert job['upload_id']=='upload' and job['raw_source_id']=='raw'
    assert 'processing_retry_request' not in job


def test_raw_v2_transports_only_original_files(tmp_path,cfg,queue):
    # Actual A-generated fixture, not an emulated recorder.
    from pathlib import Path
    fixture=Path(__file__).parent/'fixtures/openarm-raw-v1'
    assert fixture.exists()
    root=tmp_path/'data/raw'
    shutil.copytree(fixture,root)
    cfg['data_directory']=str(root.parent)
    marker=read_json(root/'recording-complete.json')
    cfg.update(robot_id=marker['robot_id'],local_robot_id=marker['local_robot_id'],task_id=marker['collection_task_id'])
    job=queue.enqueue(root,cfg)
    assert job['manifest']['source_format']=='CAPTURE_BUNDLE'
    assert job['manifest']['format_metadata']['openarm_recording']['version']==2
    assert not any(a['path'].endswith(('.sqlite3','quality.json')) for a in job['manifest']['assets'])
    assert queue.enqueue(root,cfg)['id']==job['id']
