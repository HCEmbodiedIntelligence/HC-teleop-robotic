"""Consume actual B-produced ready records; never contact a platform."""
import hashlib
import os
from pathlib import Path

import pytest
from humanoid_upload.common import DEFAULTS, atomic_json, read_json
from humanoid_upload.queue import Queue
from humanoid_upload.worker import Worker
from test_hc_artifacts import OfflinePlatform


def test_b_produced_artifacts(tmp_path, monkeypatch):
    if not os.environ.get('HC_HANDOFF_READY_DIR'):
        pytest.skip('B handoff artifact directory not provided')
    state=Path(os.environ['HC_HANDOFF_READY_DIR'])
    data=Path(os.environ['HC_HANDOFF_DATA_DIR'])
    receipts=[]
    paths=sorted(state.glob('*.json'))
    assert paths, 'B handoff must contain atomic ready descriptors'
    for index,path in enumerate(paths):
        descriptor=read_json(path)
        cfg=dict(DEFAULTS,platform_base_url='http://example.invalid',credential_ref='env:NEVER_USED_IN_OFFLINE_TEST',
            robot_id='synthetic-platform-robot',local_robot_id=descriptor['profile_id'],task_id='synthetic-task',
            data_directory=str(data),queue_directory=str(tmp_path/f'queue-{index}'),
            artifact_state_dir=str(tmp_path/'locks'),binding_id=descriptor.get('binding_id') or 'synthetic-binding',
            read_bytes_per_second=1e9,bandwidth_bytes_per_second=1e9)
        if descriptor['kind']=='platform_raw':
            marker=read_json(Path(descriptor['root'])/'recording-complete.json')
            cfg.update(robot_id=marker['robot_id'],local_robot_id=marker['local_robot_id'],task_id=marker['collection_task_id'])
        queue=Queue(cfg['queue_directory']); job=queue.enqueue_artifact(descriptor,cfg)
        assert queue.enqueue_artifact(descriptor,cfg)['id']==job['id']
        api=OfflinePlatform(); api.fail_commit=True
        monkeypatch.setattr('humanoid_upload.worker.Http',lambda _cfg:api)
        Worker(queue,cfg).tick(); assert queue.jobs()[0]['state']=='retry_wait'
        count=api.puts
        restarted=Queue(cfg['queue_directory']); restarted.retry(job['id'])
        Worker(restarted,dict(cfg,task_id='changed-default')).tick()
        result=restarted.jobs()[0]
        assert result['server_state']=='COMMITTED' and result['id']==job['id']
        assert result['frozen']['task_id']==cfg['task_id'] and api.creates==1 and api.puts==count
        assert result['quality_status']=='PENDING'
        assert result['state']==('awaiting_slice' if descriptor['kind']=='platform_raw' else 'raw_committed')
        receipts.append(dict(artifact_id=descriptor['artifact_id'],kind=descriptor['kind'],
            descriptor_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),files=len(descriptor['files']),
            bytes=sum(a['size_bytes'] for a in job['manifest']['assets']),upload_state=result['state'],
            creates=api.creates,puts=api.puts,quality_status=result['quality_status'],
            transport='offline in-process mock',restart_reconciled=True))
    if os.environ.get('HC_HANDOFF_RECEIPTS'):
        atomic_json(os.environ['HC_HANDOFF_RECEIPTS'],receipts)


@pytest.mark.parametrize('delete_entry',['delete_recording','delete_artifact'])
def test_b_recorder_to_uploader_delete_lease(tmp_path,monkeypatch,delete_entry):
    """Development smoke against B's selected source; no ROS initialization or platform HTTP."""
    import threading
    import time
    from humanoid_upload.common import UploadError
    from core.upload_service import UploadService
    if not os.environ.get('HC_RECORDING_WORKTREE'):
        pytest.skip('B recording worktree not provided')
    worktree=Path(os.environ['HC_RECORDING_WORKTREE']).resolve()
    monkeypatch.syspath_prepend(str(worktree))
    from middleware.core import recording_service
    assert Path(recording_service.__file__).resolve().is_relative_to(worktree)
    recorder=recording_service.RecordingService(dict(directory=str(tmp_path/'data'),
        job_state_dir=str(tmp_path/'jobs'),artifact_state_dir=str(tmp_path/'ready'),
        min_free_bytes=1,format='mcap',max_pending_jobs=4),tmp_path,
        {'enabled':False,'subscriptions':[{'topic':'/observation/joint_states',
         'type':'sensor_msgs/msg/JointState','outputs':['record']}]},
        lambda:{'/observation/joint_states':{'state':'ok'}})
    cfg=dict(DEFAULTS,enabled=True,auto_enqueue=True,platform_base_url='https://example.invalid',
        credential_ref='env:NEVER_USED',robot_id='synthetic-robot',local_robot_id='synthetic-local',task_id='synthetic-task',
        queue_directory=str(tmp_path/'queue'),read_bytes_per_second=1e9,bandwidth_bytes_per_second=1e9)
    svc=UploadService(cfg,artifact_state_dir=recorder.artifact_state_dir,data_directory=recorder.directory,
                      request_raw_seal=recorder.request_raw_seal)
    recorder.upload_service=svc; recorder.binding_snapshot=svc.binding_snapshot
    entered=threading.Event(); release=threading.Event(); worker=None
    try:
        source=Path(recorder.start(metadata={'profile_id':'synthetic-local'}))
        recorder.record({'kind':'event','payload':{'test':'upload-delete-race'}})
        recorder.stop()
        deadline=time.monotonic()+15
        while not list(recorder.artifact_state_dir.glob('*.json')):
            assert time.monotonic()<deadline
            time.sleep(.02)
        svc.scan(); jobs=svc.queue.jobs(); assert len(jobs)==1
        job=jobs[0]; api=OfflinePlatform(); api.put_hook=lambda:(entered.set(),release.wait(10))
        monkeypatch.setattr('humanoid_upload.worker.Http',lambda _cfg:api)
        worker=threading.Thread(target=Worker(svc.queue,svc._config()).tick); worker.start()
        assert entered.wait(5)
        argument=source.name if delete_entry=='delete_recording' else job['artifact_id']
        with pytest.raises(UploadError,match='ARTIFACT_BUSY'):
            getattr(recorder,delete_entry)(argument)
        assert source.is_file() and svc.artifact_usage(job['artifact_id'])['reading']
        release.set(); worker.join(5); assert not worker.is_alive()
        assert svc.queue.jobs()[0]['state']=='cancelled'
        getattr(recorder,delete_entry)(argument)
        assert not source.exists() and svc.jobs()[0]['artifact_state']=='deleted'
        svc.queue.retry(job['id']); Worker(svc.queue,svc._config()).tick()
        assert svc.queue.jobs()[0]['code']=='ARTIFACT_DELETED'
        assert api.upload['state']!='COMMITTED'
    finally:
        release.set()
        if worker: worker.join(5)
        recorder.shutdown()
