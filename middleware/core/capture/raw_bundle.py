"""Seal captures; deferred recordings encode media only after recording stops."""
from datetime import datetime, timezone, timedelta
import hashlib
import os
from pathlib import Path, PurePosixPath
import shutil
import uuid

from .catalog import read_json
from .completion import inventory, canonical, sync_directory
from .storage import atomic_json

MARKER='recording-complete.json'


def prepare(path,output,progress=None):
    path,output=Path(path),Path(output)
    snapshot=read_json(path/'session.json',{})
    manifest=read_json(path/'manifest.json',{})
    if snapshot.get('recording_mode')!='raw' or manifest.get('state')!='completed':
        raise ValueError('原始上传需要正常完成的原始录制')
    context=dict(snapshot.get('metadata',{}).get('capture_context',{}))
    context.update(read_json(path/'capture-binding.json',{}))
    if not context.get('robot_id') or not context.get('collection_task_id'):
        raise ValueError('请先明确绑定平台机器人和任务')
    identity=uuid.uuid5(uuid.NAMESPACE_URL,':'.join(('raw-v1',context['robot_id'],context['collection_task_id'],
        context['local_robot_id'],snapshot['session_id'],snapshot['config_hash'])))
    existing=read_json(output/MARKER,{})
    if existing:
        if existing['session_id']!=snapshot['session_id'] or existing['export_id']!=str(identity):
            raise ValueError('录制输出目录身份冲突')
        return {'path':str(output),'completion':{'root':str(output.resolve()),'marker_path':str((output/MARKER).resolve())},'recovered':True}
    if output.exists():raise ValueError('输出目录已存在但未封存')
    if manifest['end_ns']<=manifest['t0_ns']:raise ValueError('空录制不能上传')
    paths={item['path'] for item in manifest['committed']}
    if 'raw.mcap' not in paths:raise ValueError('原始 MCAP 尚未完成')
    paths.update({'session.json','manifest.json'})
    if (path/'capture-binding.json').exists():paths.add('capture-binding.json')
    if len(paths)+1>4096:
        raise ValueError('原始录制超过 4096 个总上传资产（含完成标记）；原件保留，请显式分段处理')
    for relative in paths:
        parts=PurePosixPath(relative)
        source=path/relative
        if (parts.is_absolute() or '..' in parts.parts or '\\' in relative or ':' in relative
                or source.is_symlink() or not source.resolve().is_relative_to(path.resolve())
                or not source.is_file() or source.suffix in {'.sqlite3','.db'}
                or source.name=='quality.json'):
            raise ValueError('原始资产缺失或路径无效')
    temporary=output.parent/('.export-'+uuid.uuid4().hex)
    temporary.mkdir(parents=True)
    renamed=False
    try:
        deferred=snapshot.get('raw_storage_format')=='deferred-rgbd/v1'
        if deferred:
            from .deferred import materialize
            result=materialize(path,temporary,snapshot,manifest,progress)
            if len(result['committed'])+len(paths-{'raw.mcap','recording-fields.json','manifest.json'})+1>4096:
                raise ValueError('编码后超过 4096 个上传资产；原始录制保留')
        for relative in sorted(paths):
            source=path/relative
            destination=temporary/relative
            destination.parent.mkdir(parents=True,exist_ok=True)
            if deferred and relative=='originals.mcap':
                from .originals import compress_archive
                compress_archive(source,destination)
            elif not deferred or relative not in {'raw.mcap','recording-fields.json','manifest.json'}:
                shutil.copyfile(source,destination)
            if progress:progress({'state':'packaging','file':relative})
        assets=[]
        for item in inventory(temporary):
            media=('video/mp4' if item['path'].endswith('.mp4') else
                   'application/mcap' if item['path'].endswith('.mcap') else 'application/json')
            assets.append({'path':item['path'],'role':'AUXILIARY','camera_id':None,'media_type':media,
                'size':item['size_bytes'],'sha256':item['sha256'],'crc64':int(item['crc64']),'part_count':1})
        start=datetime.fromtimestamp(snapshot['started_unix_ns']/1e9,timezone.utc)
        end=start+timedelta(microseconds=max(1,round((manifest['end_ns']-manifest['t0_ns'])/1000)))
        command={'schema_version':'openarm-raw-recording-upload/v1','recording_id':'recording-'+identity.hex,
            'rollout_id':'capture-'+identity.hex,'data_package_id':'capture-'+identity.hex,
            'collection_task_id':context['collection_task_id'],'collection_job_id':'capture-'+identity.hex,
            'robot_id':context['robot_id'],'device_id':context['local_robot_id'],
            'capture_started_at':start.isoformat(),'capture_ended_at':end.isoformat(),'assets':assets}
        marker={'schema_version':'openarm-recording-complete/v2','export_id':str(identity),
            'session_id':snapshot['session_id'],'robot_id':context['robot_id'],
            'local_robot_id':context['local_robot_id'],'collection_task_id':context['collection_task_id'],
            'synthetic':context.get('synthetic',False),'content_sha256':hashlib.sha256(canonical(assets)).hexdigest(),
            'recording_upload':command}
        atomic_json(temporary/MARKER,marker)
        for file in temporary.rglob('*'):
            if file.is_file():
                with file.open('rb') as stream:os.fsync(stream.fileno())
        for directory in sorted((p for p in temporary.rglob('*') if p.is_dir()),reverse=True):sync_directory(directory)
        sync_directory(temporary)
        temporary.rename(output)
        renamed=True
        sync_directory(output.parent)
        return {'path':str(output),'mode':'raw','completion':{'root':str(output.resolve()),'marker_path':str((output/MARKER).resolve())}}
    except BaseException:
        if renamed:
            (output/MARKER).unlink(missing_ok=True)
        if temporary.exists():
            (temporary/MARKER).unlink(missing_ok=True)
            atomic_json(temporary/'FAILED.json',{'complete':False,'message':'原始录制保留；封存失败'})
        raise
