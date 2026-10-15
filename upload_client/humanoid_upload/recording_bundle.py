"""Independent validation/transport of an unsliced OpenArm recording package."""
import hashlib
from datetime import datetime
from fractions import Fraction
from pathlib import Path
import stat
from uuid import UUID

from .common import UploadError, canonical, read_json, validate, Limiter
from .bundle import checksum, finalized_mp4, safe_path

MARKER='recording-complete.json'
MAX_TOTAL_ASSETS=4096
MAX_RECORDING_ASSETS=MAX_TOTAL_ASSETS-1


def manifest_for_recording(root,cfg,client_id):
    root=Path(root).absolute()
    data=Path(cfg['data_directory'])
    if any(p.is_symlink() for p in (root,*root.parents)) or not root.resolve().is_relative_to(data):
        raise UploadError('EXPORT_OUTSIDE_DATA_DIRECTORY')
    root=root.resolve()
    if any(p.startswith('.export-') for p in root.parts):raise UploadError('EXPORT_INCOMPLETE')
    marker=read_json(root/MARKER)
    validate(marker,'openarm-recording-complete')
    raw=marker.get('schema_version')=='openarm-recording-complete/v2'
    if marker.get('schema_version') not in {'openarm-recording-complete/v1','openarm-recording-complete/v2'}:raise UploadError('SCHEMA_INVALID')
    UUID(marker['export_id'])
    command=marker['recording_upload']
    if command.get('schema_version')!=('openarm-raw-recording-upload/v1' if raw else 'continuous-recording-upload/v2'):raise UploadError('SCHEMA_INVALID')
    for key,other in (('robot_id','robot_id'),('collection_task_id','collection_task_id'),('local_robot_id','device_id')):
        if marker[key]!=command[other]:raise UploadError('IDENTITY_MISMATCH')
    if marker['robot_id']!=cfg['robot_id'] or marker['local_robot_id']!=cfg['local_robot_id']:
        raise UploadError('ROBOT_IDENTITY_MISMATCH')
    if marker['collection_task_id']!=cfg['task_id']:raise UploadError('TASK_BINDING_MISMATCH')
    start=datetime.fromisoformat(command['capture_started_at'].replace('Z','+00:00'))
    end=datetime.fromisoformat(command['capture_ended_at'].replace('Z','+00:00'))
    if not start.tzinfo or not end.tzinfo or end<=start:raise UploadError('CAPTURE_TIME_INVALID')
    entries=command['assets']
    if not 3<=len(entries)<=MAX_RECORDING_ASSETS or hashlib.sha256(canonical(entries)).hexdigest()!=marker['content_sha256']:
        raise UploadError('ASSET_INVENTORY')
    paths=[a['path'] for a in entries]
    if paths!=sorted(set(paths)):raise UploadError('ASSET_INVENTORY')
    actual=[]
    for path in root.rglob('*'):
        mode=path.lstat().st_mode
        if not (stat.S_ISREG(mode) or stat.S_ISDIR(mode)):raise UploadError('SYMLINK_OR_SPECIAL_ASSET')
        rel=path.relative_to(root).as_posix()
        safe_path(rel)
        if any(p.startswith('.export-') or p in {'FAILED.json'} or p.endswith(('.part','.tmp','.incomplete')) for p in path.parts):
            raise UploadError('EXPORT_INCOMPLETE')
        if path.is_file() and rel!=MARKER:actual.append(rel)
    if sorted(actual)!=paths:raise UploadError('ASSET_INVENTORY')
    limiter=Limiter(cfg['read_bytes_per_second'])
    assets=[]
    for index,item in enumerate(entries):
        safe_path(item['path'])
        facts=checksum(root/item['path'],limiter)
        if facts!={'size_bytes':item['size'],'sha256':item['sha256'],'crc64':str(item['crc64'])}:
            raise UploadError('ASSET_INTEGRITY')
        if item['role']=='RAW_VIDEO' or item['path'].endswith('.mp4'):finalized_mp4(root/item['path'])
        assets.append({'asset_id':f'asset-{index:03d}','path':item['path'],**facts,
            'role':item['role'],'camera_id':item.get('camera_id'),'media_type':item['media_type'],'metadata':{}})
    config_assets=[a for a in entries if a['role']=='RECORDING_CONFIG']
    if not raw and (len(config_assets)!=1 or read_json(root/config_assets[0]['path'])!=command['recording_config']):
        raise UploadError('RECORDING_CONFIG_MISMATCH')
    if raw and (not {'raw.mcap','session.json','manifest.json'}<=set(paths)
                or any(p.endswith(('.sqlite3','.db')) or p=='quality.json' for p in paths)):
        raise UploadError('RAW_INVENTORY_INVALID')
    assets.append({'asset_id':f'asset-{len(entries):03d}','path':MARKER,**checksum(root/MARKER,limiter),
                   'role':'AUXILIARY','media_type':'application/json','metadata':{}})
    cameras=[]
    duration=round((end-start).total_seconds()*1e9)
    for c in (() if raw else command['recording_config']['cameras']):
        matches=[a for a in entries if a['role']=='RAW_VIDEO' and a.get('camera_id')==c['camera_id']]
        if len(matches)!=1:raise UploadError('CAMERA_ASSET_MISSING')
        fps=Fraction(str(c['fps'])).limit_denominator(1000000)
        cameras.append({'camera_id':c['camera_id'],'path':matches[0]['path'],'codec':c['codec'],
            'width':c['width'],'height':c['height'],'clock_domain':c['clock_domain'],
            'declared_fps':{'numerator':fps.numerator,'denominator':fps.denominator},
            'declared_frame_count':round(duration*c['fps']/1e9),'declared_duration_ns':duration,
            'time_base':{'numerator':c['time_base_numerator'],'denominator':c['time_base_denominator']},
            'capture_started_at':command['capture_started_at'],'capture_ended_at':command['capture_ended_at']})
    manifest={'schema_version':'robot-ingest/v1','client_upload_id':client_id,
        'collection_task_id':command['collection_task_id'],'robot_id':command['robot_id'],
        'source_format':'CAPTURE_BUNDLE','source_format_version':'2','capture_mode':'CONTINUOUS',
        'capture_started_at':command['capture_started_at'],'capture_ended_at':command['capture_ended_at'],
        'declared_episode_count':None,'assets':assets,'cameras':cameras,
        'format_metadata':{'adapter_name':'capture_bundle','recording_id':command['recording_id'],
            'openarm_recording':{'version':2 if raw else 1,'marker_path':MARKER,'synthetic':marker['synthetic']}}}
    validate(manifest,'robot-ingest-manifest')
    return root,marker,manifest
