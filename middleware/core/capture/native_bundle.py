"""Completed, unsliced A session -> platform recording/v2 (no training episodes).

The original session is archived byte-for-byte. Preview MP4s use the recorded
alignment grid; missing images are explicit black placeholders, never valid
training frames. The validity stream gates platform slice QC.
"""
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import tarfile
import uuid

import av
import numpy as np
from mcap.writer import Writer, CompressionType

from .catalog import read_json, connection, bindings, annotation_ranges
from .completion import inventory, canonical, sync_directory
from .exporter import FrameReader, TrainingVideo
from .storage import atomic_json, decode_depth
from . import holobrain_fields as public
from .holobrain_depth import DepthVideo
from mcap.reader import make_reader

MARKER='recording-complete.json'
VALIDITY_TOPIC='/openarm/capture_validity'


def prepare(path, output, progress=None):
    path,output=Path(path),Path(output)
    manifest=read_json(path/'manifest.json',{})
    snapshot=read_json(path/'session.json',{})
    if manifest.get('state')!='completed' or not snapshot.get('config',{}).get('capture'):
        raise ValueError('整段上传需要正常收尾的 OpenArm 录制；原始异常数据保留本地')
    context=dict(snapshot['metadata']['capture_context'])
    if not context.get('robot_id') or not context.get('collection_task_id'):
        context.update(read_json(path/'capture-binding.json',{}))
    if not context.get('robot_id') or not context.get('collection_task_id'):
        raise ValueError('请先为离线录制明确绑定平台机器人和任务')
    existing=read_json(output/MARKER,{})
    if existing:
        if existing.get('session_id')!=snapshot['session_id']:
            raise ValueError('录制输出目录身份冲突')
        return {'path':str(output),'completion':{'root':str(output.resolve()),
                'marker_path':str((output/MARKER).resolve())},'recovered':True}
    if output.exists():
        raise ValueError('整段录制输出目录已存在但未封存')
    cfg=snapshot['config']
    profile=context['profile']
    fps=cfg['dataset']['fps']
    if fps != 30:
        raise ValueError('OpenArm 整段上传 v2 需要 30 Hz 对齐网格')
    count=manifest['target_count']
    if count<=0:
        raise ValueError('录制没有目标帧')
    temporary=output.parent/('.export-'+uuid.uuid4().hex)
    temporary.mkdir(parents=True)
    roles={}
    readers={}
    videos={}
    public_axes=public.axes(profile)
    names=[a['name'] for a in public_axes]
    units=[a['unit'] for a in public_axes]
    cameras=[c for c in profile['cameras']]
    topics=('/observation/state','/action',VALIDITY_TOPIC,'source.timestamp_ns')
    try:
        (temporary/'sensors').mkdir()
        (temporary/'source').mkdir()
        # Archive only committed files and the closed index/context, never active files.
        paths={item['path'] for item in manifest['committed']}
        paths.update({'session.json','manifest.json','quality.json','index.sqlite3'})
        if (path/'capture-binding.json').exists():paths.add('capture-binding.json')
        with tarfile.open(temporary/'source/session.tar','w') as archive:
            for relative in sorted(paths):
                source=path/relative
                if source.is_symlink() or not source.resolve().is_relative_to(path.resolve()) or not source.is_file():
                    raise ValueError('原始录制资产缺失或路径不安全')
                archive.add(source,arcname=relative,recursive=False)
        roles['source/session.tar']=('AUXILIARY',None,'application/x-tar')
        invalid_count=0
        with connection(path) as db, (temporary/'sensors/robot.mcap').open('wb') as stream, (temporary/'source/frames.jsonl').open('w') as trace:
            writer=Writer(stream,compression=CompressionType.ZSTD,enable_crcs=True,enable_data_crcs=True)
            writer.start(profile='openarm-recording-sensors/v1')
            schema=writer.register_schema('openarm-recording-value/v1','jsonschema',b'{}')
            channels={topic:writer.register_channel(topic,'json',schema) for topic in topics}
            recorded_invalid=annotation_ranges(path,db)
            rows=iter(db.execute('SELECT k,body FROM rows ORDER BY k'))
            current=next(rows,None)
            for k in range(count):
                row=json.loads(current[1]) if current and current[0]==k else None
                if row is not None:current=next(rows,None)
                offset=k*1_000_000_000//fps
                reasons=list(row.get('invalid_reasons',[])) if row else ['target_not_generated']
                qualified=bool(row and row.get('export_qualified',row.get('strict_qualified',False)))
                video_bindings,missing=bindings(db,row,manifest) if row else ({},[])
                reasons.extend(missing)
                if row and any(a<=row['target_time_ns']<b for a,b in recorded_invalid):
                    reasons.append('operator_marked_invalid');qualified=False
                try:
                    source_stamp=public.source_timestamp(row,cfg) if row else None
                except (KeyError,ValueError):
                    source_stamp=None
                    qualified=False;reasons.append('primary_rgb_timestamp_missing')
                values={'source.timestamp_ns':source_stamp}
                for kind,topic in (('state',topics[0]),('action',topics[1])):
                    from .completion import components
                    parts=components(row,kind,cfg) if row else []
                    if row:
                        by_name=dict(zip((a['name'] for a in profile['axes']),parts))
                        parts=[by_name[a['source_name']] for a in public_axes]
                    valid=len(parts)==len(names) and all(p['valid'] for p in parts)
                    values[topic]={'values':[float(np.float32(p['values'])) if p['valid'] else None for p in parts],
                                   'names':names,'units':units}
                    if not valid:qualified=False;reasons.append(kind+'_invalid')
                for camera in cameras:
                    ident=camera['id']
                    if ident not in videos:
                        videos[ident]=TrainingVideo(temporary/f'videos/{ident}.mp4',fps,(camera['height'],camera['width'],3))
                        readers[ident]=FrameReader(path)
                    binding=video_bindings.get(ident)
                    try:
                        if binding is None:raise ValueError('missing image')
                        pixels=readers[ident].read(binding)
                    except (ValueError,OSError):
                        pixels=np.zeros((camera['height'],camera['width'],3),dtype=np.uint8)
                        reasons.append(ident+':preview_placeholder');qualified=False
                    videos[ident].write(pixels)
                    depth_id=ident+'_depth'
                    if depth_id not in videos:
                        videos[depth_id]=DepthVideo(temporary/f'videos/{depth_id}.mp4',fps,(camera['height'],camera['width']))
                    image=row.get('images',{}).get(ident) if row else None
                    selected=db.execute('SELECT log_ns FROM depths WHERE source=? AND seq=?',
                        (ident,image['source_seq'])).fetchone() if image else None
                    depth=np.zeros((camera['height'],camera['width']),dtype=np.uint16)
                    scale=.001
                    if selected:
                        with (path/'raw.mcap').open('rb') as raw:
                            for _,channel,message in make_reader(raw).iter_messages(start_time=selected[0],end_time=selected[0]+1):
                                if channel.message_encoding=='humanoid-depth':
                                    metadata,depth=decode_depth(message.data)
                                    scale=metadata['depth']['depth_scale']
                                    break
                            else:raise ValueError('Committed source depth missing')
                    else:
                        qualified=False;reasons.append(ident+':depth_not_committed')
                    videos[depth_id].write(depth,scale)
                qualified=qualified and not missing
                invalid_count+=int(not qualified)
                values[VALIDITY_TOPIC]={'valid':qualified,'source_frame':k,'reasons':sorted(set(reasons))}
                for topic,value in values.items():
                    writer.add_message(channels[topic],log_time=offset,publish_time=offset,sequence=k,data=canonical(value))
                trace.write(json.dumps({'source_frame':k,'offset_ns':offset,'row':row,'valid':qualified},separators=(',',':'))+'\n')
                if progress and k%30==0:progress({'state':'packaging','frames_written':k+1})
            writer.finish()
        for video in videos.values():video.close()
        videos.clear()
        camera_config=[]
        for camera in cameras:
            ident=camera['id']
            relative=f'videos/{ident}.mp4'
            with av.open(str(temporary/relative)) as container:
                base=container.streams.video[0].time_base
            for depth in (False,True):
                camera_id=public.camera_name(ident)+('_depth' if depth else '')
                relative=f'videos/{ident}'+('_depth' if depth else '')+'.mp4'
                with av.open(str(temporary/relative)) as container:
                    base=container.streams.video[0].time_base
                camera_config.append({'camera_id':camera_id,'topic':public.image_key(ident,depth),'fps':fps,
                    'width':camera['width'],'height':camera['height'],'codec':'hevc' if depth else 'h264','clock_domain':'recording_grid',
                    'time_base_numerator':base.numerator,'time_base_denominator':base.denominator,
                    'capture_start_offset_ns':'0'})
                roles[relative]=('RAW_VIDEO',camera_id,'video/mp4')
        recording_config={'schema_version':'recording-config/v1','recorder_version':'openarm-session/v2',
            'primary_clock_domain':'recording_grid','cameras':camera_config,
            'sensors':[{'topic':topic,'clock_domain':'recording_grid','timestamp_mode':'RECORDING_OFFSET_NS','required':True} for topic in topics]}
        atomic_json(temporary/'recording-config.json',recording_config)
        roles['recording-config.json']=('RECORDING_CONFIG',None,'application/json')
        roles['sensors/robot.mcap']=('SENSOR_DATA',None,'application/mcap')
        roles['source/frames.jsonl']=('AUXILIARY',None,'application/x-ndjson')
        atomic_json(temporary/'capture-provenance.json',{'schema_version':'openarm-recording-provenance/v1',
            'capture_context':context,'session_id':snapshot['session_id'],'frame_count':count,'invalid_frame_count':invalid_count,
            'source_t0_ns':manifest['t0_ns'],'source_end_ns':manifest['end_ns'],
            'preview_policy':'fixed_grid; missing images are black and explicitly invalid; original bytes in source/session.tar',
            'axis_names':names,'axis_units':units})
        roles['capture-provenance.json']=('AUXILIARY',None,'application/json')
        assets=[]
        for asset in inventory(temporary):
            role,camera_id,media=roles[asset['path']]
            assets.append({'path':asset['path'],'role':role,'camera_id':camera_id,'media_type':media,
                'size':asset['size_bytes'],'sha256':asset['sha256'],'crc64':int(asset['crc64']),'part_count':1})
        identity=uuid.uuid5(uuid.NAMESPACE_URL,':'.join((context['robot_id'],context['collection_task_id'],
            context['local_robot_id'],snapshot['session_id'],snapshot['config_hash'])))
        start=datetime.fromtimestamp(snapshot['started_unix_ns']/1e9,timezone.utc)
        end=start+timedelta(microseconds=round(count*1_000_000/fps))
        command={'schema_version':'continuous-recording-upload/v2','recording_id':'recording-'+identity.hex,
            'rollout_id':'capture-'+identity.hex,'data_package_id':'capture-'+identity.hex,
            'collection_task_id':context['collection_task_id'],'collection_job_id':'capture-'+identity.hex,
            'robot_id':context['robot_id'],'device_id':context['local_robot_id'],
            'capture_started_at':start.isoformat(),'capture_ended_at':end.isoformat(),
            'recording_config':recording_config,'assets':assets}
        marker={'schema_version':'openarm-recording-complete/v1','export_id':str(identity),
            'session_id':snapshot['session_id'],'robot_id':context['robot_id'],
            'local_robot_id':context['local_robot_id'],'collection_task_id':context['collection_task_id'],
            'synthetic':context.get('synthetic',False),'content_sha256':hashlib.sha256(canonical(assets)).hexdigest(),
            'recording_upload':command}
        atomic_json(temporary/MARKER,marker)
        for item in temporary.rglob('*'):
            if item.is_file():
                import os
                with item.open('rb') as stream:os.fsync(stream.fileno())
        sync_directory(temporary)
        temporary.rename(output)
        sync_directory(output.parent)
        return {'path':str(output),'frames':count,'invalid_frames':invalid_count,
            'completion':{'root':str(output.resolve()),'marker_path':str((output/MARKER).resolve())}}
    except BaseException:
        if temporary.exists():
            (temporary/MARKER).unlink(missing_ok=True)
            atomic_json(temporary/'FAILED.json',{'complete':False,'message':'原始录制保留；整段打包失败'})
        raise
    finally:
        for reader in readers.values():reader.close()
        for video in videos.values():
            try:video.close()
            except Exception:pass
