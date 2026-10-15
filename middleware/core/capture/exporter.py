"""Streaming LeRobot v3 export (pinned 0.4.4 layout) plus exact-depth extension."""
from contextlib import ExitStack, closing
from fractions import Fraction
import json
from pathlib import Path
import sqlite3
import struct
import time
import uuid

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from mcap.reader import make_reader
from mcap.writer import Writer, CompressionType

from . import LEROBOT_VERSION
from .catalog import connection, read_json, edits, bindings, annotation_ranges, version_index
from .storage import atomic_json, decode_depth, decode_pointcloud
from .models import freeze_json
from . import holobrain_fields as public


class Moments:
    def __init__(self):
        self.n=0

    def add(self,value):
        x=np.asarray(value,dtype=np.float64)
        if self.n==0:
            self.mean=x.copy()
            self.m2=np.zeros_like(x)
            self.low=x.copy()
            self.high=x.copy()
        self.n+=1
        delta=x-self.mean
        self.mean+=delta/self.n
        self.m2+=delta*(x-self.mean)
        self.low=np.minimum(self.low,x)
        self.high=np.maximum(self.high,x)

    def pixels(self,image,scale=255):
        channels=image.shape[2] if image.ndim==3 else 1
        values=np.asarray(image,dtype=np.float64).reshape(-1,channels)/scale
        n=len(values)
        mean=values.mean(axis=0).reshape(channels,1,1)
        m2=((values-mean.reshape(channels))**2).sum(axis=0).reshape(channels,1,1)
        low,high=values.min(axis=0).reshape(channels,1,1),values.max(axis=0).reshape(channels,1,1)
        if self.n==0:
            self.mean,self.m2,self.low,self.high=mean,m2,low,high
            self.n=n
            self.frames=1
            return
        delta=mean-self.mean
        self.mean+=delta*n/(self.n+n)
        self.m2+=m2+delta**2*self.n*n/(self.n+n)
        self.low,self.high=np.minimum(self.low,low),np.maximum(self.high,high)
        self.n+=n
        self.frames+=1

    def result(self):
        return {'min':self.low.tolist(),'max':self.high.tolist(),'mean':self.mean.tolist(),
            'std':np.sqrt(self.m2/self.n).tolist(),'count':[getattr(self,'frames',self.n)]}


class FrameReader:
    def __init__(self,root):
        self.root=Path(root)
        self.container=None
        self.path=None
        self.index=-1
        self.frame=None

    def read(self,binding,seek=False):
        path=binding['video_segment_path']
        target=binding['display_frame_index']
        if path!=self.path or target<self.index:
            self.close()
            self.container=av.open(str(self.root/path))
            self.iterator=iter(self.container.decode(video=0))
            self.path,self.index=path,-1
        expected=Fraction(binding['pts'])*Fraction(*binding['time_base'])
        if seek and target>self.index+30:
            stream=self.container.streams.video[0]
            self.container.seek(int(expected/stream.time_base),stream=stream,backward=True)
            self.iterator=iter(self.container.decode(video=0))
            for frame in self.iterator:
                stamp=frame.pts*frame.time_base
                if stamp==expected:
                    self.frame,self.index=frame,target
                    break
                if stamp>expected:
                    raise ValueError('预览索引对应的 PTS 不存在')
        while self.index<target:
            try:
                self.frame=next(self.iterator)
            except StopIteration as error:
                raise ValueError('已提交视频缺少映射帧: '+path) from error
            self.index+=1
        if abs(self.frame.pts*self.frame.time_base-expected)>Fraction(1,1000000):
            raise ValueError('视频帧 PTS 与索引不一致')
        return self.frame.to_ndarray(format='rgb24')

    def close(self):
        if self.container is not None:
            self.container.close()
        self.container=None
        self.path=None


class TrainingVideo:
    def __init__(self,path,fps,shape):
        path.parent.mkdir(parents=True,exist_ok=True)
        self.container=av.open(str(path),'w')
        self.stream=self.container.add_stream('libx264',rate=fps)
        self.stream.height,self.stream.width=shape[:2]
        self.stream.pix_fmt='yuv420p'
        self.stream.codec_context.thread_count=1
        self.stream.options={'crf':'18','preset':'veryfast','bf':'0','tune':'zerolatency'}
        self.stream.time_base=Fraction(1,fps)
        self.fps,self.frames=fps,0

    def write(self,pixels):
        frame=av.VideoFrame.from_ndarray(pixels,format='rgb24')
        frame.pts=self.frames
        frame.time_base=Fraction(1,self.fps)
        for packet in self.stream.encode(frame):
            self.container.mux(packet)
        self.frames+=1

    def close(self):
        for packet in self.stream.encode(None):
            self.container.mux(packet)
        self.container.close()


def vector(row,kind,cfg):
    if cfg.get('capture'):
        return public.vector(row,kind,cfg)
    values,names=[],[]
    for ident,source in sorted(cfg['sources'].items()):
        if source['kind']!=kind or not source['required']:
            continue
        for field,rule in sorted(source['fields'].items()):
            data=row['state'][ident][field]['value'] if kind=='state' else row['action'][ident]['payload'][field]
            flat=np.asarray(data,dtype=np.float32).reshape(-1).tolist()
            labels=rule.get('names')
            if labels and len(labels)!=len(flat):
                raise ValueError('配置字段 names 与数值维度不匹配')
            names.extend(f'{ident}.{field}.{name}' for name in (labels or range(len(flat))))
            values.extend(flat)
    return values,names


def export(path,output,version='online',progress=None,*,on_complete=None):
    path,output=Path(path),Path(output)
    manifest=read_json(path/'manifest.json',{})
    if manifest.get('state') not in {'completed','failed','interrupted'}:
        raise ValueError('录制未收尾，不能导出训练数据')
    snapshot=read_json(path/'session.json')
    cfg=snapshot['config']
    contract=bool(cfg.get('capture'))
    if not cfg['alignment']['strict']:
        raise ValueError('诊断采集不能作为严格合格训练数据导出')
    annotations=edits(path)['data']
    episodes=annotations['episodes']
    if not episodes:
        raise ValueError('请先手动标注 episode 区间与任务')
    if output.exists():
        raise ValueError('导出目录已存在，请更换名称')
    temporary=output.parent/('.export-'+uuid.uuid4().hex)
    temporary.mkdir(parents=True)
    fps=cfg['dataset']['fps']
    cameras=[ident for ident,source in cfg['sources'].items() if source['kind']=='rgbd' and source['required']]
    depth_cameras=[ident for ident in cameras if cfg['sources'][ident].get('depth_enabled',True)]
    readers={ident:FrameReader(path) for ident in cameras}
    stats={}
    features={key:{'dtype':'float32' if key=='timestamp' else 'int64','shape':[1],'names':None}
        for key in ('timestamp','frame_index','episode_index','index','task_index')}
    features['source.timestamp_ns']={'dtype':'int64','shape':[1]}
    episode_metadata=[]
    tasks=list(dict.fromkeys(e['task'] for e in episodes))
    total=0
    published=False
    try:
        if contract:
            from .completion import context_for_export, mapping, seal, sync_directory
            context=context_for_export(path,snapshot,manifest,annotations)
        with ExitStack() as stack:
            db=stack.enter_context(connection(path))
            rows_db=db if version=='online' else stack.enter_context(closing(sqlite3.connect('file:'+str(version_index(path,version))+'?mode=ro',uri=True)))
            selection=stack.enter_context(closing(sqlite3.connect(temporary/'depth_index.sqlite3')))
            selection.executescript('''CREATE TABLE cloud_selected(source TEXT,seq INTEGER,episode INTEGER,frame INTEGER,PRIMARY KEY(source,seq,episode,frame));
                CREATE TABLE cloud_frames(source TEXT,episode INTEGER,frame INTEGER,log_ns INTEGER,PRIMARY KEY(source,episode,frame));
                CREATE TABLE selected(source TEXT,seq INTEGER,episode INTEGER,frame INTEGER,PRIMARY KEY(source,seq,episode,frame));
                CREATE INDEX selected_source ON selected(source,seq);
                CREATE TABLE depth_frames(source TEXT,episode INTEGER,frame INTEGER,log_ns INTEGER,PRIMARY KEY(source,episode,frame));''')
            recorded_invalid=annotation_ranges(path,db)
            # Reject the entire export before encoding any frame; no invalid rows removed/reindexed.
            for episode in episodes:
                start,end=episode['start_frame'],episode['end_frame']
                if any(start<x['end_frame'] and end>x['start_frame'] for x in annotations['invalid_intervals']):
                    raise ValueError('episode 与手动无效区间重叠，请调整区间')
                expected=start
                for k,body in rows_db.execute('SELECT k,body FROM rows WHERE k>=? AND k<? ORDER BY k',(start,end)):
                    row=json.loads(body)
                    _,missing=bindings(db,row,manifest)
                    if k!=expected or not row.get('export_qualified',row['strict_qualified']) or missing:
                        raise ValueError(f'目标帧 {expected} 未通过质量/持久化校验: '+', '.join(row.get('invalid_reasons',[])+missing))
                    if any(left<=row['target_time_ns']<right for left,right in recorded_invalid):
                        raise ValueError(f'目标帧 {k} 已被按钮标记为无效')
                    expected+=1
                if expected!=end:
                    raise ValueError(f'目标帧 {expected} 缺少对齐记录，请调整区间或离线重新对齐')
            audit=stack.enter_context((temporary/'source_mapping.jsonl').open('w'))
            trace=stack.enter_context((temporary/'source_trace.jsonl').open('w')) if contract else None
            for ep,episode in enumerate(episodes):
                chunk,file=divmod(ep,1000)
                relative=f'chunk-{chunk:03d}/file-{file:03d}'
                parquet_path=temporary/'data'/(relative+'.parquet')
                parquet_path.parent.mkdir(parents=True,exist_ok=True)
                pqwriter=None
                videos={}
                batch=[]
                count=episode['end_frame']-episode['start_frame']
                epmeta={'episode_index':ep,'tasks':[episode['task']],'length':count,'dataset_from_index':total,
                    'dataset_to_index':total+count,'data/chunk_index':chunk,'data/file_index':file,
                    'meta/episodes/chunk_index':0,'meta/episodes/file_index':0}
                try:
                    for j,(body,) in enumerate(rows_db.execute('SELECT body FROM rows WHERE k>=? AND k<? ORDER BY k',(episode['start_frame'],episode['end_frame']))):
                        row=json.loads(body)
                        record={'timestamp':float(np.float32(j/fps)),'frame_index':j,'episode_index':ep,'index':total+j,'task_index':tasks.index(episode['task'])}
                        record['source.timestamp_ns']=public.source_timestamp(row,cfg)
                        stats.setdefault('source.timestamp_ns',Moments()).add([record['source.timestamp_ns']])
                        for kind,key in (('state','observation.state'),('action','action')):
                            values,names=vector(row,kind,cfg)
                            if not values:
                                continue
                            feature={'dtype':'float32','shape':[len(values)],'names':names}
                            if contract:
                                feature['units']=[a['unit'] for a in public.axes(cfg['capture']['profile'])]
                            if key in features and features[key]!=feature:
                                raise ValueError('采集过程中字段维度或名称发生变化，请拆分导出')
                            features[key]=feature
                            record[key]=values if contract or len(values)!=1 else values[0]
                            stats.setdefault(key,Moments()).add(values)
                        bindings_by_source,_=bindings(db,row,manifest)
                        for ident in cameras:
                            pixels=readers[ident].read(bindings_by_source[ident])
                            key=public.image_key(ident)
                            if ident not in videos:
                                videos[ident]=TrainingVideo(temporary/'videos'/key/(relative+'.mp4'),fps,pixels.shape)
                                epmeta.update({f'videos/{key}/chunk_index':chunk,f'videos/{key}/file_index':file,
                                    f'videos/{key}/from_timestamp':0.,f'videos/{key}/to_timestamp':count/fps})
                                feature={'dtype':'video','shape':list(pixels.shape),'names':['height','width','channels'],
                                    'info':{'video.height':pixels.shape[0],'video.width':pixels.shape[1],'video.codec':'h264','video.pix_fmt':'yuv420p','video.fps':fps,'video.channels':3,'is_depth_map':False,'has_audio':False}}
                                if key in features and features[key]!=feature:
                                    raise ValueError('图像尺寸或编码布局改变，请拆分导出')
                                features[key]=feature
                            videos[ident].write(pixels)
                            # Per-channel frame means for monitoring; full normalization is documented below.
                            stats.setdefault(key,Moments()).pixels(pixels)
                            selection.execute('INSERT INTO selected VALUES(?,?,?,?)',(ident,row['images'][ident]['source_seq'],ep,j))
                        for ident,cloud in row.get('pointclouds',{}).items():
                            if cloud.get('required'):
                                selection.execute('INSERT INTO cloud_selected VALUES(?,?,?,?)',(ident,cloud['source_seq'],ep,j))
                        legacy={'episode_index':ep,'frame_index':j,'target':row,'video_bindings':bindings_by_source}
                        audit.write(json.dumps(mapping(row,cfg,context,ep,j,bindings_by_source) if contract else legacy,separators=(',',':'))+'\n')
                        if trace:
                            trace.write(json.dumps(legacy,separators=(',',':'))+'\n')
                        batch.append(record)
                        if len(batch)>=256 or j==count-1:
                            table=pa.Table.from_pylist(batch)
                            # Explicit float32 timestamp matches LeRobot's declared feature schema.
                            table=table.set_column(table.schema.get_field_index('timestamp'),'timestamp',pa.array([x['timestamp'] for x in batch],type=pa.float32()))
                            for key in ('action','observation.state'):
                                if key in table.column_names:
                                    dtype=pa.list_(pa.float32()) if isinstance(batch[0][key],list) else pa.float32()
                                    table=table.set_column(table.schema.get_field_index(key),key,pa.array([x[key] for x in batch],type=dtype))
                            if pqwriter is None:
                                pqwriter=pq.ParquetWriter(parquet_path,table.schema,compression='zstd')
                            pqwriter.write_table(table)
                            batch.clear()
                            if progress:
                                progress({'state':'exporting','episode_index':ep,'frames_written':total+j+1})
                finally:
                    if pqwriter:
                        pqwriter.close()
                    for video in videos.values():
                        video.close()
                for ident in cameras:
                    key=public.image_key(ident)
                    with av.open(str(temporary/'videos'/key/(relative+'.mp4'))) as video:
                        actual=sum(1 for _ in video.decode(video=0))
                    if actual!=count:
                        raise ValueError('训练 MP4 帧数与数据行数不一致')
                total+=count
                episode_metadata.append(epmeta)
            selection.commit()
            # One sequential pass retains selected depths exactly; no uint8 visualization conversion.
            if depth_cameras:
                with (temporary/'depth.mcap').open('wb') as output_depth,(path/'raw.mcap').open('rb') as raw:
                    writer=Writer(output_depth,compression=CompressionType.ZSTD)
                    writer.start(profile='humanoid-depth-extension/1')
                    channel=writer.register_channel('/depth','humanoid-depth',0)
                    ordinal=0
                    for _,channel_in,message in make_reader(raw).iter_messages(log_time_order=False):
                        if channel_in.message_encoding!='humanoid-depth':
                            continue
                        metadata,array=decode_depth(message.data)
                        for ep,j in selection.execute('SELECT episode,frame FROM selected WHERE source=? AND seq=?',(metadata['source_id'],metadata['source_seq'])):
                            ordinal=max(ordinal+1,time.time_ns())
                            header=freeze_json({**metadata,'episode_index':ep,'frame_index':j})
                            writer.add_message(channel_id=channel,log_time=ordinal,publish_time=ordinal,
                                data=struct.pack('<I',len(header))+header+array.tobytes())
                            selection.execute('INSERT INTO depth_frames VALUES(?,?,?,?)',(metadata['source_id'],ep,j,ordinal))
                    writer.finish()
                selection.commit()
                if selection.execute('SELECT count(*) FROM depth_frames').fetchone()[0]!=total*len(depth_cameras):
                    raise ValueError('深度导出与训练行数不一致')
                from .holobrain_depth import DepthVideo, DEPTH_INFO
                depth_reader=DepthReader(temporary)
                for ep,epmeta in enumerate(episode_metadata):
                    chunk,file=divmod(ep,1000)
                    for ident in depth_cameras:
                        key=public.image_key(ident,True)
                        video=None
                        try:
                            for j in range(epmeta['length']):
                                metadata,array=depth_reader.get(ident,ep,j)
                                if video is None:
                                    video=DepthVideo(temporary/'videos'/key/f'chunk-{chunk:03d}/file-{file:03d}.mp4',fps,array.shape)
                                    feature={'dtype':'video','shape':[*array.shape[:2],1],
                                        'names':['height','width','channels'],'info':{**DEPTH_INFO,
                                        'video.height':array.shape[0],'video.width':array.shape[1],'video.fps':fps}}
                                    if key in features and features[key]!=feature:
                                        raise ValueError('深度尺寸发生变化，请拆分导出')
                                    features[key]=feature
                                video.write(array,metadata['depth']['depth_scale'])
                                stats.setdefault(key,Moments()).pixels(
                                    np.asarray(array,dtype=np.float32)*metadata['depth']['depth_scale']*1000,scale=1)
                        finally:
                            if video:video.close()
                        epmeta.update({f'videos/{key}/chunk_index':chunk,f'videos/{key}/file_index':file,
                            f'videos/{key}/from_timestamp':0.,f'videos/{key}/to_timestamp':epmeta['length']/fps})
            clouds=selection.execute('SELECT count(*) FROM cloud_selected').fetchone()[0]
            if clouds:
                with (temporary/'pointcloud.mcap').open('wb') as out,(path/'raw.mcap').open('rb') as raw:
                    writer=Writer(out,compression=CompressionType.ZSTD)
                    writer.start(profile='humanoid-pointcloud-extension/1.2')
                    channel=writer.register_channel('/pointcloud','humanoid-pointcloud',0)
                    ordinal=0
                    for _,channel_in,message in make_reader(raw).iter_messages(log_time_order=False):
                        if channel_in.message_encoding!='humanoid-pointcloud':
                            continue
                        metadata,points=decode_pointcloud(message.data)
                        for ep,j in selection.execute('SELECT episode,frame FROM cloud_selected WHERE source=? AND seq=?',(metadata['source_id'],metadata['source_seq'])):
                            ordinal=max(ordinal+1,time.time_ns())
                            header=freeze_json({**metadata,'episode_index':ep,'frame_index':j})
                            writer.add_message(channel_id=channel,log_time=ordinal,publish_time=ordinal,
                                data=struct.pack('<I',len(header))+header+points)
                            selection.execute('INSERT INTO cloud_frames VALUES(?,?,?,?)',(metadata['source_id'],ep,j,ordinal))
                    writer.finish()
                selection.commit()
                if selection.execute('SELECT count(*) FROM cloud_frames').fetchone()[0]!=clouds:
                    raise ValueError('点云导出与必需来源行映射不一致')
            (temporary/'meta/episodes/chunk-000').mkdir(parents=True)
            pq.write_table(pa.Table.from_pylist(episode_metadata),temporary/'meta/episodes/chunk-000/file-000.parquet')
            # pandas index metadata is part of the tasks.parquet contract.
            task_table=pa.table({'task_index':pa.array(range(len(tasks)),type=pa.int64()),'__index_level_0__':tasks})
            pandas_meta={'index_columns':['__index_level_0__'],'column_indexes':[{'name':None,'field_name':None,'pandas_type':'unicode','numpy_type':'object','metadata':{'encoding':'UTF-8'}}],
                'columns':[{'name':'task_index','field_name':'task_index','pandas_type':'int64','numpy_type':'int64','metadata':None},{'name':None,'field_name':'__index_level_0__','pandas_type':'unicode','numpy_type':'object','metadata':None}],
                'creator':{'library':'pyarrow','version':pa.__version__},'pandas_version':'2.2.3'}
            pq.write_table(task_table.replace_schema_metadata({b'pandas':json.dumps(pandas_meta).encode()}),temporary/'meta/tasks.parquet')
            atomic_json(temporary/'meta/info.json',{'codebase_version':'v3.0','robot_type':context['profile']['robot_type'] if contract else snapshot['metadata'].get('robot_id','humanoid'),
                'recording_field_schema':public.SCHEMA,
                'total_episodes':len(episodes),'total_frames':total,'total_tasks':len(tasks),'chunks_size':1000,
                'data_files_size_in_mb':100,'video_files_size_in_mb':500,'fps':fps,'splits':{'train':f'0:{len(episodes)}'},
                'data_path':'data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet',
                'video_path':'videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4' if cameras else None,'features':features})
            atomic_json(temporary/'meta/stats.json',{k:v.result() for k,v in stats.items()})
            atomic_json(temporary/'meta/recording-fields.json',public.description(cfg))
            atomic_json(temporary/'export.json',{'session_id':path.name,'lerobot_version':LEROBOT_VERSION,'format':'v3.0',
                'alignment_version':version,'camera_validation':cfg['alignment'].get('camera_validation','strict'),'exposure_validation':('metadata' if any(s.get('require_exposure_metadata') for s in cfg['sources'].values()) else 'disabled') if cfg['alignment'].get('camera_validation') in {'driver_headers','timestamps'} else 'strict','episodes':episodes,'total_frames':total,
                'source_mapping':'source_mapping.jsonl','action_definition':'active_at_target, actual sent command; send_time approximation explicitly retained',
                'depth':{'format':'humanoid-depth-extension/1','reader':'middleware.core.capture.exporter.DepthReader',
                         'native_lerobot_rgb_video':False,'native_lerobot_depth_video':True,
                         'video_encoding':'hevc/gray12le','exact_source_values':'depth.mcap'},
                'pointcloud':{'format':'humanoid-pointcloud-extension/1.2','reader':'middleware.core.capture.exporter.PointCloudReader','selected_frames':clouds,'native_lerobot_field':False},
                'image_stats':'streaming per-channel pixel statistics over selected RGB frames',
                'video_policy':'selected source frames reencoded at dataset.fps; raw session unchanged'})
            if contract:
                atomic_json(temporary/'capture-context.json',context)
        if contract:
            marker=seal(temporary,context)
        temporary.rename(output)
        published=True
        if contract:
            sync_directory(output.parent)
        result={'path':str(output),'frames':total,'episodes':len(episodes),'lerobot_version':LEROBOT_VERSION}
        if contract:
            result['completion']={'root':str(output.resolve()),'marker_path':str((output/'export-complete.json').resolve()),
                                  'export_id':marker['export_id'],'contract_version':marker['contract_version'],
                                  'content_sha256':marker['content_sha256']}
    except BaseException:
        # Retain failed artifacts for diagnosis, never expose as a complete dataset.
        if published:
            output.rename(temporary)
        (temporary/'export-complete.json').unlink(missing_ok=True)
        atomic_json(temporary/'FAILED.json',{'complete':False,'message':'导出未完成；原始采集未修改'})
        raise
    finally:
        for reader in readers.values():
            reader.close()
    if contract and on_complete is not None:
        try:
            on_complete(dict(result['completion']))
        except Exception as error:
            # A sealed package is recoverable from its marker after D restarts.
            # A subscriber failure cannot turn committed bytes into FAILED data.
            result['notification_error']=str(error)
    return result


class DepthReader:
    """Explicit companion reader; returns exact source dtype, values and metadata."""
    def __init__(self,root):
        self.root=Path(root)

    def get(self,source,episode_index,frame_index):
        with closing(sqlite3.connect('file:'+str(self.root/'depth_index.sqlite3')+'?mode=ro',uri=True)) as db:
            result=db.execute('SELECT log_ns FROM depth_frames WHERE source=? AND episode=? AND frame=?',(source,episode_index,frame_index)).fetchone()
        if result is None:
            raise KeyError((source,episode_index,frame_index))
        with (self.root/'depth.mcap').open('rb') as f:
            for _,_,message in make_reader(f).iter_messages(start_time=result[0],end_time=result[0]+1):
                return decode_depth(message.data)
        raise ValueError('深度索引存在，但 MCAP 原始距离消息缺失')


class PointCloudReader:
    """Exact PointCloud2 bytes and layout, paired by exported episode/frame."""
    def __init__(self,root):
        self.root=Path(root)

    def get(self,source,episode_index,frame_index):
        with closing(sqlite3.connect('file:'+str(self.root/'depth_index.sqlite3')+'?mode=ro',uri=True)) as db:
            result=db.execute('SELECT log_ns FROM cloud_frames WHERE source=? AND episode=? AND frame=?',(source,episode_index,frame_index)).fetchone()
        if result is None:
            raise KeyError((source,episode_index,frame_index))
        with (self.root/'pointcloud.mcap').open('rb') as f:
            for _,_,message in make_reader(f).iter_messages(start_time=result[0],end_time=result[0]+1):
                return decode_pointcloud(message.data)
        raise ValueError('Committed pointcloud index is missing its MCAP message')
