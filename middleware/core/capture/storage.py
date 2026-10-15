"""Single-owner MCAP/SQLite writer, per-camera MP4 encoders, explicit commits."""
from collections import Counter
from fractions import Fraction
import json
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import threading
import time

import numpy as np
from mcap.writer import Writer, CompressionType

from . import SCHEMA_VERSION
from .buffers import ByteQueue
from .media_spool import MediaSpool
from .models import freeze_json


def atomic_json(path,value,sync=True):
    path=Path(path)
    temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,allow_nan=False,indent=2)
        f.flush()
        if sync:
            os.fsync(f.fileno())
    os.replace(temporary,path)
    if sync:
        fd=os.open(path.parent,os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


def decode_depth(data):
    size=struct.unpack('<I',data[:4])[0]
    metadata=json.loads(data[4:4+size])
    array=np.frombuffer(data[4+size:],dtype=metadata['depth']['dtype']).reshape(metadata['depth']['shape'])
    return metadata,array


def decode_pointcloud(data):
    size=struct.unpack('<I',data[:4])[0]
    return json.loads(data[4:4+size]),data[4+size:]


class Storage:
    def __init__(self,path,cfg,fail,*,index=True,defer_media=False,offline=False,compact_media=False,depth_video=None):
        self.path,self.cfg,self.fail=Path(path),cfg,fail
        self.index=index
        self.defer_media=defer_media
        self.compact_media=compact_media
        self.depth_video=cfg['storage'].get('depth_video_encoding',False) if depth_video is None else depth_video
        self.offline=offline
        b=cfg['buffers']
        self.queue=ByteQueue(b['writer_items'],b['writer_bytes'])
        self.thread=threading.Thread(target=self.run,name='session-mcap-writer',daemon=True)
        self.ready=threading.Event()
        self.error=None
        self.counts=Counter()
        self.committed=[]
        self.closed=False

    def start(self):
        self.thread.start()
        if not self.ready.wait(5) or self.error:
            raise RuntimeError(self.error or 'MCAP writer 启动超时')

    def submit(self,kind,data,raw=b''):
        encoded=freeze_json(data)
        return self.queue.offer((kind,encoded,raw),len(encoded)+len(raw)+128)

    def required(self,kind,data,raw=b''):
        # Background-only callers. Never used in device callbacks.
        until=time.monotonic()+2
        while not self.submit(kind,data,raw):
            if self.error or self.queue.closed or (not self.offline and time.monotonic()>until):
                self.fail('writer_queue_overflow:'+kind)
                return False
            time.sleep(.005)
        return True

    def run(self):
        stream=db=None
        try:
            stream=(self.path/'raw.mcap').open('xb')
            # Live capture writes original pixels directly; CPU compression and
            # compatibility encodings run only in the post-recording job.
            compression=CompressionType.NONE if self.defer_media else CompressionType.ZSTD
            writer=Writer(stream,compression=compression,chunk_size=1024*1024,enable_crcs=True,enable_data_crcs=True)
            writer.start(profile=SCHEMA_VERSION,library='humanoid_manager')
            schema=writer.register_schema(name=SCHEMA_VERSION,encoding='jsonschema',data=freeze_json({'type':'object'}))
            depth_schema=writer.register_schema(name=SCHEMA_VERSION+'/depth',encoding='jsonschema',data=freeze_json({
                'description':'uint32 little-endian JSON byte length, UTF-8 metadata JSON, C-order lossless depth bytes; dtype/shape in metadata.depth'}))
            cloud_schema=writer.register_schema(name=SCHEMA_VERSION+'/pointcloud',encoding='jsonschema',data=freeze_json({
                'description':'uint32 LE JSON length, UTF-8 metadata, exact PointCloud2.data; layout in pointcloud_layout'}))
            rgb_schema=writer.register_schema(name=SCHEMA_VERSION+'/rgb',encoding='jsonschema',data=freeze_json({
                'description':'uint32 LE JSON length, UTF-8 metadata, original uint8 RGB pixels; shape in metadata.rgb'}))
            channels={}
            from .holobrain_mcap import PublicWriter
            from .holobrain_fields import description
            public_writer=None if self.defer_media or self.compact_media else PublicWriter(writer,self.cfg)
            fields=description(self.cfg)
            if self.compact_media:
                fields['storage_contract']={'schema_version':'compact-mp4/v1' if self.depth_video else 'rgb-mp4-raw-depth/v1',
                    'rgb':'h264/yuv420p','depth':'hevc/gray12le' if self.depth_video else 'humanoid-depth/source-bytes',
                    'depth_quantized':self.depth_video,
                    'raw_pixel_copies':False,'ros_image_compatibility_topics':False,
                    'depth_reference_topic':('/humanoid/session/depth_video_frame/<source_id>' if self.depth_video
                                             else '/humanoid/session/depth/<source_id>'),
                    'rgb_reference_topic':'/humanoid/session/video_frame/<source_id>'}
            atomic_json(self.path/'recording-fields.json',fields)
            if self.index:
                db=sqlite3.connect(self.path/'index.sqlite3')
                db.executescript('''
                PRAGMA journal_mode=WAL;
                CREATE TABLE events(id INTEGER PRIMARY KEY,kind TEXT,body TEXT);
                CREATE TABLE rows(k INTEGER PRIMARY KEY,target_ns INTEGER,valid INTEGER,version TEXT,body TEXT);
                CREATE TABLE frames(source TEXT,seq INTEGER,body TEXT,PRIMARY KEY(source,seq));
                CREATE TABLE pointclouds(source TEXT,seq INTEGER,log_ns INTEGER,body TEXT,PRIMARY KEY(source,seq));
                CREATE TABLE depths(source TEXT,seq INTEGER,log_ns INTEGER,body TEXT,PRIMARY KEY(source,seq));
                CREATE TABLE commits(path TEXT PRIMARY KEY,body TEXT);
                CREATE TABLE raw_samples(source TEXT,seq INTEGER,capture_ns INTEGER,arrival_ns INTEGER,kind TEXT,body TEXT,action_ns INTEGER,clock_epoch INTEGER,source_epoch TEXT,PRIMARY KEY(source,clock_epoch,source_epoch,seq));
                CREATE INDEX raw_capture ON raw_samples(source,capture_ns,seq);
                CREATE INDEX raw_action ON raw_samples(source,action_ns,seq);
            ''')
            self.ready.set()
            last_flush=time.monotonic()
            while not self.queue.closed or self.queue.items:
                item=self.queue.get()
                if item is not None:
                    kind,encoded,raw=item
                    d=json.loads(encoded)
                    topic='/humanoid/session/'+kind+('/'+d['source_id'] if 'source_id' in d else '')
                    is_depth=kind=='depth'
                    is_cloud=kind=='pointcloud'
                    is_rgb=kind=='rgb'
                    if topic not in channels:
                        encoding,ident=(('humanoid-depth',depth_schema) if is_depth else
                            ('humanoid-pointcloud',cloud_schema) if is_cloud else
                            ('humanoid-rgb',rgb_schema) if is_rgb else ('json',schema))
                        channels[topic]=writer.register_channel(topic=topic,message_encoding=encoding,schema_id=ident)
                    log_ns=time.time_ns()
                    if public_writer is not None:public_writer.append(kind,d,raw,log_ns)
                    # Publication wall timestamp only when actually supplied, never capture_time_ns.
                    publish_ns=d.get('publish_time_unix_ns') or log_ns
                    payload=struct.pack('<I',len(encoded))+encoded+raw if is_depth or is_cloud or is_rgb else encoded
                    writer.add_message(channel_id=channels[topic],log_time=log_ns,publish_time=publish_ns,
                        sequence=self.counts[topic]%(2**32),data=payload)
                    self.counts[topic]+=1
                    text=encoded.decode()
                    if db is None:
                        pass
                    elif kind=='aligned':
                        db.execute('INSERT INTO rows VALUES(?,?,?,?,?)',(d['target_frame_index'],d['target_time_ns'],int(d['valid']),d['alignment_version'],text))
                    elif kind=='video_frame':
                        db.execute('INSERT INTO frames VALUES(?,?,?)',(d['source_id'],d['source_seq'],text))
                    elif kind=='depth':
                        db.execute('INSERT INTO depths VALUES(?,?,?,?)',(d['source_id'],d['source_seq'],log_ns,text))
                    elif kind=='pointcloud':
                        db.execute('INSERT INTO pointclouds VALUES(?,?,?,?)',(d['source_id'],d['source_seq'],log_ns,text))
                    elif kind=='raw':
                        db.execute('INSERT INTO raw_samples VALUES(?,?,?,?,?,?,?,?,?)',(d['source_id'],d['source_seq'],d['capture_time_ns'],d['arrival_time_ns'],d['kind'],text,d.get(d.get('action_time_field','')),d['clock_epoch'],str(d.get('source_epoch',''))))
                    else:
                        db.execute('INSERT INTO events(kind,body) VALUES(?,?)',(kind,text))
                    if kind=='commit':
                        if db is not None:
                            db.execute('INSERT INTO commits VALUES(?,?)',(d['path'],text))
                        self.committed.append(d)
                        manifest=json.loads((self.path/'manifest.json').read_text())
                        manifest['committed']=list(self.committed)
                        atomic_json(self.path/'manifest.json',manifest,self.cfg['storage']['fsync_on_commit'])
                now=time.monotonic()
                if now-last_flush>=self.cfg['storage']['numeric_flush_seconds']:
                    if db is not None:db.commit()
                    stream.flush()  # Flush != power-loss durability; only closed commits qualify.
                    if shutil.disk_usage(self.path).free<self.cfg['storage']['minimum_free_bytes']:
                        raise OSError('磁盘可用空间低于配置下限')
                    last_flush=now
            writer.finish()
            stream.flush()
            if self.cfg['storage']['fsync_on_commit']:
                os.fsync(stream.fileno())
            stream.close()
            stream=None
            commit={'path':'raw.mcap','kind':'mcap','closed':True,'fsynced':self.cfg['storage']['fsync_on_commit']}
            if db is not None:
                db.execute('INSERT INTO commits VALUES(?,?)',('raw.mcap',freeze_json(commit).decode()))
                db.commit()
                db.execute('PRAGMA wal_checkpoint(TRUNCATE)')
            self.committed.append(commit)
            self.committed.append({'path':'recording-fields.json','kind':'public_schema',
                                   'closed':True,'fsynced':True})
            self.closed=True
        except Exception as error:
            self.error=f'{type(error).__name__}: {error}'
            self.fail('storage_failure:'+self.error)
        finally:
            self.ready.set()
            if stream:
                stream.close()
            if db:
                db.close()

    def stop(self):
        self.queue.close()
        self.thread.join(None if self.offline else 30)
        if self.thread.is_alive():
            self.fail('writer_close_timeout')

    def status(self):
        return {'queue':self.queue.status(),'messages':sum(self.counts.values()),'error':self.error,
            'closed':self.closed,'committed_files':len(self.committed)}


class VideoEncoder:
    def __init__(self,ident,cfg,storage,fail):
        self.av=__import__('av')
        self.ident,self.cfg,self.storage,self.fail=ident,cfg,storage,fail
        self.source=cfg['sources'][ident]
        divisor=2 if storage.compact_media and self.source.get('depth_enabled',True) else 1
        self.queue=(MediaSpool(storage.path/'.media-spool'/(ident+'-rgb.frames'),
                               sync=cfg['storage']['fsync_on_commit'],
                               reserve_bytes=cfg['storage']['minimum_free_bytes']) if storage.compact_media else
                    ByteQueue(cfg['buffers']['video_frames_per_camera'],cfg['buffers']['video_bytes_per_camera']//divisor))
        self.thread=threading.Thread(target=self.run,name='mp4-'+ident,daemon=True)
        self.encoded=0
        self.error=None
        self.thread.start()

    def submit(self,sample):
        # References immutable RGB bytes; depth bytes are not retained in the encoder queue.
        d=sample.data()
        return self.queue.offer((d,sample.rgb_pixels),len(sample.document)+len(sample.rgb_pixels)+128)

    def run(self):
        av=self.av
        container=None
        pending={}
        segment=-1
        display=0
        first_capture=None
        shape=None
        stream=None
        rate=Fraction(str(self.source['fps']))

        def mux(packets):
            for packet in packets:
                original=pending.pop(packet.pts,None)
                if original is None:
                    raise RuntimeError('编码器返回无法绑定的 PTS')
                pts=packet.pts
                time_base=packet.time_base
                container.mux(packet)
                data,index=original
                binding={'source_id':self.ident,'source_seq':data['source_seq'],
                    'rgb_source_seq':data['rgb']['source_seq'],'source_capture_time_ns':data['rgb']['exposure_midpoint_ns'],
                    'pair_seq':data['pair_seq'],'video_segment_path':relative,'display_frame_index':index,
                    'pts':pts,'time_base':[time_base.numerator,time_base.denominator],
                    'encode_status':'encoded','pts_policy':'new_frames_in_arrival_order_at_declared_camera_fps'}
                if not self.storage.required('video_frame',binding):
                    raise RuntimeError('视频帧索引未能持久化')
                self.encoded+=1

        def close():
            nonlocal container
            if container is None:
                return
            mux(stream.encode(None))
            if pending:
                raise RuntimeError('编码器未返回所有帧')
            container.close()
            container=None
            path=self.storage.path/relative
            with av.open(str(path)) as check:
                if sum(1 for _ in check.decode(video=0)) != display:
                    raise ValueError('RGB MP4 decoded frame count mismatch')
            if self.cfg['storage']['fsync_on_commit']:
                with path.open('rb') as f:
                    os.fsync(f.fileno())
                fd=os.open(path.parent,os.O_DIRECTORY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            self.storage.required('commit',{'path':relative,'kind':'rgb_mp4','closed':True,
                'fsynced':self.cfg['storage']['fsync_on_commit'],'source_id':self.ident,'frames':display})

        try:
            while not self.queue.closed or self.queue.items:
                item=self.queue.get()
                if item is None:
                    continue
                d,pixels=item
                new_shape=tuple(d['rgb']['shape'])
                if container is not None and (d['capture_time_ns']-first_capture>=self.cfg['storage']['video_segment_seconds']*1e9 or new_shape!=shape):
                    close()
                if container is None:
                    segment+=1
                    relative=f'videos/{self.ident}/{segment:06d}.mp4'
                    path=self.storage.path/relative
                    path.parent.mkdir(parents=True,exist_ok=True)
                    container=av.open(str(path),'w',format='mp4')
                    stream=container.add_stream('libx264',rate=rate)
                    shape=new_shape
                    stream.height,stream.width=shape[:2]
                    if stream.width%2 or stream.height%2:
                        raise ValueError('RGB 编码尺寸必须为偶数')
                    stream.pix_fmt='yuv420p'
                    stream.codec_context.thread_count=self.cfg['storage']['encoder_threads']
                    stream.options={'crf':'18','preset':'veryfast','tune':'zerolatency','bf':'0'}
                    stream.time_base=1/rate
                    first_capture=d['capture_time_ns']
                    display=0
                rgb=np.frombuffer(pixels,np.uint8).reshape(shape)
                from .holobrain_mcap import jpeg
                if not self.storage.compact_media and not self.storage.required('public_rgb',d,jpeg(rgb)):
                    raise RuntimeError('公共 RGB MCAP 写盘队列已满')
                frame=av.VideoFrame.from_ndarray(rgb,format='rgb24')
                frame.pts=display
                frame.time_base=1/rate
                pending[display]=(d,display)
                display+=1
                mux(stream.encode(frame))
            close()
        except Exception as error:
            self.error=f'{type(error).__name__}: {error}'
            self.fail('encoder_failure:'+self.ident+':'+self.error)
            if container is not None:
                try:
                    container.close()
                except Exception:
                    pass

    def stop(self):
        self.queue.close()
        self.thread.join(None if self.storage.offline or self.storage.compact_media else 30)
        if self.thread.is_alive():
            self.fail('encoder_close_timeout:'+self.ident)

    def status(self):
        return {'queue':self.queue.status(),'encoded':self.encoded,'error':self.error}
