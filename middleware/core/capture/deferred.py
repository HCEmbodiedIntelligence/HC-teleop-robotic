"""CPU-only post-recording encoding into compact RGB/depth MP4 uploads."""
from collections import Counter
import json
from pathlib import Path
import struct
import time

from mcap.reader import make_reader

from .models import RGBDSample, freeze_json
from .storage import Storage, VideoEncoder, atomic_json
from .compact_depth import CompactDepth

FORMAT='deferred-rgbd/v1'


def materialize(source, output, snapshot, manifest, progress=None):
    """Write a compatible upload bundle without changing the captured files."""
    source,output=Path(source),Path(output)
    errors=[]
    storage=Storage(output,snapshot['config'],errors.append,index=False,offline=True,compact_media=True,depth_video=True)
    encoders={}
    depths={}
    rgb_counts=Counter()
    atomic_json(output/'manifest.json',{**manifest,'state':'processing','committed':[]})
    processed=0
    try:
        storage.start()
        with (source/'raw.mcap').open('rb') as stream:
            for _,channel,message in make_reader(stream,validate_crcs=True).iter_messages(log_time_order=False):
                if not channel.topic.startswith('/humanoid/session/'):
                    raise ValueError('原始缓存含有未知消息，保留原件并停止封装')
                kind=channel.topic.removeprefix('/humanoid/session/').split('/')[0]
                binary=channel.message_encoding in {'humanoid-rgb','humanoid-depth','humanoid-pointcloud'}
                if binary:
                    size=struct.unpack('<I',message.data[:4])[0]
                    data=json.loads(message.data[4:4+size])
                    raw=message.data[4+size:]
                elif channel.message_encoding=='json':
                    data=json.loads(message.data)
                    raw=b''
                else:
                    raise ValueError('原始缓存消息编码不受支持')
                # Only MP4s carry image pixels; MCAP keeps source facts and refs.
                if kind=='depth':
                    ident=data['source_id']
                    if ident not in depths:
                        depths[ident]=CompactDepth(output,ident,snapshot['config']['dataset']['fps'],data['depth']['shape'])
                    data=depths[ident].write(data,raw)
                    kind,raw='depth_video_frame',b''
                if kind!='rgb' and not storage.required(kind,data,raw):
                    raise RuntimeError('; '.join(errors) or '后台写盘失败')
                if kind=='rgb':
                    ident=data['source_id']
                    rgb_counts[ident]+=1
                    shape=data['rgb']['shape']
                    if len(shape)!=3 or shape[2]!=3 or len(raw)!=shape[0]*shape[1]*3:
                        raise ValueError('原始 RGB 像素大小与记录尺寸不一致')
                    if ident not in encoders:
                        encoders[ident]=VideoEncoder(ident,snapshot['config'],storage,errors.append)
                    sample=RGBDSample(source_id=ident,source_seq=data['source_seq'],
                        capture_time_ns=data['capture_time_ns'],receive_time_ns=data['receive_time_ns'],
                        arrival_time_ns=data['arrival_time_ns'],clock_epoch=data['clock_epoch'],
                        document=freeze_json(data),rgb_pixels=raw,depth_values=b'')
                    # Offline work may wait for bounded encoder capacity; it
                    # must never drop a captured frame to catch up with a clock.
                    while not encoders[ident].submit(sample):
                        if errors or encoders[ident].error or storage.error:
                            raise RuntimeError('; '.join(errors) or '后台编码失败')
                        time.sleep(.005)
                    processed+=1
                    if progress and processed%30==0:
                        progress({'state':'encoding','frames':processed})
        for encoder in encoders.values():encoder.stop()
        for ident,depth in depths.items():
            depth.close()
            storage.committed.append({'path':depth.relative,'kind':'depth_mp4',
                'source_id':ident,'frames':depth.frames,'closed':True,'fsynced':False})
        storage.stop()
        if errors or storage.error or not storage.closed:
            raise RuntimeError('; '.join(errors) or storage.error or '后台文件未完成')
        if not encoders and any(s['kind']=='rgbd' and s['required'] for s in snapshot['config']['sources'].values()):
            raise ValueError('录制中没有保存必需的 RGB 原始帧')
        encoded={ident:encoder.encoded for ident,encoder in encoders.items()}
        expected=dict(rgb_counts)
        if encoded!=expected:
            raise ValueError('后台编码帧数与原始录制不一致')
        originals=[item for item in manifest['committed'] if item.get('kind')=='original_ros']
        result={**manifest,'committed':list(storage.committed)+originals,
                'postprocessing':{'schema_version':'deferred-recording/v1','source_format':FORMAT,
                                  'processor':'cpu','original_pixels_preserved':False,'encoded_frames':encoded,
                                  'media_storage':'compact-mp4/v1','depth_encoding':'hevc/gray12le',
                                  'depth_profile':'hc-depth/1','depth_codes_verified':True}}
        atomic_json(output/'manifest.json',result)
        return result
    finally:
        for depth in depths.values():
            depth.video.close()
        for encoder in encoders.values():
            if encoder.thread.is_alive():encoder.stop()
        if storage.thread.is_alive():storage.stop()
