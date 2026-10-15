"""Session inspection and annotations; source files are never edited in place."""
from contextlib import contextmanager, closing
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import uuid

from .storage import atomic_json


def session_path(root,ident):
    if not isinstance(ident,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,100}',ident):
        raise ValueError('无效 session ID')
    path=Path(root)/ident
    if path.is_symlink() or not path.is_dir():
        raise ValueError('采集目录不存在')
    return path


@contextmanager
def connection(path):
    db=sqlite3.connect('file:'+str(Path(path)/'index.sqlite3')+'?mode=ro',uri=True)
    try:
        yield db
    finally:
        db.close()


def read_json(path,default=None):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        return default


def edits(path):
    data=read_json(Path(path)/'edits.json',{'episodes':[],'invalid_intervals':[],'notes':''})
    return {'data':data,'etag':hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()}


def save_edits(path,value,etag):
    if etag!=edits(path)['etag']:
        raise ValueError('标注已被其他窗口修改，请重新读取')
    if not isinstance(value,dict) or set(value)-{'episodes','invalid_intervals','notes'}:
        raise ValueError('无效标注文档')
    manifest=read_json(Path(path)/'manifest.json',{})
    if manifest.get('state') not in {'completed','failed','interrupted'}:
        raise ValueError('请在录制结束后划分 episode')
    for key in ('episodes','invalid_intervals'):
        ranges=value.setdefault(key,[])
        if not isinstance(ranges,list) or len(ranges)>4096:
            raise ValueError('标注列表格式无效或超过 4096 个区间')
        for item in ranges:
            start,end=item.get('start_frame'),item.get('end_frame')
            if type(start) is not int or type(end) is not int or not 0<=start<end<=manifest.get('target_count',0):
                raise ValueError('标注使用 [起始帧, 结束帧)，且必须位于采集目标范围内')
            if key=='episodes' and (not isinstance(item.get('task'),str) or not item['task'].strip()):
                raise ValueError('每个 episode 必须填写任务说明')
            if key=='episodes':
                item.setdefault('source_episode_id',uuid.uuid4().hex)
                item.setdefault('outcome','unknown')
                if item['outcome'] not in {'success','failure','unknown'} or not isinstance(item['source_episode_id'],str) or not item['source_episode_id']:
                    raise ValueError('Invalid episode identity or outcome')
    value.setdefault('notes','')
    if not isinstance(value['notes'],str) or len(value['notes'])>100000:
        raise ValueError('备注格式无效或过长')
    atomic_json(Path(path)/'edits.json',value)
    return edits(path)


def bindings(db,row,manifest):
    commits={x['path'] for x in manifest.get('committed',[])}
    missing=[]
    if 'raw.mcap' not in commits:
        missing.append('raw_mcap_not_committed')
    videos={}
    for ident,group in row['images'].items():
        if not group.get('required',True):
            continue
        record=db.execute('SELECT body FROM frames WHERE source=? AND seq=?',(ident,group['source_seq'])).fetchone()
        depth=db.execute('SELECT body FROM depths WHERE source=? AND seq=?',(ident,group['source_seq'])).fetchone()
        if record is None:
            missing.append(ident+':rgb_binding_missing')
        else:
            video=json.loads(record[0])
            videos[ident]=video
            if video['video_segment_path'] not in commits or video.get('encode_status')!='encoded':
                missing.append(ident+':rgb_segment_not_committed')
        if 'depth' in group and (depth is None or 'raw.mcap' not in commits):
            missing.append(ident+':depth_not_committed')
    for ident,cloud in row.get('pointclouds',{}).items():
        if cloud.get('required'):
            found=db.execute('SELECT body FROM pointclouds WHERE source=? AND seq=?',(ident,cloud['source_seq'])).fetchone()
            if found is None or 'raw.mcap' not in commits:
                missing.append(ident+':pointcloud_not_committed')
    return videos,missing


def annotation_ranges(path,db):
    manifest=read_json(Path(path)/'manifest.json',{})
    t0=manifest.get('t0_ns') or 0
    end=manifest.get('end_ns') or 2**63-1
    cfg=read_json(Path(path)/'session.json')['config']
    period=1e9/cfg['dataset']['fps']
    active={}
    ranges=[]
    for item, in db.execute("SELECT body FROM events WHERE kind='annotation' ORDER BY id"):
        d=json.loads(item)
        stamp=d['target_time_ns']
        if d['action']=='recording':
            ranges.append((t0,end))
        elif d['action']=='point':
            ranges.append((round(stamp-period/2),round(stamp+period/2)+1))
        elif d['action']=='start':
            active[d['id']]=stamp
        elif d['action']=='end' and d['id'] in active:
            ranges.append((active.pop(d['id']),stamp))
    ranges.extend((start,end) for start in active.values())
    return ranges


def row_at(path,k,version='online'):
    manifest=read_json(Path(path)/'manifest.json',{})
    if manifest.get('recording_mode')=='raw':
        raise ValueError('原始录制请上传后在平台预览和切片，本地没有对齐帧索引')
    with connection(path) as db:
        if version=='online':
            record=db.execute('SELECT body FROM rows WHERE k=?',(k,)).fetchone()
        else:
            index=version_index(path,version)
            with closing(sqlite3.connect('file:'+str(index)+'?mode=ro',uri=True)) as alternate:
                record=alternate.execute('SELECT body FROM rows WHERE k=?',(k,)).fetchone()
        if not record:
            return {'target_frame_index':k,'valid':False,'exportable':False,'invalid_reasons':['target_not_generated_online'],'persistence_state':'missing'}
        row=json.loads(record[0])
        videos,missing=bindings(db,row,manifest)
        row.update(video_bindings=videos,persistence_reasons=missing,persistence_state='committed' if not missing else 'pending_or_failed')
        manual=[]
        if any(left<=row['target_time_ns']<right for left,right in annotation_ranges(path,db)):
            manual.append('button_marked_invalid')
        if any(item['start_frame']<=k<item['end_frame'] for item in edits(path)['data']['invalid_intervals']):
            manual.append('manually_marked_invalid')
        row['manual_invalid_reasons']=manual
        row['exportable']=row.get('export_qualified',row['strict_qualified']) and not missing and not manual
        return row


def version_index(path,version):
    if not re.fullmatch(r'offline_[0-9a-f]{12}',version):
        raise ValueError('无效对齐版本')
    index=Path(path)/'alignments'/version/'index.sqlite3'
    if not index.is_file():
        raise ValueError('对齐版本不存在')
    return index


def summary(path):
    path=Path(path)
    manifest=read_json(path/'manifest.json',{})
    info=read_json(path/'session.json',{})
    quality=read_json(path/'quality.json',{})
    result={'session_id':path.name,'directory':str(path.resolve()),'manifest':manifest,'config':info.get('config',{}),
        'metadata':info.get('metadata',{}),'binding':read_json(path/'capture-binding.json',{}),'quality':quality,'edits':edits(path),
        'alignment_versions':['online']+[p.name for p in sorted((path/'alignments').glob('offline_*')) if (p/'manifest.json').exists()]}
    result['recording_mode']=info.get('recording_mode','aligned')
    if result['recording_mode']=='raw':
        result.update(index_state='platform_managed',alignment_versions=[])
        return result
    try:
        with connection(path) as db:
            result['row_count'],result['valid_rows']=db.execute('SELECT count(*),coalesce(sum(valid),0) FROM rows').fetchone()
            result['persistence_gap_count']=db.execute('''SELECT count(*) FROM raw_samples r LEFT JOIN frames f ON r.source=f.source AND r.seq=f.seq
                WHERE r.kind='rgbd' AND f.seq IS NULL''').fetchone()[0]
            result['gaps']=[json.loads(x[0]) for x in db.execute("SELECT body FROM events WHERE kind IN ('gap','alignment_gap') ORDER BY id DESC LIMIT 100")]
    except sqlite3.OperationalError:
        result['index_state']='unavailable_or_still_starting'
    return result
