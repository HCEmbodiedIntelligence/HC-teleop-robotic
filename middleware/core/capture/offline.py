"""Re-associate retained samples into a new version; cannot repair physical timing."""
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import time
import uuid

from mcap.writer import Writer, CompressionType

from . import ALIGNMENT_VERSION
from .alignment import Aligner, target_time, target_count
from .buffers import TimeCache
from .catalog import connection, read_json
from .models import Sample, freeze_json
from .storage import atomic_json


def realign(path,progress=None):
    path=Path(path)
    manifest=read_json(path/'manifest.json',{})
    if manifest.get('state') not in {'completed','failed','interrupted'} or manifest.get('t0_ns') is None:
        raise ValueError('需要已结束且具有目标时间轴的 session')
    if 'raw.mcap' not in {x['path'] for x in manifest.get('committed',[])}:
        raise ValueError('原始 MCAP 尚未提交；请先恢复完整可读块')
    cfg=read_json(path/'session.json')['config']
    version='offline_'+uuid.uuid4().hex[:12]
    directory=path/'alignments'/version
    directory.mkdir(parents=True)
    cache=TimeCache(cfg['buffers'])
    aligner=Aligner(cfg,cache,ALIGNMENT_VERSION+'+'+version)
    t0,end=manifest['t0_ns'],manifest['end_ns']
    count=target_count(t0,end,cfg['dataset']['fps'])
    # Disk-backed source index avoids reading an hour of data into memory.
    with connection(path) as db,closing(sqlite3.connect(directory/'index.sqlite3')) as dest,(directory/'aligned.mcap').open('xb') as f:
        dest.execute('CREATE TABLE rows(k INTEGER PRIMARY KEY,target_ns INTEGER,valid INTEGER,version TEXT,body TEXT)')
        writer=Writer(f,compression=CompressionType.ZSTD)
        writer.start(profile='humanoid-offline-alignment/1.1')
        channel=writer.register_channel('/humanoid/session/aligned','json',0)
        timeline=db.execute("SELECT body FROM events WHERE kind='timeline' ORDER BY id LIMIT 1").fetchone()
        if timeline:
            aligner.trigger_origins=json.loads(timeline[0]).get('trigger_origins',{})
        has_action_index=any(x[1]=='action_ns' for x in db.execute('PRAGMA table_info(raw_samples)'))
        valid=0
        for k in range(count):
            target=target_time(t0,k,cfg['dataset']['fps'])
            retention=round(cfg['buffers']['retention_ms']*1e6)
            cache=TimeCache(cfg['buffers'])
            aligner.cache=cache
            for ident,source in cfg['sources'].items():
                if source['kind']=='event':
                    continue
                queries=[('SELECT body FROM raw_samples WHERE source=? AND capture_ns>=? AND capture_ns<=? ORDER BY capture_ns,seq',(ident,target-retention,target+retention)),
                    ('SELECT body FROM raw_samples WHERE source=? AND capture_ns<? ORDER BY capture_ns DESC,seq DESC LIMIT 1',(ident,target-retention))]
                if 'named_axes' in source:
                    # Bounded time window plus each axis's last termination and
                    # predecessor. Never scan the complete recording per frame.
                    queries=[('SELECT body FROM raw_samples WHERE source=? AND capture_ns>=? AND capture_ns<=? ORDER BY capture_ns,seq',
                              (ident,target-retention,target+retention))]
                    for axis in source['named_axes']:
                        for is_event in (False,True):
                            queries.append(('SELECT body FROM raw_samples WHERE source=? AND capture_ns<? '
                                'AND EXISTS(SELECT 1 FROM json_each(body,\'$.joint_names\') WHERE value=?) '
                                'AND (json_extract(body,\'$.validity_event\') IS NOT NULL)=? '
                                'ORDER BY capture_ns DESC,seq DESC LIMIT 1',
                                (ident,target-retention,axis['name'],int(is_event))))
                elif source['kind']=='action':
                    time_column='action_ns' if has_action_index else "coalesce(json_extract(body,'$.effective_time_ns'),json_extract(body,'$.send_time_ns'))"
                    queries=[(f'SELECT body FROM raw_samples WHERE source=? AND {time_column}<=? ORDER BY {time_column} DESC,seq DESC LIMIT 1',(ident,target))]
                for sql,args in queries:
                    for body, in db.execute(sql,args):
                        d=json.loads(body)
                        sample=Sample(ident,d['source_seq'],d['capture_time_ns'],d['receive_time_ns'],d['arrival_time_ns'],d['clock_epoch'],body.encode())
                        cache.add(sample,target)
            row=aligner.row(k,t0,time.monotonic_ns(),deadline=2**63-1)
            row['online_deadline_ns']=target+round(cfg['alignment']['delay_ms']*1e6)
            row['association_policy']='offline_including_late_arrivals'
            payload=freeze_json(row)
            stamp=time.time_ns()
            writer.add_message(channel_id=channel,log_time=stamp,publish_time=stamp,data=payload)
            dest.execute('INSERT INTO rows VALUES(?,?,?,?,?)',(k,target,int(row['valid']),aligner.version,payload.decode()))
            valid+=int(row['valid'])
            if k%256==0:
                dest.commit()
                if progress:
                    progress({'state':'realigning','targets_processed':k+1,'target_count':count})
        writer.finish()
        f.flush()
        import os
        os.fsync(f.fileno())
        dest.commit()
    atomic_json(directory/'manifest.json',{'version':version,'alignment_version':aligner.version,'rows':count,
        'valid_rows':valid,'committed':['aligned.mcap','index.sqlite3'],'source_session':path.name,
        'policy':'new version; late samples allowed; all physical exposure and clock limits unchanged'})
    return {'version':version,'rows':count,'valid_rows':valid}
