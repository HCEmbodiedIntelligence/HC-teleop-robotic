"""Bounded continuous acquisition. Faults stop recording, never command the robot."""
from collections import Counter, deque
from dataclasses import replace
import copy
import json
import os
from pathlib import Path
import resource
import threading
import time
import uuid

from .adapters import preflight
from .alignment import Aligner, image_quality, target_time, target_count
from .buffers import ByteQueue, TimeCache
from .config import validate, config_hash
from .models import normalize, RGBDSample, PointCloudSample, CONTRACT
from .clocks import RosSteadyMapping, RosClockJump
from .storage import Storage, VideoEncoder, atomic_json


class Distribution:
    """Bounded reservoir, exact max/count; quantiles explicitly approximate."""
    def __init__(self):
        self.values=[]
        self.count=0
        self.maximum=None

    def add(self,value):
        if value is None:
            return
        self.count+=1
        self.maximum=value if self.maximum is None else max(self.maximum,value)
        if len(self.values)<4096:
            self.values.append(value)
        else:
            index=((self.count*1103515245+12345)&0x7fffffff)%self.count
            if index<4096:
                self.values[index]=value

    def summary(self):
        values=sorted(self.values)
        return {'count':self.count,'max':self.maximum,'quantiles':'bounded_reservoir_approximation',
            **{key:values[round((len(values)-1)*p)] if values else None for key,p in [('p50',.5),('p95',.95),('p99',.99)]}}


class Session:
    def __init__(self,root,config,clocks,reports,metadata=None,session_id=None,*,raw_only=False):
        self.raw_only=raw_only
        self.cfg=validate(config)
        self.clocks=clocks
        self.ros_time=self.cfg['clock']['domain']=='ros'
        self.scheduler=RosSteadyMapping(round(self.cfg['clock']['jump_tolerance_ms']*1e6))
        self.scheduler.observe(time.time_ns(),time.monotonic_ns())
        self.clock_failed=False
        self.reports=copy.deepcopy(reports)
        check=preflight(self.cfg,reports,clocks)
        if not check['ready']:
            raise ValueError('启动检查失败：'+'；'.join(x['source_id']+': '+x['reason'] for x in check['issues']))
        self.id=session_id or time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.path=Path(root)/self.id
        self.path.mkdir(parents=True,exist_ok=False)
        self.metadata=copy.deepcopy(metadata or {})
        if self.cfg['capture']:
            from .profile import freeze_context
            self.metadata['capture_context']=freeze_context(self.cfg,self.metadata.get('capture_context',{}))
        self.ingress=ByteQueue(self.cfg['buffers']['ingress_items'],self.cfg['buffers']['ingress_bytes'])
        self.cache=None if raw_only else TimeCache(self.cfg['buffers'])
        self.aligner=None if raw_only else Aligner(self.cfg,self.cache)
        from .next_state_action import NextStateActions
        self.next_state_actions=(NextStateActions(self.cfg) if raw_only and self.cfg['capture']
            and self.cfg['alignment'].get('derive_action_from_next_state',False) else None)
        compact=raw_only and not self.cfg['storage'].get('retain_original_ros',False)
        self.storage=Storage(self.path,self.cfg,self.fail,index=not raw_only,
            defer_media=raw_only and not compact,compact_media=compact)
        from .originals import OriginalArchive
        self.originals=(OriginalArchive(self.path,self.cfg,self.fail) if raw_only and
            self.cfg['storage'].get('retain_original_ros',False) and
            any(s.get('original_topics') for s in self.cfg['sources'].values()) else None)
        self.encoders={}
        self.state='starting'
        self.errors=deque(maxlen=100)
        self.counters=Counter()
        self.source_counts=Counter()
        self.highest_seq={}
        self.stream_sequences={}
        self.button_seq=0
        self.copy_slots=threading.BoundedSemaphore(1)
        self.recent_seq={}
        self.sequence_order={}
        self.last_clock_sample={}
        self.source_ready=set()
        self.t0=None
        self.end=None
        self.stop_requested=False
        self.k=0
        self.started_ns=time.monotonic_ns()
        self.stop_event=threading.Event()
        self.closed=threading.Event()
        self.metrics={}
        self.latest_rows=deque(maxlen=32)
        self.active_marker=None
        self.annotation_lock=threading.Lock()
        self.thread=threading.Thread(target=self.run,name='session-aligner',daemon=True)

    def data_now(self):
        return self.scheduler.to_ros(time.monotonic_ns()) if self.ros_time else time.monotonic_ns()

    def deadline_steady(self,ros_ns):
        return self.scheduler.to_steady(ros_ns) if self.ros_time else ros_ns

    def observe_ros_clock(self,ros_ns,steady_ns):
        if not self.ros_time or self.clock_failed:
            return
        try:
            self.scheduler.observe(ros_ns,steady_ns)
        except RosClockJump as error:
            # End this segment explicitly. A subsequent start creates a fresh session/epoch.
            self.clock_failed=True
            self.errors.append({'reason':str(error),'new_clock_epoch':self.scheduler.epoch,'time_ns':steady_ns})
            self.end=(self.scheduler.ros_ns if self.raw_only else
                target_time(self.t0,self.k,self.cfg['dataset']['fps']) if self.t0 is not None else 0)
            self.stop_event.set()
            self.storage.submit('clock_epoch_end',{'reason':str(error),'new_epoch':self.scheduler.epoch,
                'first_unaligned_target':self.k,'receive_steady_ns':steady_ns,'observed_ros_ns':ros_ns})

    def metric(self,name,value):
        self.metrics.setdefault(name,Distribution()).add(value)

    def start(self):
        try:
            return self._start()
        except Exception as error:
            self.fail('startup_failure:'+str(error))
            self.state='failed'
            for encoder in self.encoders.values():
                encoder.stop()
            if self.storage.thread.is_alive():
                self.storage.stop()
            if self.originals:self.originals.stop()
            self.closed.set()
            try:
                atomic_json(self.path/'manifest.json',{'session_id':self.id,'state':'failed','committed':self.storage.committed,'errors':list(self.errors)})
            except OSError:
                pass
            raise

    def _start(self):
        snapshot={'session_id':self.id,'config':self.cfg,'config_hash':config_hash(self.cfg),
            'recording_mode':'raw' if self.raw_only else 'aligned',
            'metadata':self.metadata,'capability_reports':self.reports,
            'clock_models':[vars(m) for m in self.clocks.models.values()],
            'capture_clock_id':self.cfg['clock']['domain'],'ros_to_steady':{'ros_ns':self.scheduler.ros_ns,'steady_ns':self.scheduler.steady_ns,'epoch':self.scheduler.epoch},
            'started_monotonic_ns':self.started_ns,'started_unix_ns':time.time_ns(),
            'host_boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
        if self.raw_only:
            snapshot['raw_storage_format']=(('compact-mp4/v1' if self.storage.depth_video else 'rgb-mp4-raw-depth/v1')
                                            if self.storage.compact_media else 'deferred-rgbd/v1')
        if self.next_state_actions:snapshot['action_fallback']=self.next_state_actions.status()
        if self.originals:
            snapshot['original_capture']={'path':'originals.mcap','boundary':'driver_ros_subscription',
                'association_required':False,'topics':self.originals.topics}
        atomic_json(self.path/'session.json',snapshot)
        atomic_json(self.path/'manifest.json',{'session_id':self.id,'state':'recording','committed':[]})
        self.storage.start()
        if self.originals:self.originals.start()
        self.storage.required('session',snapshot)
        for ident,source in self.cfg['sources'].items():
            if source['kind']=='rgbd' and (not self.raw_only or self.storage.compact_media):
                self.encoders[ident]=VideoEncoder(ident,self.cfg,self.storage,self.fail)
                if self.raw_only and self.storage.depth_video and source.get('depth_enabled',True):
                    from .compact_depth import DepthEncoder
                    self.encoders[ident+'_depth']=DepthEncoder(ident,self.cfg,self.storage,self.fail)
        self.state='recording' if self.raw_only else 'warming_up'
        if self.raw_only:
            self.t0=self.data_now()
            duration=self.cfg['dataset'].get('duration_seconds')
            if duration is not None:self.end=self.t0+round(duration*1e9)
            self.storage.required('recording_start',{'capture_time_ns':self.t0})
        self.thread.start()
        return self.status()

    def record_original(self,topic,data,receive_ns):
        """Persist independent ROS inputs, even when later metadata/QC is invalid."""
        if (self.originals is None or self.stop_requested or self.stop_event.is_set()
                or self.state!='recording' or (self.end is not None and receive_ns>=self.end)):
            return False
        accepted=self.originals.submit(topic,data,receive_ns)
        if not accepted and not self.originals.queue.closed:
            self.counters['original_ingress_rejected']+=1
            self.fail('original_writer_queue_overflow:'+topic)
        return accepted

    def submit(self,metadata,rgb=None,depth=None,points=None):
        if self.stop_event.is_set() or (self.raw_only and self.stop_requested) or self.state not in {'warming_up','recording'}:
            return {'accepted':False,'reason':'session_not_accepting'}
        ident=metadata.get('source_id')
        if ident not in self.cfg['sources']:
            raise ValueError('未配置的数据来源')
        if self.ros_time and metadata.get('input_contract')!=CONTRACT:
            raise ValueError('ROS 采集仅接受 humanoid-ros-capture/1.2 标准化输入；设备时间换算属于独立相机节点')
        now=time.monotonic_ns()
        size=(len(points) if points is not None else 0)+sum(getattr(x,'nbytes',0) for x in (rgb,depth) if x is not None)
        if size*2>self.cfg['buffers']['working_bytes'] or not self.copy_slots.acquire(blocking=False):
            self.counters['copy_budget_rejected']+=1
            if self.cfg['capture'] and 'named_axes' in self.cfg['sources'][ident]:
                self.fail('named_input_copy_budget_exceeded:'+ident)
            return {'accepted':False,'reason':'copy_budget_exceeded'}
        try:
            sample=normalize(metadata,self.cfg['sources'][ident],self.clocks,self.id,now,rgb,depth,points,
                             allow_unknown_depth=self.raw_only)
            accepted=self.ingress.offer(sample,sample.nbytes)
        except (ValueError,KeyError,TypeError):
            if self.cfg['capture'] and 'named_axes' in self.cfg['sources'][ident]:
                self.fail('named_input_contract_rejected:'+ident)
            raise
        finally:
            self.copy_slots.release()
        if not accepted:
            # A stop/duration cutoff can close ingress during normalization.
            # That boundary is expected, not an overflow or failed recording.
            if self.ingress.closed:
                return {'accepted':False,'reason':'session_not_accepting'}
            self.counters['ingress_rejected']+=1
            if self.cfg['capture'] and 'named_axes' in self.cfg['sources'][ident]:
                self.fail('named_input_ingress_overflow:'+ident)
        return {'accepted':accepted,'reason':None if accepted else 'ingress_budget_exceeded','source_seq':sample.source_seq}

    def fail(self,reason):
        self.errors.append({'reason':reason,'time_ns':time.monotonic_ns()})
        now=self.data_now() if not self.clock_failed else (self.end or 0)
        self.end=min(self.end,now) if self.end is not None else now
        self.stop_event.set()

    def process(self,sample):
        ident=sample.source_id
        d=sample.data()
        sequence_key=(ident,d['clock_id'],d['clock_epoch'],d.get('source_epoch'))
        seqs=self.recent_seq.setdefault(sequence_key,set())
        order=self.sequence_order.setdefault(sequence_key,deque())
        if sample.source_seq in seqs or sample.source_seq<self.highest_seq.get(sequence_key,-1)-8192:
            self.storage.required('gap',{'source_id':ident,'source_seq':sample.source_seq,'reason':'duplicate_or_out_of_window_sequence'})
            self.counters['duplicate_samples']+=1
            return
        seqs.add(sample.source_seq)
        order.append(sample.source_seq)
        self.highest_seq[sequence_key]=max(self.highest_seq.get(sequence_key,0),sample.source_seq)
        if len(seqs)>8192:
            seqs.discard(order.popleft())
        source=self.cfg['sources'][ident]
        d['kind']=source['kind']
        if 'named_axes' in source and self.ros_time and d['clock_epoch']!=self.scheduler.epoch:
            self.storage.required('source_restart',{'source_id':ident,'metadata':d,'reason':'source_clock_epoch_changed'})
            self.fail('source_clock_epoch_changed:'+ident)
            return
        last=self.last_clock_sample.get(ident)
        if last and d['clock_id']!=last[0]:
            self.storage.required('source_restart',{'source_id':ident,'metadata':d,'reason':'source_clock_identity_changed'})
            self.fail('source_clock_identity_changed:'+ident)
            return
        jumped=bool(last and (d['clock_id'],d['clock_epoch'])==last[:2] and d['source_seq']>last[2] and d['source_timestamp_ns']<last[3]-round(self.cfg['buffers']['retention_ms']*1e6))
        if jumped:
            d.update(valid=False,command_valid=False)
            d.setdefault('ingest_invalid_reasons',[]).append('clock_jump_requires_new_epoch')
            self.counters['clock_jumps:'+ident]+=1
            sample=replace(sample,document=json.dumps(d,allow_nan=False,separators=(',',':')).encode())
        elif (last is None or d['source_seq']>last[2]) and (
                'named_axes' not in source or d['source_timestamp_ns']<=d['receive_time_ns']):
            # A future command can be buffered, but is not evidence that the
            # source clock already advanced (nor that the next packet jumped).
            self.last_clock_sample[ident]=(d['clock_id'],d['clock_epoch'],d['source_seq'],d['source_timestamp_ns'])
        repeated=set()
        if self.raw_only:
            self._persist_raw(sample,d,source)
            if self.next_state_actions:
                self.next_state_actions.observe(sample)
                if self.next_state_actions.cache.evictions:
                    self.fail('next_state_action_cache_capacity_exceeded')
            return
        if isinstance(sample,RGBDSample):
            for name in (('rgb','depth') if self.cfg['sources'][ident].get('depth_enabled',True) else ('rgb',)):
                image=d[name]
                seen=self.stream_sequences.setdefault((ident,name),{})
                key=(image['clock_id'],image['clock_epoch'],image['source_seq'])
                if key in seen:
                    repeated.add(name)
                else:
                    seen[key]=True
                    if len(seen)>8192:
                        del seen[next(iter(seen))]
            reasons=list(d.get('ingest_invalid_reasons',[]))+[name+':duplicate_raw_source_frame' for name in sorted(repeated)]
            report=self.reports.get(ident,{})
            if self.cfg['alignment'].get('camera_validation')=='strict' and self.cfg['alignment']['strict'] and any(d.get(key)!=report.get(key) for key in ('camera_config_id','calibration_id','sync_evidence_version')):
                reasons.append('camera_configuration_changed_requires_revalidation')
            calibration=report.get('calibration',{})
            for name in (('rgb','depth') if self.cfg['sources'][ident].get('depth_enabled',True) else ('rgb',)):
                intrinsics=calibration.get(name+'_intrinsics',{})
                if intrinsics and d[name]['shape'][:2]!=[intrinsics.get('height'),intrinsics.get('width')]:
                    reasons.append(name+':calibration_dimensions_mismatch')
            d['ingest_invalid_reasons']=reasons
            _,raw_reasons=image_quality(d,sample.capture_time_ns,self.cfg['alignment'],source)
            d['raw_pair_quality_reasons']=raw_reasons
            for reason in raw_reasons:
                self.counters['raw_invalid:'+ident+':'+reason]+=1
            sample=replace(sample,document=json.dumps(d,allow_nan=False,separators=(',',':')).encode())
            self.counters['repeated_raw_frames']+=len(repeated)
        # This runs on the session worker, never a ROS/control callback. Retry
        # brief writer-lock contention within the existing bounded deadline.
        if not self.storage.required('raw',d):
            self.fail('raw_numeric_queue_overflow:'+ident)
            return
        self.source_counts[ident]+=1
        self.metric(ident+'.receive_minus_capture_ns',sample.receive_time_ns-sample.capture_time_ns)
        if self.t0 is not None:
            nearest=round((sample.capture_time_ns-self.t0)*self.cfg['dataset']['fps']/1e9)
            deadline=target_time(self.t0,nearest,self.cfg['dataset']['fps'])+round(self.cfg['alignment']['delay_ms']*1e6)
            if not self.clock_failed and sample.arrival_time_ns>self.deadline_steady(deadline):
                self.counters['late_samples:'+ident]+=1
        if d.get('valid',True) or not self.cfg['alignment']['strict']:
            self.source_ready.add(ident)
        if isinstance(sample,PointCloudSample) and source.get('persist',True):
            if not self.storage.required('pointcloud',d,sample.points):
                self.fail('pointcloud_persistence_queue_overflow:'+ident)
        if isinstance(sample,RGBDSample):
            if source.get('depth_enabled',True) and 'depth' not in repeated and not self.storage.required('depth',d,sample.depth_values):
                self.fail('depth_persistence_queue_overflow:'+ident)
            if 'rgb' not in repeated and not self.encoders[ident].submit(sample):
                self.counters['video_rejected']+=1
                self.storage.required('gap',{'source_id':ident,'source_seq':sample.source_seq,'reason':'video_queue_overflow'})
            for name in (('rgb','depth') if self.cfg['sources'][ident].get('depth_enabled',True) else ('rgb',)):
                image=d[name]
                actual,bound=image.get('actual_exposure_us'),image.get('exposure_uncertainty_us')
                if actual is not None and bound is not None:
                    self.metric(ident+'.'+name+'.exposure_upper_us',actual+bound)
                    if actual+bound>source['exposure']['max_actual_us']:
                        self.counters['exposure_violations']+=1
        if source['kind']!='event':
            previous_evictions=self.cache.evictions
            self.cache.add(sample,self.data_now() if not self.clock_failed else sample.capture_time_ns)
            if self.cfg['capture'] and self.cache.evictions>previous_evictions:
                self.fail('capture_cache_capacity_exceeded:'+ident)
        if not self.clock_failed and self.t0 is None and all(not v['required'] or v['kind']=='event' or k in self.source_ready for k,v in self.cfg['sources'].items()):
            cameras=sorted(k for k,v in self.cfg['sources'].items() if v['kind']=='rgbd' and v['required'])
            primary=self.cfg['alignment']['primary_camera'] or (cameras[0] if cameras else None)
            if primary is None:
                self.t0=self.data_now()
            elif ident==primary:
                _,reasons=image_quality(d,sample.capture_time_ns,self.cfg['alignment'],source)
                if not reasons or not self.cfg['alignment']['strict']:
                    self.t0=sample.capture_time_ns
            if self.t0 is not None:
                duration=self.cfg['dataset'].get('duration_seconds')
                if duration is not None and self.end is None:
                    self.end=self.t0+round(duration*1e9)
                self.state='recording'
                if primary is not None and d.get('trigger_cycle_id') is not None and self.reports.get(primary,{}).get('trigger_grid_mapping_verified'):
                    for camera in cameras:
                        report=self.reports.get(camera,{})
                        if camera==primary or (report.get('trigger_cycle_domain') and report.get('trigger_cycle_domain')==self.reports[primary].get('trigger_cycle_domain')):
                            if report.get('trigger_grid_mapping_verified') and report.get('trigger_cycle_fps')==self.cfg['dataset']['fps']:
                                self.aligner.trigger_origins[camera]=d['trigger_cycle_id']
                self.storage.required('timeline',{'t0_ns':self.t0,'fps':self.cfg['dataset']['fps'],'trigger_origins':self.aligner.trigger_origins})

    def _persist_raw(self,sample,data,source):
        """Preserve source facts; derived action candidates use a separate channel."""
        ident=sample.source_id
        if not self.storage.required('raw',data):
            self.fail('raw_writer_queue_overflow:'+ident)
            return
        self.source_counts[ident]+=1
        if isinstance(sample,PointCloudSample) and source.get('persist',True):
            if not self.storage.required('pointcloud',data,sample.points):
                self.fail('pointcloud_persistence_queue_overflow:'+ident)
        if isinstance(sample,RGBDSample):
            for name in (('rgb','depth') if self.cfg['sources'][ident].get('depth_enabled',True) else ('rgb',)):
                frame=data[name]
                key=(frame['clock_id'],frame['clock_epoch'],frame['source_seq'])
                seen=self.stream_sequences.setdefault((ident,name),{})
                if key in seen:
                    self.counters['repeated_raw_frames']+=1
                    self.storage.required('gap',{'source_id':ident,'source_seq':sample.source_seq,
                        'reason':name+':duplicate_raw_source_frame'})
                    continue
                seen[key]=True
                if len(seen)>8192:del seen[next(iter(seen))]
                if self.storage.compact_media and (name=='rgb' or self.storage.depth_video):
                    accepted=self.encoders[ident if name=='rgb' else ident+'_depth'].submit(sample)
                else:
                    accepted=self.storage.required(name,data,sample.rgb_pixels if name=='rgb' else sample.depth_values)
                if not accepted:
                    self.storage.required('gap',{'source_id':ident,'source_seq':sample.source_seq,
                        'reason':name+':persistence_queue_overflow'})
                    self.fail(name+':persistence_queue_overflow:'+ident)

    def run(self):
        try:
            while True:
                if self.ros_time and not self.clock_failed:
                    self.observe_ros_clock(time.time_ns(),time.monotonic_ns())
                delay=0 if self.raw_only else round(self.cfg['alignment']['delay_ms']*1e6)
                if self.stop_event.is_set() or self.clock_failed or (self.end is not None and time.monotonic_ns()>=self.deadline_steady(self.end+delay)):
                    self.stop_event.set()
                    self.ingress.close()
                sample=self.ingress.get(.002)
                if sample is not None:
                    self.process(sample)
                now=self.data_now() if not self.clock_failed else (self.end or 0)
                if (self.next_state_actions and self.t0 is not None and not self.clock_failed
                        and not self.ingress.items and not self.stop_event.is_set()):
                    self._write_action_targets(now)
                if not self.raw_only and self.t0 is not None and not self.clock_failed:
                    fps=self.cfg['dataset']['fps']
                    delay=round(self.cfg['alignment']['delay_ms']*1e6)
                    available_end=min(now-delay+1,self.end if self.end is not None else now+1)
                    due=target_count(self.t0,available_end,fps)
                    # Skip explicit target ranges on sustained lag, never renumber or chase forever.
                    if due-self.k>self.cfg['alignment']['max_catchup_rows']:
                        next_k=due-self.cfg['alignment']['max_catchup_rows']
                        self.storage.required('alignment_gap',{'first_target':self.k,'end_target_exclusive':next_k,
                            'alignment_version':self.aligner.version,'reason':'online_catchup_budget_exhausted'})
                        self.counters['offline_repair_targets']+=next_k-self.k
                        self.k=next_k
                    # Drain all samples that were already offered before evaluating deadlines.
                    # Limited batches leave overdue targets to the explicit catch-up budget.
                    if self.ingress.items and not self.stop_event.is_set():
                        continue
                    while self.k<due:
                        target=target_time(self.t0,self.k,fps)
                        row=self.aligner.row(self.k,self.t0,now,deadline=self.deadline_steady(target+delay),epoch=self.scheduler.epoch if self.ros_time else 0)
                        row.update(deadline_ros_ns=target+delay,ready_steady_ns=time.monotonic_ns(),ros_clock_epoch=self.scheduler.epoch if self.ros_time else 0)
                        if not self.storage.submit('aligned',row):
                            self.storage.required('alignment_gap',{'first_target':self.k,'end_target_exclusive':self.k+1,
                                'alignment_version':self.aligner.version,'reason':'aligned_writer_queue_overflow'})
                            self.counters['offline_repair_targets']+=1
                        else:
                            self.counters['rows']+=1
                            self.counters['valid_rows' if row['valid'] else 'invalid_rows']+=1
                            for reason in row['invalid_reasons']:
                                self.counters['invalid:'+reason]+=1
                            self.latest_rows.append(row)
                            self.metric('alignment_output_delay_ns',now-row['target_time_ns'])
                            self.metric('camera_pair_max_bound_ns',row['camera_pair_max_bound_ns'])
                            for ident,group in row['images'].items():
                                self.metric(ident+'.rgbd_skew_ns',group['rgbd_midpoint_skew_ns'])
                                for name in (('rgb','depth') if self.cfg['sources'][ident].get('depth_enabled',True) else ('rgb',)):
                                    error=group[name]['time_error_ns']
                                    self.metric(ident+'.'+name+'.grid_error_abs_ns',abs(error) if error is not None else None)
                            for ident,fields in row['state'].items():
                                for name,field in fields.items():
                                    if 'bracket_gap_ns' in field:
                                        self.metric(ident+'.'+name+'.bracket_gap_ns',field['bracket_gap_ns'])
                        self.k+=1
                if self.stop_event.is_set() and not self.ingress.items and (self.raw_only or self.t0 is None or self.k>=target_count(self.t0,self.end,self.cfg['dataset']['fps'])):
                    break
            if self.t0 is None:
                self.errors.append({'reason':'required_sources_never_ready','time_ns':time.monotonic_ns()})
            if self.raw_only and not self.source_counts:
                self.errors.append({'reason':'no_samples_received','time_ns':time.monotonic_ns()})
            if self.next_state_actions and self.t0 is not None and not self.clock_failed:
                self._write_action_targets(self.end,final=True)
            self.state='draining'
            if self.originals:
                self.originals.stop()
                if self.originals.closed:
                    self.storage.committed.append({'path':'originals.mcap','kind':'original_ros',
                        'closed':True,'fsynced':self.cfg['storage']['fsync_on_commit']})
                    for topic,spec in self.originals.topics.items():
                        if (spec['stream'] in {'rgb','depth','rgb_frame','depth_frame'} and
                                self.cfg['sources'][spec['source_id']]['required'] and
                                not self.originals.counts[topic]):
                            self.errors.append({'reason':'original_stream_missing:'+topic,
                                                'time_ns':time.monotonic_ns()})
            if self.active_marker is not None:
                self.mark('segment',note='录制结束自动关闭无效片段',closing=True)
            for encoder in self.encoders.values():
                encoder.stop()
            self.storage.required('session_end',{'t0_ns':self.t0,'end_ns':self.end,'target_count':self.k,
                'source_counts':dict(self.source_counts),'counters':dict(self.counters),'errors':list(self.errors)})
            self.storage.stop()
            self.state='failed' if self.errors or self.storage.error else 'completed'
            from .media_spool import MediaSpool
            spooled=[e for e in self.encoders.values() if isinstance(e.queue,MediaSpool)]
            if self.state=='completed' and any(e.encoded!=e.queue.submitted for e in spooled):
                raise RuntimeError('encoded media count does not match persisted source frames')
            for encoder in spooled:
                encoder.queue.release(success=self.state=='completed')
        except Exception as error:
            self.fail(type(error).__name__+': '+str(error))
            self.state='failed'
            if self.originals:self.originals.stop()
            for encoder in self.encoders.values():
                encoder.stop()
            self.storage.stop()
        finally:
            from .media_spool import MediaSpool
            for encoder in self.encoders.values():
                if isinstance(encoder.queue,MediaSpool) and not encoder.thread.is_alive():
                    encoder.queue.release()
            try:
                atomic_json(self.path/'manifest.json',{'session_id':self.id,'state':self.state,
                    'recording_mode':'raw' if self.raw_only else 'aligned',
                    'ended_unix_ns':time.time_ns(),
                    't0_ns':self.t0,'end_ns':self.end,'target_count':self.k,
                    'source_counts':dict(self.source_counts),
                    'action_fallback':self.next_state_actions.status() if self.next_state_actions else None,
                    'original_capture':self.originals.status() if self.originals else None,
                    'committed':self.storage.committed,'errors':list(self.errors),
                    'durability_policy':'closed_and_fsync' if self.cfg['storage']['fsync_on_commit'] else 'closed_without_fsync'})
                if not self.raw_only:atomic_json(self.path/'quality.json',self.status())
            finally:
                self.closed.set()

    def _write_action_targets(self,now,*,final=False):
        for target in self.next_state_actions.advance(self.t0,now,end=self.end,
                epoch=self.scheduler.epoch if self.ros_time else 0,final=final):
            if not self.storage.required('action_target',target):
                self.fail('action_target_writer_queue_overflow')
                return
            self.counters['action_target_candidates']+=1

    def record_button(self,event):
        if self.stop_requested or self.stop_event.is_set() or self.closed.is_set():
            return
        self.button_seq+=1
        now=self.data_now()
        data={'session_id':self.id,'source_id':'buttons','source_seq':self.button_seq,
            'source_timestamp_ns':now,'capture_time_ns':now,'receive_time_ns':now,
            'clock_id':self.cfg['clock']['domain'],'clock_epoch':self.scheduler.epoch,'clock_model_id':'host_receive_v1',
            'timestamp_quality':'host_receive','units':'button_bitmask','coordinate_frame':'controller',
            'payload':event}
        if not self.storage.submit('button',data):
            self.counters['button_raw_rejected']+=1

    def mark(self,scope='segment',note='',source='web',closing=False):
        if not closing and (self.stop_requested or self.stop_event.is_set() or self.closed.is_set()):
            raise ValueError('当前没有进行中的对齐采集')
        if scope not in {'segment','recording','frame'}:
            raise ValueError('无效标记范围')
        with self.annotation_lock:
            now=self.end if closing else min(self.data_now() if not self.clock_failed else (self.end or 0),self.end if self.end is not None else 2**63-1)
            ident=self.active_marker if scope=='segment' and self.active_marker else uuid.uuid4().hex
            action='end' if scope=='segment' and self.active_marker else {'segment':'start','recording':'recording','frame':'point'}[scope]
            data={'id':ident,'action':action,'target_time_ns':now,'source':source,'note':str(note)[:10000],'label':'invalid'}
            if not self.storage.submit('annotation',data):
                raise ValueError('标记队列已满，未保存')
            if scope=='segment':
                self.active_marker=None if action=='end' else ident
            return data

    def request_stop(self):
        """Freeze this recording's end immediately; drain/encode on its own thread."""
        if self.closed.is_set() or self.stop_requested:
            return self.status()
        now=self.data_now() if not self.clock_failed else (self.end or 0)
        self.end=min(self.end,now) if self.end is not None else now
        self.stop_requested=True
        if self.raw_only:
            self.stop_event.set()
            self.ingress.close()
        return {**self.status(),'state':'draining'}

    def stop(self):
        self.request_stop()
        if self.thread.ident is None:
            return self.status()
        # Keep accepting already-in-flight samples until the final deadline.
        self.thread.join(45)
        if self.thread.is_alive():
            raise RuntimeError('采集仍在后台收尾，请稍后查看状态；活动文件尚未提交')
        return self.status()

    def status(self):
        return {'session_id':self.id,'state':'draining' if self.stop_requested and not self.closed.is_set() else self.state,
            'active':not self.closed.is_set(),
            'recording':not self.closed.is_set() and not self.stop_requested and not self.stop_event.is_set(),
            'capture_clock_id':self.cfg['clock']['domain'],'clock_epoch':self.scheduler.epoch,'clock_mapping_valid':not self.clock_failed,
            't0_ns':self.t0,'end_ns':self.end,'target_count':self.k,'source_counts':dict(self.source_counts),
            'counters':dict(self.counters),'errors':list(self.errors),'active_marker':self.active_marker,
            'ingress':self.ingress.status(),'writer':self.storage.status(),
            'original_writer':self.originals.status() if self.originals else None,
            'encoders':{k:v.status() for k,v in self.encoders.items()},
            'recording_mode':'raw' if self.raw_only else 'aligned',
            'action_fallback':self.next_state_actions.status() if self.next_state_actions else None,
            'cache':{'bytes':self.cache.bytes,'peak_bytes':self.cache.peak_bytes,'evictions':self.cache.evictions} if self.cache else {},
            'peak_process_rss_bytes':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
            'metrics':{k:v.summary() for k,v in tuple(self.metrics.items())},
            'latest_rows':list(self.latest_rows)[-3:]}
