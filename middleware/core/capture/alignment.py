"""Deterministic fixed-grid association. No IO, decoding, control commands or waits."""
import math
import numpy as np

from . import ALIGNMENT_VERSION
from .config import config_hash
from .camera_rules import uses_rgb_reference, limit_ns
from .primary_timeline import uses_primary_timeline, match_limit_ns
from .exposure_association import exposure_errors, numeric_target, associate_exposure_states


def target_time(t0,k,fps):
    # Integer round-to-nearest, ties-to-even, matching round(k * 1e9 / F).
    quotient,remainder=divmod(k*1_000_000_000,fps)
    return t0+quotient+int(2*remainder>fps or (2*remainder==fps and quotient%2))


def target_count(t0,end,fps):
    if end<=t0:
        return 0
    k=max(0,((end-t0)*fps)//1_000_000_000)
    while target_time(t0,k,fps)<end:
        k+=1
    while k>0 and target_time(t0,k-1,fps)>=end:
        k-=1
    return k


def clock_reasons(d):
    reasons=[]
    if d.get('timestamp_quality')=='ros_stamped':
        return reasons
    if d.get('timestamp_quality') not in {'hardware_synced','mapped','ros_stamped'}:
        reasons.append('timestamp_quality_insufficient')
    if d.get('clock_uncertainty_ns') is None or not d.get('clock_evidence'):
        reasons.append('clock_uncertainty_unavailable')
    return reasons


def known_bound(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and value>=0


def image_quality(d,target,cfg,source):
    streams=('rgb','depth') if source.get('depth_enabled',True) else ('rgb',)
    rgb_reference=uses_rgb_reference(cfg)
    grid_limit=limit_ns(cfg,source,'camera_max_error_ms')
    pair_limit=limit_ns(cfg,source,'rgbd_max_midpoint_skew_ms')
    if cfg.get('camera_validation') in {'driver_headers','timestamps'}:
        reasons=list(d.get('ingest_invalid_reasons',[]))+list(d.get('invalid_reasons',[]))
        if d.get('valid') is False and not reasons:
            reasons.append('upstream_invalid')
        images={}
        for name in streams:
            item=dict(d[name])
            reasons.extend(name+':'+error for error in exposure_errors(item,source))
            capture=item.get('capture_time_ns',item.get('exposure_midpoint_ns'))
            error=None if capture is None else capture-target
            if error is None:
                reasons.append(name+':sampling_timestamp_missing')
            elif not uses_primary_timeline(cfg) and (name=='rgb' or not rgb_reference) and abs(error)>grid_limit:
                reasons.append(name+':grid_error_exceeded')
            if cfg['causal'] and error is not None and error>0:
                reasons.append(name+':future_image_in_causal_mode')
            item.update(exposure_midpoint_ns=capture,time_error_ns=error,repeated=False,
                sampling_time_basis=item.get('timestamp_semantics','ros_driver_header'),physical_midpoint_verified=False)
            images[name]=item
        times=[images[k]['exposure_midpoint_ns'] for k in streams]
        skew=abs(times[0]-times[1]) if len(times)==2 and None not in times else None
        if skew is not None and skew>pair_limit:
            reasons.append('rgbd_midpoint_skew_exceeded')
        midpoint_semantics=all(images[k].get('timestamp_semantics')=='exposure_midpoint' for k in streams)
        return {'source_seq':d['source_seq'],'pair_seq':d['pair_seq'],'trigger_cycle_id':None,
            'pairing_method':d.get('pairing_method'),'clock_epoch':d['clock_epoch'],
            'camera_config_id':d.get('camera_config_id'),'calibration_id':d.get('calibration_id'),
            'rgbd_midpoint_skew_ns':skew if midpoint_semantics else None,'driver_header_skew_ns':None if midpoint_semantics else skew,
            'rgbd_relative_uncertainty_ns':None,'physical_midpoint_verified':False,
            'rgbd_limit_ns':pair_limit,'rgb_grid_limit_ns':grid_limit,
            'timing_reference':'per_camera_rgb' if rgb_reference else 'grid',
            'exposure_validation':'metadata' if source.get('require_exposure_metadata') else 'disabled',**images},reasons
    reasons=list(dict.fromkeys(d.get('ingest_invalid_reasons',[])+d.get('invalid_reasons',[])+d.get('quality',{}).get('invalid_reasons',[])))
    if d.get('valid') is False and not reasons:
        reasons.append('upstream_invalid')
    images={}
    if 'depth' in streams and (d.get('pairing_method') not in {'verified_trigger','vendor_guaranteed','verified_shared_source'} or not d.get('sync_evidence_version')):
        reasons.append('synchronization_evidence_missing')
    cycles=[d.get('trigger_cycle_id')]+[d[k].get('trigger_cycle_id') for k in streams]
    if any(v is not None for v in cycles) and (None in cycles or len(set(cycles))!=1):
        reasons.append('rgbd_cycle_mismatch')
    if d.get('pairing_method')=='verified_trigger' and cycles[0] is None:
        reasons.append('trigger_cycle_missing')
    if not d.get('calibration_id') or not d.get('camera_config_id'):
        reasons.append('calibration_or_config_version_missing')
    for name in streams:
        image=dict(d[name])
        for reason in clock_reasons(image):
            reasons.append(name+':'+reason)
        actual,bound=image.get('actual_exposure_us'),image.get('exposure_uncertainty_us')
        if not known_bound(actual) or not known_bound(bound) or not image.get('uncertainty_evidence'):
            reasons.append(name+':exposure_evidence_missing')
        elif actual+bound>source['exposure']['max_actual_us']:
            reasons.append(name+':exposure_limit_exceeded')
        if source['exposure'].get('auto'):
            auto=d.get('auto_control_status',{})
            if not auto.get('ready') or not auto.get('auto_enabled') or not d.get('limit_verification'):
                reasons.append(name+':limited_auto_mode_unverified')
            if not known_bound(image.get('gain_level')) or image['gain_level']>source['exposure'].get('auto_gain_limit',64):
                reasons.append(name+':gain_limit_exceeded_or_unavailable')
        temporal=image.get('temporal_support',{})
        if temporal.get('model')!='global_single_exposure' and not (temporal.get('model')=='validated_temporal_model' and temporal.get('evidence')):
            reasons.append(name+':temporal_support_unverified')
        if temporal.get('cross_frame_fusion',False):
            reasons.append(name+':cross_frame_fusion')
        midpoint=image.get('exposure_midpoint_ns')
        error=None if midpoint is None else midpoint-target
        uncertainty=image.get('grid_uncertainty_ns')
        if known_bound(uncertainty) and known_bound(image.get('clock_uncertainty_ns')) and uncertainty<image['clock_uncertainty_ns']:
            reasons.append(name+':grid_bound_below_clock_bound')
        if error is None or not known_bound(uncertainty) or not image.get('uncertainty_evidence'):
            reasons.append(name+':grid_timing_evidence_missing')
        elif not uses_primary_timeline(cfg) and (name=='rgb' or not rgb_reference) and abs(error)+uncertainty>grid_limit:
            reasons.append(name+':grid_error_exceeded')
        if error is not None and cfg['causal'] and error>0:
            reasons.append(name+':future_exposure_in_causal_mode')
        image.update(time_error_ns=error,repeated=False,
            exposure_limit_us=source['exposure']['max_actual_us'],grid_limit_ns=grid_limit)
        images[name]=image
    rgb,depth=images['rgb']['exposure_midpoint_ns'],images.get('depth',{}).get('exposure_midpoint_ns')
    skew=None if rgb is None or depth is None else abs(rgb-depth)
    bound=d.get('rgbd_relative_uncertainty_ns')
    if 'depth' in streams and (skew is None or not known_bound(bound) or not d.get('relative_uncertainty_evidence')):
        reasons.append('rgbd_relative_uncertainty_missing')
    elif 'depth' in streams and skew+bound>pair_limit:
        reasons.append('rgbd_midpoint_skew_exceeded')
    return {'source_seq':d['source_seq'],'pair_seq':d['pair_seq'],
        'trigger_cycle_id':d.get('trigger_cycle_id'),'pairing_method':d.get('pairing_method'),
        'sync_evidence_version':d.get('sync_evidence_version'),'calibration_id':d.get('calibration_id'),
        'camera_config_id':d.get('camera_config_id'),'rgbd_midpoint_skew_ns':skew,
        'clock_epoch':d['clock_epoch'],'auto_control_status':d.get('auto_control_status'),
        'limit_verification':d.get('limit_verification'),
        'rgbd_relative_uncertainty_ns':bound,'rgbd_limit_ns':pair_limit,
        **images},reasons


def interpolate(left,right,alpha,rule,gap):
    a,b=np.asarray(left,dtype=float),np.asarray(right,dtype=float)
    if a.shape!=b.shape:
        raise ValueError('state_shape_changed')
    semantics=rule['semantics']
    if semantics=='angle':
        period=float(rule.get('period',2*math.pi))
        speed=rule.get('max_speed')
        if speed is None or period<=0 or speed*gap/1e9>=period/2:
            raise ValueError('angle_turn_count_unverified')
        delta=(b-a+period/2)%period-period/2
        if np.any(np.abs(delta)>speed*gap/1e9+1e-9):
            raise ValueError('angle_discontinuity')
        result=(a+alpha*delta+period/2)%period-period/2
    elif semantics=='quaternion':
        if a.shape!=(4,) or min(np.linalg.norm(a),np.linalg.norm(b))<1e-12:
            raise ValueError('invalid_quaternion')
        a,b=a/np.linalg.norm(a),b/np.linalg.norm(b)
        dot=float(np.dot(a,b))
        if dot<0:
            b,dot=-b,-dot
        dot=min(1.,dot)
        if dot>.9995:
            result=a+alpha*(b-a)
            result/=np.linalg.norm(result)
        else:
            theta=math.acos(dot)
            result=(math.sin((1-alpha)*theta)*a+math.sin(alpha*theta)*b)/math.sin(theta)
    else:
        result=(1-alpha)*a+alpha*b
    return result.tolist()


class Aligner:
    def __init__(self,cfg,cache,version=ALIGNMENT_VERSION):
        self.cfg,self.cache,self.version=cfg,cache,version
        if cfg['alignment'].get('state_reference')=='primary_exposure_midpoint':
            self.version+='+exposure-midpoint.1'
        if uses_rgb_reference(cfg['alignment']):
            self.version+='+camera-rules.1'
        self.hash=config_hash(cfg)
        self.previous_images={}
        self.trigger_origins={}

    def row(self,k,t0,ready_time_ns,deadline=None,epoch=0,*,target_ns=None):
        a=self.cfg['alignment']
        primary_mode=uses_primary_timeline(a)
        target=target_time(t0,k,self.cfg['dataset']['fps']) if target_ns is None else target_ns
        deadline=target+round(a['delay_ms']*1e6) if deadline is None else deadline
        row={'target_frame_index':k,'target_time_ns':target,'timestamp':k/self.cfg['dataset']['fps'],
            'ready_time_ns':ready_time_ns,'deadline_ns':deadline,'valid':True,'invalid_reasons':[],
            'images':{},'pointclouds':{},'state':{},'action':{},'alignment_version':self.version,'config_hash':self.hash,
            'persistence_state':'pending','strict_qualified':False}
        reasons=row['invalid_reasons']
        used=[]
        for ident,source in sorted(self.cfg['sources'].items(),key=lambda item:(item[1]['kind']!='rgbd',primary_mode and item[0]!=a['primary_camera'])):
            samples=self.cache.eligible(ident,deadline)
            errors=[]
            reference=numeric_target(row,self.cfg) if source['kind'] in {'state','action'} else target
            if source['kind']=='rgbd':
                if primary_mode and ident!=a['primary_camera']:
                    reference=numeric_target(row,self.cfg)
                streams=('rgb','depth') if source.get('depth_enabled',True) else ('rgb',)
                origin=self.trigger_origins.get(ident,source.get('trigger_origin'))
                if origin is not None:
                    choices=[s for s in samples if s.data().get('trigger_cycle_id')==origin+k]
                else:
                    choices=samples
                if primary_mode:
                    choices=[s for s in choices if reference is not None and abs(s.capture_time_ns-reference)<=match_limit_ns(self.cfg)]
                if a['causal']:
                    choices=[s for s in choices if all(s.data()[key].get('exposure_midpoint_ns') is not None and s.data()[key]['exposure_midpoint_ns']<=target for key in streams)]
                selected=min(choices,key=lambda s:(abs(s.capture_time_ns-reference),s.capture_time_ns,s.data()['pair_seq'])) if choices else None
                if selected is None:
                    errors.append('camera_missing')
                else:
                    image,errors=image_quality(selected.data(),reference,a,source)
                    if primary_mode:
                        image['timing_reference']='primary_rgb_exposure_midpoint'
                        image['match_limit_ns']=match_limit_ns(self.cfg)
                    for name in streams:
                        key=(ident,name,image[name]['clock_id'],image[name]['clock_epoch'],image[name]['source_seq'])
                        if self.previous_images.get((ident,name))==key:
                            image[name]['repeated']=True
                            if not source.get('allow_camera_repeat',a['allow_camera_repeat']):
                                errors.append(name+':repeated_source_frame')
                        used.append(((ident,name),key))
                    image['required']=source['required']
                    image['receive_time_ns']=selected.receive_time_ns
                    row['images'][ident]=image
            elif source['kind'] in {'state','action'} and reference is None:
                errors.append('primary_exposure_midpoint_missing')
                if 'named_axes' in source:
                    from .named import aligned
                    row[source['kind']][ident]=aligned([],None,source,a,epoch)[0]
                else:
                    row[source['kind']][ident]={}
            elif source['kind'] in {'state','action'} and 'named_axes' in source:
                from .named import aligned
                value,errors=aligned(samples,reference,source,a,epoch)
                row[source['kind']][ident]=value
            elif source['kind']=='state':
                value,errors=self.state(samples,reference,source)
                row['state'][ident]=value
            elif source['kind']=='action':
                value,errors=self.action(samples,reference,source)
                row['action'][ident]=value
            if errors:
                row.setdefault('source_issues',{})[ident]=errors
                if source['required']:
                    reasons.extend(ident+':'+e for e in errors)
        for ident,source in self.cfg['sources'].items():
            if source['kind']!='pointcloud':
                continue
            group=row['images'].get(source['camera_source_id'])
            errors=[]
            candidates=[s.data() for s in self.cache.eligible(ident,deadline)]
            matching=[d for d in candidates if group and d.get('pair_seq')==group['pair_seq']
                and d.get('source_depth_seq')==group['depth']['source_seq']
                and d.get('source_rgb_seq') in (None,group['rgb']['source_seq'])
                and d.get('clock_epoch')==group['depth']['clock_epoch']
                and d.get('clock_id')==group['depth']['clock_id']
                and d.get('capture_time_ns')==group['depth']['exposure_midpoint_ns']
                and d.get('camera_config_id')==group['camera_config_id']]
            if not matching:
                errors.append('pointcloud_lineage_mismatch' if candidates else 'pointcloud_missing_or_late')
            else:
                selected=min(matching,key=lambda d:(d.get('compute_complete_time_ns') or 0,d['source_seq']))
                row['pointclouds'][ident]={**selected,'required':source['required'],'persist':source.get('persist',True)}
                errors.extend(selected.get('invalid_reasons',[]))
                if selected.get('valid') is False:
                    errors.append('pointcloud_upstream_invalid')
            if errors:
                row.setdefault('source_issues',{})[ident]=errors
                if source['required']:
                    reasons.extend(ident+':'+e for e in errors)
        associate_exposure_states(self,row,deadline,epoch)
        comparison_streams=('rgb',) if uses_rgb_reference(a) else ('rgb','depth')
        row['camera_pair_time_basis']='rgb_midpoints' if uses_rgb_reference(a) else 'all_image_midpoints'
        required_images=[v[name] for ident,v in row['images'].items() if self.cfg['sources'][ident]['required'] for name in comparison_streams if name in v]
        maximum=0
        for i,image in enumerate(required_images):
            for other in required_images[i+1:]:
                # Sum validated image-to-reference bounds conservatively; correlation unknown.
                bounds=[image.get('grid_uncertainty_ns'),other.get('grid_uncertainty_ns')]
                times=[image.get('exposure_midpoint_ns'),other.get('exposure_midpoint_ns')]
                if all(known_bound(b) for b in bounds) and None not in times:
                    maximum=max(maximum,abs(times[0]-times[1])+sum(bounds))
                elif a.get('camera_validation') in {'driver_headers','timestamps'} and None not in times:
                    maximum=max(maximum,abs(times[0]-times[1]))
                else:
                    reasons.append('camera_pair_uncertainty_missing')
        row['camera_pair_max_bound_ns']=maximum
        row['camera_pair_limit_ns']=round(a['camera_pair_max_skew_ms']*1e6)
        if maximum>row['camera_pair_limit_ns']:
            if not primary_mode or a.get('require_camera_sync',False) or a.get('camera_validation')=='strict':
                reasons.append('camera_pair_skew_exceeded')
            else:
                row.setdefault('timing_warnings',[]).append('camera_pair_skew_exceeded')
        row['valid']=not reasons
        row['clock_epoch']=epoch
        row['strict_qualified']=row['valid'] and a['strict'] and a.get('camera_validation','strict')=='strict'
        row['export_qualified']=row['valid'] and a['strict']
        row['camera_validation']=a.get('camera_validation','strict')
        # Reuse forbidden across adjacent valid rows, including invalid rows between them.
        if row['valid'] or primary_mode:
            self.previous_images.update(used)
        return row

    def state(self,samples,target,source):
        result,errors={},[]
        data=[s.data() for s in samples]
        # Explicitly invalid updates create a discontinuity rather than being bridged.
        before=[d for d in data if d['capture_time_ns']<=target]
        after=[d for d in data if d['capture_time_ns']>=target]
        left=max(before,key=lambda d:(d['capture_time_ns'],d['source_seq'])) if before else None
        right=min(after,key=lambda d:(d['capture_time_ns'],-d['source_seq'])) if after else None
        for name,rule in source['fields'].items():
            l,r=left,right
            if l is None or (r is None and rule['semantics']!='discrete'):
                errors.append(name+':state_missing_bracket')
                continue
            if rule['semantics']=='discrete':
                r=l
                if target-l['capture_time_ns']>rule['max_age_ms']*1e6:
                    errors.append(name+':state_expired')
                    continue
            if l.get('joint_names')!=r.get('joint_names'):
                errors.append(name+':joint_order_changed')
                continue
            if not l.get('valid',True) or not r.get('valid',True):
                errors.append(name+':state_invalid')
                continue
            if (l['clock_id'],l['clock_epoch'],l.get('continuity_id'))!=(r['clock_id'],r['clock_epoch'],r.get('continuity_id')):
                errors.append(name+':clock_or_trajectory_discontinuity')
                continue
            if self.cfg['alignment']['strict']:
                errors.extend(name+':'+e for d in (l,r) for e in clock_reasons(d))
            gap=r['capture_time_ns']-l['capture_time_ns']
            if gap>self.cfg['alignment']['joint_max_bracket_gap_ms']*1e6:
                errors.append(name+':bracket_gap_exceeded')
                continue
            alpha=(target-l['capture_time_ns'])/gap if gap else 0.
            try:
                value=l['payload'][name] if rule['semantics']=='discrete' else interpolate(l['payload'][name],r['payload'][name],alpha,rule,gap)
            except ValueError as error:
                errors.append(name+':'+str(error))
                continue
            result[name]={'value':value,'left_seq':l['source_seq'],'right_seq':r['source_seq'],
                'left_distance_ns':target-l['capture_time_ns'],'right_distance_ns':r['capture_time_ns']-target,
                'bracket_gap_ns':gap,'alpha':alpha,'semantics':rule['semantics'],
                'units':rule['units'],'coordinate_frame':rule['coordinate_frame'],
                'clock_model_ids':[l['clock_model_id'],r['clock_model_id']],
                'timestamp_quality':[l['timestamp_quality'],r['timestamp_quality']]}
        return result,list(dict.fromkeys(errors))

    def action(self,samples,target,source):
        candidates=[s.data() for s in samples]
        candidates=[d for d in candidates if d[d['action_time_field']]<=target]
        if not candidates:
            return {},['action_missing']
        d=max(candidates,key=lambda d:(d[d['action_time_field']],d['source_seq']))
        errors=[]
        if not d['command_valid'] or (d.get('valid_until_ns') is not None and target>=d['valid_until_ns']):
            errors.append('action_invalid_or_expired')
        if source.get('max_age_ms') is not None and target-d[d['action_time_field']]>source['max_age_ms']*1e6:
            errors.append('action_protocol_timeout')
        if source.get('control_mode') and d.get('control_mode')!=source['control_mode']:
            errors.append('action_control_mode_mismatch')
        if self.cfg['alignment']['strict']:
            errors.extend(clock_reasons(d))
        return {'payload':d['payload'],'source_seq':d['source_seq'],'pairing':'active_at_target',
            'time_field':d['action_time_field'],'time_quality':d['action_time_quality'],
            'command_time_ns':d[d['action_time_field']],'control_mode':d.get('control_mode'),
            'clock_model_id':d['clock_model_id'],'timestamp_quality':d['timestamp_quality'],
            'units':d['units'],'coordinate_frame':d['coordinate_frame']},errors
