"""Immutable samples at the driver boundary, with explicit missing evidence."""
from dataclasses import dataclass
import json
import math
import struct

import numpy as np

from .clocks import exposure_midpoint
from .depth_source import inspect_depth

CONTRACT='humanoid-ros-capture/1.2'


def freeze_json(value):
    # Bytes make nested payloads immutable too (a frozen dataclass alone does not).
    return json.dumps(value,allow_nan=False,separators=(',',':')).encode()


@dataclass(frozen=True)
class Sample:
    source_id: str
    source_seq: int
    capture_time_ns: int
    receive_time_ns: int
    arrival_time_ns: int
    clock_epoch: int
    document: bytes

    def data(self):
        return json.loads(self.document)

    @property
    def nbytes(self):
        return len(self.document)+256


@dataclass(frozen=True)
class RGBDSample(Sample):
    rgb_pixels: bytes
    depth_values: bytes

    @property
    def nbytes(self):
        return super().nbytes+len(self.rgb_pixels)+len(self.depth_values)


@dataclass(frozen=True)
class PointCloudSample(Sample):
    points: bytes

    @property
    def nbytes(self):
        return super().nbytes+len(self.points)


def integer(value, name, minimum=0):
    if type(value) is not int or value<minimum:
        raise ValueError(name+' 必须为整数纳秒或非负序号')
    return value


def normalize(metadata, source, clocks, session_id, arrival_time_ns, rgb=None, depth=None, points=None,
              *, allow_unknown_depth=False):
    nonfinite=[]
    def clean(value,path):
        if isinstance(value,float) and not math.isfinite(value):
            nonfinite.append({'path':path,'float64_le_hex':struct.pack('<d',value).hex()})
            return None
        if isinstance(value,list):
            return [clean(x,path+[i]) for i,x in enumerate(value)]
        if isinstance(value,dict):
            return {k:clean(v,path+[k]) for k,v in value.items()}
        return value
    cleaned=clean(dict(metadata),[])
    document=freeze_json(cleaned)
    if len(document)>256*1024:
        raise ValueError('单样本元数据超过 256 KiB；像素请使用独立二进制缓冲')
    d = json.loads(document)
    if nonfinite:
        d.update(valid=False,raw_nonfinite_values=nonfinite)
    ident = d['source_id']
    for key in ('source_seq','source_timestamp_ns','clock_epoch','receive_time_ns'):
        integer(d[key],key)
    if d.get('timestamp_quality') not in {'hardware_synced','mapped','ros_stamped','ros_driver_header','host_receive','unavailable'}:
        raise ValueError('必须明确 timestamp_quality')
    normalized=d.get('input_contract')==CONTRACT
    if normalized:
        capture=integer(d['capture_time_ns'],'capture_time_ns',1)
        if d.get('capture_clock_id')!='ros' or d.get('header_timestamp_ns')!=capture:
            raise ValueError('标准化输入必须 header.stamp=capture_time_ns，且属于共同 ROS 时间轴')
        d.setdefault('clock_uncertainty_ns',None)
        d.setdefault('clock_evidence','')
        d.setdefault('receive_clock_id','ros')
        d['upstream_receive_steady_ns']=d.get('receive_steady_ns')
        d['receive_steady_ns']=arrival_time_ns
        d['invalid_reasons']=list(dict.fromkeys(d.get('invalid_reasons',[])+d.get('quality',{}).get('invalid_reasons',[])))
        if d['invalid_reasons'] or d.get('quality',{}).get('valid') is False:
            d['valid']=False
    else:
        # Read compatibility for explicit v1.1 diagnostic inputs only.
        capture, model = clocks.stamp(d)
        if d['receive_time_ns']>arrival_time_ns+1_000_000:
            raise ValueError('接收时间位于主机未来，请检查单调时钟域')
        d.update(clock_uncertainty_ns=model.uncertainty_ns,clock_evidence=model.evidence)
    d.update(session_id=session_id,capture_time_ns=capture,arrival_time_ns=arrival_time_ns)
    if source['kind']=='pointcloud':
        for key in ('pair_seq','source_depth_seq'):
            integer(d[key],key)
        if d.get('source_rgb_seq') is not None:
            integer(d['source_rgb_seq'],'source_rgb_seq')
        if d.get('camera_source_id')!=source['camera_source_id'] or not d.get('processing_version'):
            raise ValueError('点云必须有来源相机和处理版本')
        payload=bytes(points) if points is not None else b''
        layout=d.get('pointcloud_layout',{})
        if len(payload)!=layout.get('row_step',0)*layout.get('height',0) or not payload:
            raise ValueError('PointCloud2 长度与布局不符')
        d['data_bytes']=len(payload)
        return PointCloudSample(ident,d['source_seq'],capture,d['receive_time_ns'],arrival_time_ns,d['clock_epoch'],freeze_json(d),payload)
    if source['kind']!='rgbd':
        if source['kind'] in {'state','action'}:
            payload = d['payload']
            for name in source['fields']:
                if name not in payload:
                    raise ValueError('缺少数值字段: '+name)
                values=np.asarray(payload[name])
                if not d.get('validity_event') and not nonfinite and (values.dtype.kind not in 'biuf' or values.size<1 or values.size>4096 or not np.all(np.isfinite(values))):
                    raise ValueError('数值字段必须是非空有限数值（最多 4096 维）')
            if 'named_axes' in source:
                names=d.get('joint_names',[])
                axes={a['name']:a for a in source['named_axes']}
                if not names or len(set(names))!=len(names) or set(names)-axes.keys():
                    raise ValueError('Unknown or duplicate named axes')
                if not d.get('validity_event') and len(payload['position'])!=len(names):
                    raise ValueError('Named position dimension mismatch')
                if d.get('validity_event') and (source['kind']!='action' or d['validity_event'] not in {
                        'pause','input_loss','clutch_release','rebind','clock_epoch_change','source_restart'} or d.get('source_epoch') is None):
                    raise ValueError('Invalid source termination evidence')
                if d.get('joint_units')!=[axes[n]['unit'] for n in names]:
                    d['valid']=False
                    d.setdefault('invalid_reasons',[]).append('joint_units_mismatch')
            d.setdefault('units',{name:rule['units'] for name,rule in source['fields'].items()})
            d.setdefault('coordinate_frame',{name:rule['coordinate_frame'] for name,rule in source['fields'].items()})
            for key,attribute in (('units','units'),('coordinate_frame','coordinate_frame')):
                if d[key]!={name:rule[attribute] for name,rule in source['fields'].items()}:
                    d['valid']=False
                    d.setdefault('invalid_reasons',[]).append(key+'_mismatch')
        if source['kind']=='action':
            if d.get('action_stage')!='sent':
                raise ValueError('动作必须来自实际发送链路，action_stage=sent')
            time_field = 'effective_time_ns' if d.get('effective_time_ns') is not None else 'send_time_ns'
            integer(d[time_field],time_field)
            d['action_time_field']=time_field
            d['action_time_quality']='effective' if time_field=='effective_time_ns' else 'send_time_approximation'
            d.setdefault('command_valid',True)
            if nonfinite or not d.get('valid',True):
                d['command_valid']=False
            if type(d['command_valid']) is not bool:
                raise ValueError('command_valid 必须为布尔值')
            if d.get('valid_until_ns') is not None:
                integer(d['valid_until_ns'],'valid_until_ns')
        return Sample(ident,d['source_seq'],capture,d['receive_time_ns'],arrival_time_ns,d['clock_epoch'],freeze_json(d))
    integer(d['pair_seq'],'pair_seq')
    if d.get('trigger_cycle_id') is not None:
        integer(d['trigger_cycle_id'],'trigger_cycle_id')
    depth_enabled=source.get('depth_enabled',True)
    if not depth_enabled and (depth is not None or 'depth' in d):
        raise ValueError('RGB-only source unexpectedly supplied depth')
    rgb_array,depth_array = np.asarray(rgb),np.asarray(depth) if depth_enabled else None
    if rgb_array.dtype!=np.uint8 or rgb_array.ndim!=3 or rgb_array.shape[2]!=3:
        raise ValueError('RGB 必须是 H×W×3 uint8 RGB 顺序')
    if depth_enabled and (depth_array.ndim!=2 or depth_array.dtype.kind not in 'uif' or depth_array.dtype.itemsize not in (2,4,8)):
        raise ValueError('深度必须为原始二维数值数组，禁止彩色化深度替代')
    arrays=[('rgb',rgb_array)]+([('depth',depth_array)] if depth_enabled else [])
    for name,array in arrays:
        image=d[name]
        for key in ('source_seq','source_timestamp_ns','clock_epoch'):
            integer(image[key],name+'.'+key)
        if not normalized:
            _, imodel=clocks.stamp(image)
            image['clock_uncertainty_ns']=imodel.uncertainty_ns
            image['clock_evidence']=imodel.evidence
        else:
            image.setdefault('clock_uncertainty_ns',None)
            image.setdefault('clock_evidence','')
        exposure=image.get('actual_exposure_us')
        if exposure is not None and (not isinstance(exposure,(int,float)) or isinstance(exposure,bool) or not math.isfinite(exposure) or exposure<=0):
            raise ValueError('实际曝光必须为正有限微秒或 null')
        if normalized:
            midpoint=integer(image['capture_time_ns'],name+'.capture_time_ns',1)
            if image.get('timestamp_semantics') not in {'exposure_midpoint','ros_driver_header'} or image.get('header_timestamp_ns')!=midpoint:
                raise ValueError(name+': 必须提供上游标准化曝光中点和一致的 Header')
        else:
            midpoint=exposure_midpoint(image['source_timestamp_ns'],image.get('timestamp_semantics'),exposure,imodel) if exposure is not None else None
        # Never trust a caller supplied midpoint over the timestamp semantics.
        if image.get('exposure_midpoint_ns',midpoint)!=midpoint:
            raise ValueError(name+': 声明中点与曝光语义计算结果不同')
        if name=='depth' and image.get('source_profile') is not None:
            if image.get('shape',list(array.shape))!=list(array.shape) or image.get('dtype',array.dtype.str)!=array.dtype.str:
                image['declared_layout']={'shape':image.get('shape'),'dtype':image.get('dtype')}
                image.setdefault('contract_errors',[]).append('depth_declared_layout_mismatch')
        image.update(exposure_midpoint_ns=midpoint,shape=list(array.shape),dtype=array.dtype.str)
        from .exposure_association import exposure_errors
        errors=exposure_errors(image,source)
        if errors:
            d['valid']=False
            d.setdefault('invalid_reasons',[]).extend(name+':'+error for error in errors)
        if image.get('timestamp_quality') not in {'hardware_synced','mapped','ros_stamped','ros_driver_header','host_receive','unavailable'}:
            raise ValueError(name+': 缺少时间质量')
        for key in ('exposure_uncertainty_us','grid_uncertainty_ns'):
            if image.get(key) is not None and (not isinstance(image[key],(int,float)) or isinstance(image[key],bool) or not math.isfinite(image[key]) or image[key]<0):
                raise ValueError(name+'.'+key+': 未知用 null，已知必须为非负误差界')
        image.setdefault('temporal_support',{'model':'unavailable'})
        image.setdefault('uncertainty_evidence','')
    if depth_enabled:
        scale=d['depth'].get('depth_scale')
        depth_profile=d['depth'].get('source_profile')
        if isinstance(depth_profile,dict) and depth_profile.get('registered_to_rgb') is True and depth_array.shape!=rgb_array.shape[:2]:
            d['depth'].setdefault('contract_errors',[]).append('depth_registration_shape_mismatch')
        depth_status=inspect_depth(d['depth'],depth_array)
        if d['depth'].get('contract_errors'):
            depth_status.update(status='INVALID',errors=depth_status['errors']+d['depth']['contract_errors'])
        d['depth']['source_contract']=depth_status
        if depth_status['profile_sha256'] is not None:
            d['depth'].setdefault('source_profile_sha256',depth_status['profile_sha256'])
        if not allow_unknown_depth and d['depth'].get('source_profile') is not None and depth_status['status']!='DECLARED':
            raise ValueError('深度来源声明与实际数据不一致: '+','.join(depth_status['errors']))
        if not allow_unknown_depth and (not isinstance(scale,(int,float)) or isinstance(scale,bool) or not math.isfinite(scale) or scale<=0 or 'invalid_value' not in d['depth']):
            raise ValueError('必须提供深度 scale 和 invalid_value 定义')
    # Cache reference time can be approximate for diagnostics, but never passes strict quality.
    capture=d['rgb']['exposure_midpoint_ns'] if d['rgb']['exposure_midpoint_ns'] is not None else capture
    d['capture_time_ns']=capture
    return RGBDSample(ident,d['source_seq'],capture,d['receive_time_ns'],arrival_time_ns,d['clock_epoch'],
        freeze_json(d),rgb_array.tobytes(order='C'),depth_array.tobytes(order='C') if depth_enabled else b'')
