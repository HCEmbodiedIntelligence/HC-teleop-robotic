"""Configuration is a contract, not evidence of hardware synchronization."""
import copy
import hashlib
import json
import math
import re

from . import ALIGNMENT_VERSION, SCHEMA_VERSION, LEROBOT_VERSION

DEFAULT = {
    'capture': {},
    'clock': {'domain':'ros','jump_tolerance_ms':5},
    'dataset': {'fps': 30, 'duration_seconds': None},
    'alignment': {'camera_validation':'timestamps','strict': True, 'delay_ms': 80, 'camera_max_error_ms': 1,
        'rgbd_max_midpoint_skew_ms': 1, 'camera_pair_max_skew_ms': 1,
        'joint_max_bracket_gap_ms': 30, 'allow_camera_repeat': False,
        'causal': False, 'primary_camera': '', 'max_catchup_rows': 8},
    'buffers': {'ros_assembly_bytes':64*1024*1024,'retention_ms': 250, 'cache_items_per_source': 256,
        'working_bytes': 64*1024*1024, 'cache_bytes': 64*1024*1024, 'ingress_items': 512, 'ingress_bytes': 128*1024*1024,
        'writer_items': 4096, 'writer_bytes': 128*1024*1024,
        'video_frames_per_camera': 30, 'video_bytes_per_camera': 64*1024*1024,
        'total_bytes': 1024*1024*1024},
    'storage': {'directory': 'sessions', 'video_segment_seconds': 60,
        'numeric_flush_seconds': 1, 'fsync_on_commit': True,
        'minimum_free_bytes': 512*1024*1024, 'encoder_threads': 1},
    'sources': {},
    'requirement_status': {'exposure_max_actual_us': {'value':5000, 'status':'required'},
        'synchronization_and_grid_limits': 'provisional_engineering_targets_pending_device_validation'},
    'versions': {'alignment': ALIGNMENT_VERSION, 'schema': SCHEMA_VERSION, 'lerobot': LEROBOT_VERSION},
}


def merge(base, value):
    result = copy.deepcopy(base)
    for key, item in value.items():
        result[key] = merge(result[key], item) if isinstance(item,dict) and isinstance(result.get(key),dict) else copy.deepcopy(item)
    return result


def number(value, name, low=0, high=1e15, integer=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low <= value <= high or (integer and type(value) is not int):
        raise ValueError(f'{name}: 必须为 {low}–{high} 范围内的'+('整数' if integer else '有限数值'))
    return value


def validate(value):
    if not isinstance(value,dict):
        raise ValueError('采集配置必须是对象')
    if set(value)-set(DEFAULT):
        raise ValueError('未知采集配置项: '+', '.join(sorted(set(value)-set(DEFAULT))))
    cfg = merge(DEFAULT,value)
    for section in DEFAULT:
        if not isinstance(cfg[section],dict):
            raise ValueError(f'{section} 必须为对象')
    if cfg['clock']['domain'] not in {'ros','host_monotonic'}:
        raise ValueError('clock.domain 必须为 ros 或旧版诊断 host_monotonic')
    number(cfg['clock']['jump_tolerance_ms'],'clock.jump_tolerance_ms',.01,1000)
    number(cfg['dataset']['fps'],'dataset.fps',1,240,True)
    if cfg['dataset']['duration_seconds'] is not None:
        number(cfg['dataset']['duration_seconds'],'dataset.duration_seconds',.01,86400)
    if len(cfg['sources'])>32:
        raise ValueError('最多支持 32 个采集来源')
    if cfg['alignment']['camera_validation']=='driver_headers':
        cfg['alignment']['camera_validation']='timestamps'
    if cfg['alignment']['camera_validation'] not in {'timestamps','strict'}:
        raise ValueError('不支持的相机时间关联策略')
    a,b,s = cfg['alignment'],cfg['buffers'],cfg['storage']
    if type(s.get('depth_video_encoding',False)) is not bool:
        raise ValueError('storage.depth_video_encoding 必须为布尔值')
    if type(s.get('retain_original_ros',False)) is not bool:
        raise ValueError('storage.retain_original_ros 必须为布尔值')
    # Omitted means disabled; do not rewrite the hashes of historical snapshots.
    if type(a.get('derive_action_from_next_state',False)) is not bool:
        raise ValueError('derive_action_from_next_state 必须为布尔值')
    if a.get('timeline_mode','fixed_grid') not in {'fixed_grid','primary_camera'}:
        raise ValueError('不支持的录制时间轴')
    if type(a.get('require_camera_sync',False)) is not bool:
        raise ValueError('require_camera_sync 必须为布尔值')
    if 'camera_match_max_skew_ms' in a:
        number(a['camera_match_max_skew_ms'],'camera_match_max_skew_ms',.001,500/cfg['dataset']['fps']+1e-6)
    if a.get('timeline_mode')=='primary_camera' and (
            a.get('state_reference')!='primary_exposure_midpoint' or
            a.get('camera_timing_reference')!='per_camera_rgb'):
        raise ValueError('主相机时间轴必须使用真实曝光中点关联')
    if a.get('camera_timing_reference','grid') not in {'grid','per_camera_rgb'}:
        raise ValueError('不支持的相机时间检查基准')
    if a.get('state_reference','grid') not in {'grid','primary_exposure_midpoint'}:
        raise ValueError('不支持的关节时间基准')
    if a.get('state_reference')=='primary_exposure_midpoint' and not a.get('primary_camera'):
        raise ValueError('曝光中点关联必须指定主相机')
    for key in ('strict','allow_camera_repeat','causal'):
        if type(a[key]) is not bool:
            raise ValueError(key+' 必须为布尔值')
    for key in ('delay_ms','camera_max_error_ms','rgbd_max_midpoint_skew_ms','camera_pair_max_skew_ms','joint_max_bracket_gap_ms'):
        number(a[key],key,0,10000)
    number(a['max_catchup_rows'],'max_catchup_rows',1,1000,True)
    for key in b:
        number(b[key],key,1,16*1024**3,True)
    if b['retention_ms'] < a['delay_ms']+a['camera_max_error_ms']+a['joint_max_bracket_gap_ms']:
        raise ValueError('缓存时长必须覆盖对齐等待、相机窗口与关节插值区间')
    for key in ('video_segment_seconds','numeric_flush_seconds'):
        number(s[key],key,.01,3600)
    number(s['encoder_threads'],'encoder_threads',1,4,True)
    number(s['minimum_free_bytes'],'minimum_free_bytes',0,1e15,True)
    if type(s['fsync_on_commit']) is not bool or not isinstance(s['directory'],str) or not s['directory'].strip():
        raise ValueError('保存目录或持久化设置无效')
    if 'export_directory' in s and (not isinstance(s['export_directory'],str) or not s['export_directory'].strip()):
        raise ValueError('上传包保存目录无效')
    number(a.get('next_observation_max_gap_ms',1000),'next_observation_max_gap_ms',1,10000)
    cameras = 0
    for ident, source in cfg['sources'].items():
        if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]{0,63}',ident) or not isinstance(source,dict):
            raise ValueError('来源 ID 只允许英文字母、数字和下划线，且以字母开头')
        kind = source.get('kind')
        if kind not in {'rgbd','state','action','event','pointcloud'}:
            raise ValueError(ident+': kind 必须为 rgbd/state/action/event/pointcloud')
        source.setdefault('required',kind!='event')
        if type(source['required']) is not bool:
            raise ValueError(ident+': required 必须为布尔值')
        source.setdefault('transport','envelope')
        if source['transport'] not in {'sdk','envelope','ros_joint_state','ros_rgbd','ros_pointcloud','ros_images'}:
            raise ValueError(ident+': 不支持的来源接口')
        if source['transport']=='ros_joint_state' and kind not in {'state','action'}:
            raise ValueError('ROS JointState 接口只适用于状态/动作')
        if 'packet_topic' in source and (source['transport']!='ros_rgbd' or not isinstance(source['packet_topic'],str)
                or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*',source['packet_topic'])):
            raise ValueError(ident+': 原子相机消息话题无效')
        if source['transport'] in {'ros_rgbd','ros_pointcloud'} and not source.get('packet_topic') and not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*',source.get('metadata_topic','')):
            raise ValueError(ident+': 缺少带来源序号的 metadata_topic')
        if 'original_topics' in source:
            from .originals import STREAM_TYPES
            topics=source['original_topics']
            expected={'rgb','depth','rgb_meta','depth_meta','rgb_info','depth_info'} if source.get('depth_enabled',True) else {'rgb','rgb_meta','rgb_info'}
            bundled={'rgb_frame','depth_frame'} if source.get('depth_enabled',True) else {'rgb_frame'}
            if (kind!='rgbd' or source['transport']!='ros_rgbd' or not isinstance(topics,dict)
                    or set(topics) not in (expected,bundled) or any(not isinstance(t,str) or
                    not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*',t) for t in topics.values())
                    or len(set(topics.values()))!=len(topics)):
                raise ValueError(ident+': 原始相机话题配置无效')
        if source['transport']=='ros_images':
            for key in ('rgb_topic','depth_topic'):
                if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*',source.get(key,'')):
                    raise ValueError(ident+': 缺少 '+key)
        if source['transport'] in {'ros_joint_state','ros_rgbd','ros_pointcloud'}:
            if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*',source.get('topic','')):
                raise ValueError(ident+': 缺少绝对 ROS 话题')
            if kind=='action' and source.get('action_stage')!='sent':
                raise ValueError(ident+': 动作话题必须明确声明 action_stage=sent（实际发送值）')
        if source['transport'] in {'ros_rgbd','ros_images'} and kind!='rgbd' or source['transport']=='ros_pointcloud' and kind!='pointcloud':
            raise ValueError(ident+': ROS 接口与来源类型不匹配')
        if cfg['clock']['domain']=='ros' and source['transport']=='sdk':
            raise ValueError('相机 SDK 必须在独立相机节点运行，请订阅 ros_rgbd')
        if source['transport']=='ros_joint_state':
            source.setdefault('timestamp_semantics','unavailable')
            source.setdefault('timestamp_uncertainty_ns',None)
            source.setdefault('timestamp_evidence','')
        if kind=='pointcloud':
            source.setdefault('persist',True)
            if type(source['persist']) is not bool or not source.get('camera_source_id'):
                raise ValueError('点云必须声明 camera_source_id 和 persist')
            if source['required'] and not source['persist']:
                raise ValueError('必需点云须持久化；离线重建请配置为可选来源')
        if kind=='rgbd':
            if type(source.get('depth_enabled',True)) is not bool:
                raise ValueError('depth_enabled 必须为布尔值')
            if not source.get('depth_enabled',True) and source['transport']=='ros_images':
                raise ValueError('RGB-only 需要标准化 ros_rgbd 或 envelope 输入')
            for key in ('camera_max_error_ms','rgbd_max_midpoint_skew_ms'):
                if key in source:
                    number(source[key],ident+'.'+key,.000001,1000)
            if 'allow_camera_repeat' in source and type(source['allow_camera_repeat']) is not bool:
                raise ValueError(ident+'.allow_camera_repeat 必须为布尔值')
            if 'require_exposure_metadata' in source and type(source['require_exposure_metadata']) is not bool:
                raise ValueError('require_exposure_metadata 必须为布尔值')
            if source.get('depth_source') is not None:
                from .depth_source import validate_profile
                validate_profile(source['depth_source'])
            cameras += 1
            number(source.get('fps',30),ident+'.fps',1,240)
            source.setdefault('fps',30)
            if source['required'] and cfg['dataset']['fps']>source['fps'] and not source.get('allow_camera_repeat',a['allow_camera_repeat']):
                raise ValueError(ident+': 输出 FPS 高于相机新帧频率，禁止用旧帧补足')
            exposure_defaults={'max_actual_us':5000,'auto':True,'auto_exposure_limit_us':4500,'auto_gain_limit':64} if cfg['clock']['domain']=='ros' else {'max_actual_us':5000,'rgb_requested_us':4000,'depth_requested_us':4000,'auto':False}
            source['exposure'] = merge(exposure_defaults,source.get('exposure',{}))
            if cfg['clock']['domain']=='ros':
                for old_key in ('rgb_requested_us','depth_requested_us','startup_exposure_us','startup_gain'):
                    source['exposure'].pop(old_key,None)
            for field in (('max_actual_us',) if cfg['clock']['domain']=='ros' else ('max_actual_us','rgb_requested_us','depth_requested_us')):
                number(source['exposure'][field],ident+'.exposure.'+field,.01,5000)
            if source['exposure']['auto']:
                number(source['exposure'].get('auto_exposure_limit_us',4500),'auto_exposure_limit_us',1,5000)
                number(source['exposure'].get('auto_gain_limit',64),'auto_gain_limit',1,10000)
            source.setdefault('sync_mode','shared_stereo_free_run' if cfg['clock']['domain']=='ros' else 'hardware_trigger')
            source.setdefault('trigger_origin',None)
            if source['trigger_origin'] is not None:
                number(source['trigger_origin'],'trigger_origin',0,2**63-1,True)
            source.setdefault('temporal_fusion',False)
            if source['temporal_fusion']:
                raise ValueError('当前严格实现不支持跨帧融合；请保留原始单次采集')
        if kind in {'state','action'}:
            fields = source.setdefault('fields',{'position':{'semantics':'continuous','units':'rad','coordinate_frame':'joint'}})
            if not fields or not isinstance(fields,dict):
                raise ValueError(ident+': 必须声明数值字段')
            for name, rule in fields.items():
                if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',name) or not isinstance(rule,dict):
                    raise ValueError(ident+': 字段定义无效')
                semantics = rule.get('semantics','continuous')
                if semantics not in {'continuous','angle','quaternion','discrete'}:
                    raise ValueError(ident+'.'+name+': 高频/专用字段需要经验证的专用重采样器，当前不支持')
                rule.setdefault('semantics',semantics)
                if not rule.get('units') or not rule.get('coordinate_frame'):
                    raise ValueError(ident+'.'+name+': 必须声明单位及坐标系')
                if semantics=='angle':
                    number(rule.get('max_speed'),'angle.max_speed',.000001,100000)
                    number(rule.setdefault('period',2*math.pi),'angle.period',.000001,100000)
                if rule.get('names') is not None and (not isinstance(rule['names'],list) or not rule['names'] or any(not isinstance(x,str) or not x for x in rule['names']) or len(set(rule['names']))!=len(rule['names'])):
                    raise ValueError(ident+'.'+name+': names 必须为非空且唯一的字段名称列表')
                if semantics=='discrete':
                    number(rule.setdefault('max_age_ms',250),'max_age_ms',0,3600000)
            if kind=='action':
                source.setdefault('pairing','active_at_target')
                source.setdefault('semantics','absolute_hold')
                if source['pairing']!='active_at_target' or source['semantics']!='absolute_hold':
                    raise ValueError(ident+': 当前仅实现 active_at_target 绝对保持命令；增量转换与 cycle_linked 不可静默替代')
                if source.get('max_age_ms') is not None:
                    number(source['max_age_ms'],'action.max_age_ms',0,3600000)
            if 'named_axes' in source:
                axes=source['named_axes']
                if not axes or set(fields)!={'position'} or len({x['name'] for x in axes})!=len(axes):
                    raise ValueError('named_axes requires unique axes and position only')
                for axis in axes:
                    if axis['unit'] not in {'rad','m'}:
                        raise ValueError('named axis unit must be rad or m')
                    number(axis['max_age_ms'],'axis.max_age_ms',1,3600000)
                    if axis.get('mode','periodic') not in {'periodic','on_change'}:
                        raise ValueError('unsupported command validity mode')
    camera_window=max([a['camera_max_error_ms']]+[v.get('camera_max_error_ms',a['camera_max_error_ms'])+v.get('rgbd_max_midpoint_skew_ms',a['rgbd_max_midpoint_skew_ms']) for v in cfg['sources'].values() if v['kind']=='rgbd'])
    if a.get('camera_timing_reference')=='per_camera_rgb' and b['retention_ms'] < a['delay_ms']+camera_window+a['joint_max_bracket_gap_ms']:
        raise ValueError('缓存时长必须覆盖每台相机的时间窗口与关节插值区间')
    for ident,source in cfg['sources'].items():
        if source['kind']=='pointcloud' and (cfg['sources'].get(source['camera_source_id'],{}).get('kind')!='rgbd' or not cfg['sources'][source['camera_source_id']].get('depth_enabled',True)):
            raise ValueError(ident+': 点云来源必须引用已配置的 RGB-D 组')
    if a['primary_camera'] and a['primary_camera'] not in [k for k,v in cfg['sources'].items() if v['kind']=='rgbd' and v['required']]:
        raise ValueError('主相机必须是已配置的必需 RGB-D 来源')
    reserved = b['ros_assembly_bytes']+b['working_bytes']+b['cache_bytes']+b['ingress_bytes']+b['writer_bytes']+cameras*b['video_bytes_per_camera']
    original_topics=[topic for source in cfg['sources'].values() for topic in source.get('original_topics',{}).values()]
    if len(original_topics)!=len(set(original_topics)):
        raise ValueError('原始相机话题不能由多个来源重复保存')
    if original_topics and s.get('retain_original_ros',False):
        reserved+=b['ros_assembly_bytes']
    if reserved>b['total_bytes']:
        raise ValueError(f'各队列与缓存预算合计 {reserved} 字节超过总预算 {b["total_bytes"]}')
    cfg['versions']=copy.deepcopy(DEFAULT['versions'])
    cfg['requirement_status']=copy.deepcopy(DEFAULT['requirement_status'])
    if cfg['capture']:
        from .profile import validate_capture_config
        validate_capture_config(cfg)
    return cfg


def config_hash(cfg):
    return hashlib.sha256(json.dumps(cfg,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
