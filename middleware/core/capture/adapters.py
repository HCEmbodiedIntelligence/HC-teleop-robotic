"""Device-independent upstream readiness checks; hardware stays in camera nodes."""
import time


def preflight(cfg,reports,clocks):
    issues=[]
    if not any(v['required'] and v['kind']!='event' for v in cfg['sources'].values()):
        issues.append({'source_id':'','code':'required_sources_missing','reason':'请至少配置一个必需状态或 RGB-D 来源'})
    for ident,source in cfg['sources'].items():
        if not source['required'] or source['kind']=='event':
            continue
        report=reports.get(ident,{})
        missing=[]
        if cfg['clock']['domain']=='ros':
            if source['kind']=='rgbd' and source.get('require_exposure_metadata'):
                from .exposure_association import exposure_errors
                for stream in (('rgb','depth') if source.get('depth_enabled',True) else ('rgb',)):
                    if exposure_errors(report.get(stream,{}),source):
                        missing.append(stream+' 缺少有效的逐帧曝光中点/实际曝光元数据，或曝光超过配置上限')
            test_action_fallback = (bool(cfg.get('capture')) and source['kind']=='action' and source.get('named_axes')
                                    and cfg['alignment'].get('derive_action_from_next_state',False))
            if source['transport'].startswith('ros_') and source['kind']!='pointcloud' and not test_action_fallback and time.monotonic_ns()-report.get('registered_monotonic_ns',0)>3_000_000_000:
                missing.append('尚未收到来源数据，请检查相机/机器人节点和 ROS 话题')
            if cfg['alignment']['strict'] and source['kind'] in {'state','action'}:
                accepted={'read','sample'} if source['kind']=='state' else {'send','effective'}
                if source.get('timestamp_semantics') not in accepted:
                    missing.append('请声明 ROS Header 的读数/发送/生效语义')
            issues.extend({'source_id':ident,'code':'source_missing','reason':x} for x in missing)
            continue
        if cfg['alignment']['strict']:
            if source['transport']=='ros_joint_state':
                missing.append('普通 JointState 接口只有接收时间近似；严格采集需要带已验证时钟映射的消息接口')
            if report.get('timestamp_quality') not in {'mapped','hardware_synced'}:
                missing.append('真实采样时间戳及其语义')
            if not report.get('clock_model_ids') or any(x not in clocks.models or clocks.models[x].uncertainty_ns is None or not clocks.models[x].evidence for x in report.get('clock_model_ids',[])):
                missing.append('有测量依据及误差界的时钟映射版本')
            if source['kind']=='rgbd':
                for key,label in (
                    ('exposure_readback','RGB / 深度分别配置后的曝光回读'),
                    ('sync_evidence_version','经验证的 RGB-D 同步曝光依据'),
                    ('camera_config_id','相机配置版本'),('calibration_id','内外参及深度标定版本'),
                    ('uncertainty_evidence','曝光、相对时间及网格时间误差界的依据'),
                    ('temporal_support','快门 / 深度测量时间覆盖模型')):
                    if not report.get(key):
                        missing.append(label)
                if source.get('trigger_origin') is not None and (report.get('trigger_grid_mapping_verified') is not True or report.get('trigger_cycle_fps')!=cfg['dataset']['fps']):
                    missing.append('触发周期到目标网格的验证映射及周期频率')
                readback=report.get('exposure_readback',{})
                for stream in (('rgb','depth') if source.get('depth_enabled',True) else ('rgb',)):
                    exposure=readback.get(stream,{})
                    if exposure.get('auto') is not False or exposure.get('requested_us')!=source['exposure'][stream+'_requested_us']:
                        missing.append(stream+' 手动曝光配置回执不匹配')
                    actual,bound=exposure.get('actual_us'),exposure.get('uncertainty_us')
                    if not isinstance(actual,(int,float)) or not isinstance(bound,(int,float)) or min(actual,bound)<0 or actual+bound>source['exposure']['max_actual_us']:
                        missing.append(stream+' 实际曝光及误差上界 ≤5000 μs 的证据')
                if report.get('sync_mode')!=source['sync_mode'] or report.get('sync_verified') is not True:
                    missing.append('配置所选同步模式的验证结果')
                if report.get('measured_new_frame_fps',0)<cfg['dataset']['fps'] and not source.get('allow_camera_repeat',cfg['alignment']['allow_camera_repeat']):
                    missing.append('满足 dataset.fps 的实测新帧频率')
        if source['transport']=='sdk' and not report.get('adapter_connected'):
            missing.append('相机 SDK 适配器尚未连接')
        issues.extend({'source_id':ident,'code':'capability_missing','reason':x} for x in missing)
    return {'ready':not issues,'strict':cfg['alignment']['strict'],'issues':issues,
        'limits_status':cfg['requirement_status'],'reports':reports}
