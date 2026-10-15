"""Read-only normalized ROS inputs. No SDK, device clock math or camera controls."""
import json
import math
import threading
import time

import numpy as np

from .models import CONTRACT


def header_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def decode_image(msg):
    if msg.encoding not in {'rgb8','16UC1','32FC1','mono16'}:
        raise ValueError('Unsupported raw image encoding: '+msg.encoding)
    dtype=np.dtype('u1' if msg.encoding=='rgb8' else ('f4' if msg.encoding=='32FC1' else 'u2'))
    dtype=dtype.newbyteorder('>' if msg.is_bigendian else '<')
    channels=3 if msg.encoding=='rgb8' else 1
    row_bytes=msg.width*channels*dtype.itemsize
    if msg.height<1 or msg.width<1 or msg.step<row_bytes or len(msg.data)!=msg.height*msg.step:
        raise ValueError('ROS image layout/length mismatch')
    shape=(msg.height,msg.width,channels) if channels==3 else (msg.height,msg.width)
    strides=(msg.step,channels*dtype.itemsize,dtype.itemsize) if channels==3 else (msg.step,dtype.itemsize)
    return np.ndarray(shape,dtype=dtype,buffer=msg.data,strides=strides)


def joint_document(ident,source,message,seq,receive_ros,receive_steady,epoch=0,*,evidence=None):
    capture=header_ns(message.header)
    if capture<=0:
        raise ValueError('JointState.header.stamp is uninitialized')
    payload={}
    named='named_axes' in source
    if named and (not message.name or len(set(message.name))!=len(message.name)
                  or len(message.position)!=len(message.name)):
        raise ValueError('Named JointState requires unique names and matching positions')
    selected=[name for name in message.name if name in {a['name'] for a in source.get('named_axes',[])}]
    if named and len(selected)!=len(message.name):
        raise ValueError('Joint identity changed; resolve a new capture configuration/session')
    if named and not selected:
        raise ValueError('JointState contains no configured axes')
    for field,rule in source['fields'].items():
        names=rule.get('names')
        values=list(getattr(message,field,[]))
        if named:
            values=[values[message.name.index(name)] for name in selected]
        elif names:
            values=[values[message.name.index(name)] for name in names]
        payload[field]=values
    d={'input_contract':CONTRACT,'capture_clock_id':'ros','header_timestamp_ns':capture,
        'source_id':ident,'source_seq':seq,'source_timestamp_ns':capture,'capture_time_ns':capture,
        'receive_time_ns':receive_ros,'receive_clock_id':'ros','receive_steady_ns':receive_steady,
        'clock_id':'ros','clock_epoch':epoch,'ros_clock_epoch':epoch,'clock_model_id':'ros_header_direct',
        'timestamp_quality':'ros_stamped','clock_uncertainty_ns':source.get('timestamp_uncertainty_ns'),
        'clock_evidence':source.get('timestamp_evidence',''),'timestamp_semantics':source.get('timestamp_semantics','unavailable'),
        'known_delay_ns':source.get('known_delay_ns'),'payload':payload,
        'ros_header_timestamp_ns':capture,'ros_header_frame_id':message.header.frame_id,'joint_names':list(message.name)}
    if named:
        d['joint_names']=selected
        d['joint_units']=[next(a['unit'] for a in source['named_axes'] if a['name']==n) for n in selected]
        d.update(validity_known=False,source_epoch=None)
        if evidence is not None:
            if set(evidence)-{'validity_known','source_epoch','hold_leases','command_valid','continuity_id'}:
                raise ValueError('Unknown command evidence field')
            d.update(evidence)
    if source['kind']=='action':
        if source.get('timestamp_semantics') not in {'send','effective'}:
            raise ValueError('Action header must represent send or effective time')
        d.update(action_stage='sent',control_mode=source.get('control_mode'))
        d['effective_time_ns' if source['timestamp_semantics']=='effective' else 'send_time_ns']=capture
    return d


def validity_document(ident,source,names,reason,stamp,seq,receive_ros,receive_steady,epoch,source_epoch):
    """E supplies termination events with producer sequence and new control epoch."""
    from types import SimpleNamespace
    if source['kind']!='action' or reason not in {'pause','input_loss','clutch_release','rebind','clock_epoch_change','source_restart'}:
        raise ValueError('Unknown validity event')
    if not names or set(names)-{a['name'] for a in source['named_axes']} or source_epoch is None:
        raise ValueError('Invalid event axes or source epoch')
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp//10**9,nanosec=stamp%10**9),frame_id=''),
                        name=names,position=[0.] * len(names))
    d=joint_document(ident,source,msg,seq,receive_ros,receive_steady,epoch)
    d.update(validity_event=reason,source_epoch=source_epoch,validity_known=True,command_valid=False)
    d['payload']['position']=[]
    return d


def action_evidence_document(ident,source,item,previous,receive_ros,receive_steady,epoch):
    """Normalize one producer command/event; reject gaps before recording a claim."""
    from sensor_msgs.msg import JointState
    if item.get('schema')!='openarm-action-evidence/v1' or item.get('kind') not in {'command','event'}:
        raise ValueError('Unknown action producer evidence')
    sequence=item['sequence'];stamp=item['stamp_ns'];source_epoch=item['source_epoch']
    names=item['names']
    if (type(sequence) is not int or sequence<1 or type(stamp) is not int or stamp<=0
            or not isinstance(source_epoch,str) or not source_epoch or not isinstance(names,list)
            or not names or len(set(names))!=len(names)
            or set(names)-{a['name'] for a in source['named_axes']}):
        raise ValueError('Malformed action identity')
    if previous and sequence!=previous[0]+1:
        raise ValueError('Action producer stream gap or sequence reset')
    if previous and item['kind']=='command' and source_epoch!=previous[1]:
        raise ValueError('Action epoch changed without termination event')
    if item['kind']=='event':
        if previous and source_epoch==previous[1]:
            raise ValueError('Termination event did not fence old control epoch')
        document=validity_document(ident,source,names,item['reason'],stamp,
            sequence,receive_ros,receive_steady,epoch,source_epoch)
    else:
        positions=item['positions']
        if (not isinstance(positions,list) or len(positions)!=len(names) or
                any(isinstance(value,bool) or not isinstance(value,(int,float)) or
                    not math.isfinite(value) for value in positions)):
            raise ValueError('Action positions do not match logical names')
        joint=JointState()
        joint.header.stamp.sec,joint.header.stamp.nanosec=divmod(stamp,10**9)
        joint.name=names;joint.position=positions
        document=joint_document(ident,source,joint,sequence,receive_ros,receive_steady,
            epoch,evidence={'validity_known':True,'source_epoch':source_epoch})
    return document,(sequence,source_epoch,receive_steady,item['kind']=='command')


class RosSources:
    def __init__(self,hub,ros_config):
        self.hub,self.config=hub,ros_config
        self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self.run,name='normalized-ros-inputs',daemon=True)
        self.error=None
        self.rejected=0

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(5)

    def active_session(self):
        s=self.hub.session
        return s if s and not s.closed.is_set() else None

    def input_sessions(self):
        if hasattr(self.hub,'input_sessions'):
            return self.hub.input_sessions()
        session=self.active_session()
        return (session,) if session else ()

    def run(self):
        context=node=executor=None
        try:
            from collections import OrderedDict
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState, Image, CameraInfo, PointCloud2
            from std_msgs.msg import String
            from realsense2_camera_msgs.msg import RGBD, Metadata
            if any(s.get('packet_topic') or 'rgb_frame' in s.get('original_topics',{})
                   for s in self.hub.cfg['sources'].values()):
                from realsense2_camera_msgs.msg import RGBDMetadata, ImageMetadata
            from .ros_images import HeaderPairs
            from .originals import STREAM_TYPES
            from rclpy.serialization import serialize_message
            from humanoid_camera.transport import CaptureSubscription
            context=Context()
            rclpy.init(args=[],context=context,domain_id=self.config['domain_id'])
            node=rclpy.create_node('humanoid_normalized_sources',context=context)
            executor=SingleThreadedExecutor(context=context)
            executor.add_node(node)
            seq={};pending=OrderedDict();pending_bytes=0
            capture_subscriptions=[]
            budget=self.hub.cfg['buffers'].get('ros_assembly_bytes',64*1024*1024)
            def report(ident,data=None):
                self.hub.reports[ident]={**(data or {}),'ready':True,'capture_clock_id':'ros',
                    'registered_monotonic_ns':time.monotonic_ns()}
            def rejected(ident,error):
                self.rejected+=1
                for session in self.input_sessions():
                    session.counters['ros_contract_rejected:'+ident]+=1
                    session.storage.submit('gap',{'source_id':ident,'reason':'input_contract_rejected','detail':str(error)})
                    if session.cfg['capture'] and 'named_axes' in session.cfg['sources'][ident]:
                        session.fail('named_ros_input_contract_rejected:'+ident)
            def original(topic,data,receive_ns=None):
                sessions=self.input_sessions()
                if not any(s.originals for s in sessions):return
                now=node.get_clock().now().nanoseconds if receive_ns is None else receive_ns
                for session in sessions:
                    session.record_original(topic,data,now)
            def offer(ident,source,message,data):
                if data.get('source_id')!=ident or data.get('input_contract')!=CONTRACT or header_ns(message.header)!=data.get('capture_time_ns'):
                    raise ValueError('Normalized envelope/header mismatch')
                if source['kind']=='rgbd':
                    for name in (('rgb','depth') if source.get('depth_enabled',True) else ('rgb',)):
                        if header_ns(getattr(message,name).header)!=data[name]['capture_time_ns']:
                            raise ValueError(name+': Header differs from capture time')
                if source['kind']=='rgbd' and not source.get('depth_enabled',True) and ('depth' in data or len(message.depth.data)):
                    raise ValueError('RGB-only source unexpectedly supplied depth')
                report(ident,data)
                sessions=self.input_sessions()
                if not sessions:return
                now=node.get_clock().now().nanoseconds;steady=time.monotonic_ns()
                data.update(receive_time_ns=now,receive_clock_id='ros',receive_steady_ns=steady)
                if source['kind']=='rgbd':
                    for name in (('rgb','depth') if source.get('depth_enabled',True) else ('rgb',)):
                        image=getattr(message,name)
                        if header_ns(image.header)!=data[name]['capture_time_ns']:
                            raise ValueError(name+': Header differs from capture time')
                        data[name]['header_timestamp_ns']=header_ns(image.header)
                    rgb=decode_image(message.rgb)
                    depth=decode_image(message.depth) if source.get('depth_enabled',True) else None
                    for session in sessions:
                        session.observe_ros_clock(now,steady)
                        session.submit(dict(data),rgb,depth)
                else:
                    data['pointcloud_layout']={'width':message.width,'height':message.height,'point_step':message.point_step,
                        'row_step':message.row_step,'is_bigendian':message.is_bigendian,'is_dense':message.is_dense,
                        'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in message.fields]}
                    for session in sessions:
                        session.observe_ros_clock(now,steady)
                        session.submit(dict(data),points=message.data)
            def combine(ident,source,kind,message):
                nonlocal pending_bytes
                try:
                    key=(ident,header_ns(message.header))
                    size=len(message.json_data.encode()) if kind=='metadata' else len(message.data) if source['kind']=='pointcloud' else len(message.rgb.data)+len(message.depth.data)
                    if size>budget:
                        raise ValueError('ROS assembly byte budget exceeded')
                    while pending and (pending_bytes+size>budget or (key not in pending and len(pending)>=256)):
                        oldkey,entry=pending.popitem(last=False);pending_bytes-=sum(x[1] for x in entry.values())
                        rejected(oldkey[0],'ROS paired message/metadata not completed before bounded eviction')
                    entry=pending.setdefault(key,{})
                    if kind in entry:return
                    entry[kind]=(message,size);pending_bytes+=size
                    if len(entry)==2:
                        del pending[key];pending_bytes-=sum(x[1] for x in entry.values())
                        d=json.loads(entry['metadata'][0].json_data)
                        offer(ident,source,entry['message'][0],d)
                except (ValueError,KeyError,TypeError) as error:
                    rejected(ident,error)
            def packet(ident,source,message):
                try:
                    if len(message.rgbd.rgb.data)+len(message.rgbd.depth.data)+len(message.json_data.encode())>budget:
                        raise ValueError('ROS packet byte budget exceeded')
                    offer(ident,source,message.rgbd,json.loads(message.json_data))
                except (ValueError,KeyError,TypeError) as error:
                    rejected(ident,error)
            sources=self.hub.cfg['sources']
            action_streams={}
            def sync_action_sessions():
                active={s.id for s in self.input_sessions()}
                for key in tuple(action_streams):
                    if key not in active:
                        del action_streams[key]
            raw_count=max(1,sum(s['transport']=='ros_images' for s in sources.values()))
            for ident,source in sources.items():
                transport=source['transport']
                seq[ident]=0
                original_topics=(source.get('original_topics',{})
                    if self.hub.cfg['storage'].get('retain_original_ros',False) else {})
                for name,topic in original_topics.items():
                    typ=(ImageMetadata if name in {'rgb_frame','depth_frame'} else
                         {'sensor_msgs/msg/Image':Image,'sensor_msgs/msg/CameraInfo':CameraInfo,
                          'realsense2_camera_msgs/msg/Metadata':Metadata}[STREAM_TYPES[name]])
                    capture_subscriptions.append(CaptureSubscription(node,typ,topic,
                        lambda data,topic=topic:original(topic,data),
                        depth=10 if name in {'rgb','depth','rgb_frame','depth_frame'} else 30,raw=True))
                if transport=='ros_joint_state':
                    def joint(message,ident=ident,source=source):
                        sessions=self.input_sessions()
                        steady=time.monotonic_ns();now=node.get_clock().now().nanoseconds
                        try:
                            if any(s.originals for s in sessions):
                                original(source['topic'],serialize_message(message),now)
                            seq[ident]+=1
                            report(ident,{'timestamp_semantics':source.get('timestamp_semantics'),'timestamp_quality':'ros_stamped'})
                            for session in sessions:
                                session.observe_ros_clock(now,steady)
                                d=joint_document(ident,source,message,seq[ident],now,steady,session.scheduler.epoch)
                                session.submit(d)
                        except (ValueError,KeyError,IndexError,TypeError) as error:rejected(ident,error)
                    node.create_subscription(JointState,source['topic'],joint,qos_profile_sensor_data)
                elif transport=='envelope' and source['kind']=='action' and source.get('evidence_topic'):
                    def action_evidence(message,ident=ident,source=source):
                        sessions=self.input_sessions()
                        sync_action_sessions()
                        now=node.get_clock().now().nanoseconds
                        steady=time.monotonic_ns()
                        try:
                            if any(s.originals for s in sessions):
                                original(source['evidence_topic'],serialize_message(message),now)
                            item=json.loads(message.data)
                            if not sessions:
                                action_evidence_document(ident,source,item,None,now,steady,0)
                            for session in sessions:
                                session.observe_ros_clock(now,steady)
                                streams=action_streams.setdefault(session.id,{})
                                document,streams[ident]=action_evidence_document(
                                    ident,source,item,streams.get(ident),now,steady,session.scheduler.epoch)
                                session.submit(document)
                            report(ident,{'producer_evidence':True,'timestamp_semantics':'send'})
                        except (ValueError,KeyError,IndexError,TypeError) as error:
                            rejected(ident,error)
                    node.create_subscription(String,source['evidence_topic'],action_evidence,10)
                elif transport in {'ros_rgbd','ros_pointcloud'}:
                    if source.get('packet_topic'):
                        capture_subscriptions.append(CaptureSubscription(node,RGBDMetadata,source['packet_topic'],
                            lambda msg,ident=ident,source=source:packet(ident,source,msg),depth=10))
                        continue
                    if not source.get('metadata_topic'):
                        raise ValueError(ident+': metadata_topic is required for source lineage')
                    capture_subscriptions.append(CaptureSubscription(node,Metadata,source['metadata_topic'],lambda msg,ident=ident,source=source:combine(ident,source,'metadata',msg),depth=30))
                    capture_subscriptions.append(CaptureSubscription(node,RGBD if transport=='ros_rgbd' else PointCloud2,source['topic'],lambda msg,ident=ident,source=source:combine(ident,source,'message',msg),depth=2 if transport=='ros_pointcloud' else 10))
                elif transport=='ros_images':
                    pairs=HeaderPairs(byte_limit=budget//raw_count//2)
                    for name in ('rgb','depth'):
                        def image(message,ident=ident,source=source,pairs=pairs,name=name):
                            try:
                                pair,expired=pairs.add(name,message,node.get_clock().now().nanoseconds,time.monotonic_ns())
                                for item in expired:rejected(ident,'Unpaired image evicted from bounded input buffer')
                                if pair:
                                    report(ident)
                                    sessions=self.input_sessions()
                                    if sessions:
                                        d=pairs.document(ident,source,pair)
                                        rgb=decode_image(pair['streams']['rgb'][0])
                                        depth=decode_image(pair['streams']['depth'][0])
                                        for session in sessions:
                                            session.submit(dict(d),rgb,depth)
                            except (ValueError,KeyError,TypeError) as error:rejected(ident,error)
                        capture_subscriptions.append(CaptureSubscription(node,Image,source[name+'_topic'],image))
                        if source.get(name+'_info_topic'):
                            node.create_subscription(CameraInfo,source[name+'_info_topic'],lambda msg,pairs=pairs,name=name:pairs.info(name,msg),qos_profile_sensor_data)
            next_transport_refresh=time.monotonic()
            while not self.stop_event.is_set():
                executor.spin_once(timeout_sec=.02)
                now=time.monotonic()
                sync_action_sessions()
                for session in self.input_sessions():
                    if session.end is not None and session.data_now()>=session.end:
                        continue
                    for ident,stream in action_streams.get(session.id,{}).items():
                        max_age=max(a['max_age_ms'] for a in sources[ident]['named_axes'])
                        if stream[3] and time.monotonic_ns()-stream[2] > max_age*1_000_000:
                            session.fail('action_evidence_stream_lost:'+ident)
                if now>=next_transport_refresh:
                    for subscription in capture_subscriptions:subscription.refresh()
                    next_transport_refresh=now+1.
        except Exception as error:
            self.error=str(error)
            for session in self.input_sessions():
                session.fail('ros_source_failure:'+self.error)
            for ident,source in self.hub.cfg['sources'].items():
                if source['transport'].startswith('ros_'):self.hub.reports[ident]={'ready':False,'error':self.error}
        finally:
            if executor:executor.shutdown(timeout_sec=1)
            if node:node.destroy_node()
            if context:context.try_shutdown()
