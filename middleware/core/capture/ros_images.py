"""Bounded ROS message association by exact Header stamp, without device controls."""
from collections import OrderedDict
import hashlib
import json

from .models import CONTRACT
from .depth_source import ros_image_metadata


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


class HeaderPairs:
    def __init__(self,item_limit=32,byte_limit=64*1024*1024):
        self.items=OrderedDict()
        self.item_limit,self.byte_limit=item_limit,byte_limit
        self.bytes=self.peak_bytes=self.dropped=0
        self.seq=0
        self.stream_seq={'rgb':0,'depth':0}
        self.completed=OrderedDict()
        self.calibration={}
        self.metadata={}

    def info(self,name,message):
        self.calibration[name]={'width':message.width,'height':message.height,
            'k':list(message.k),'d':list(message.d),'r':list(message.r),'p':list(message.p),
            'distortion_model':message.distortion_model,'coordinate_frame':message.header.frame_id}

    def add(self,name,message,ros_ns,steady_ns):
        key=stamp_ns(message.header)
        if key<=0:
            raise ValueError('ROS image Header stamp is uninitialized')
        if key in self.completed or (key in self.items and name in self.items[key]):
            return None,[]
        size=len(message.data)
        self.stream_seq[name]+=1
        expired=[]
        while self.items and (self.bytes+size>self.byte_limit or (key not in self.items and len(self.items)>=self.item_limit)):
            _,old=self.items.popitem(last=False)
            for item in old.values():
                self.bytes-=len(item[0].data)
                expired.append(item)
                self.dropped+=1
        if size>self.byte_limit:
            self.dropped+=1
            return None,expired+[(message,self.stream_seq[name],ros_ns,steady_ns,name)]
        self.items.setdefault(key,{})[name]=(message,self.stream_seq[name],ros_ns,steady_ns,name)
        self.bytes+=size
        self.peak_bytes=max(self.peak_bytes,self.bytes)
        group=self.items[key]
        if set(group)!={'rgb','depth'}:
            return None,expired
        del self.items[key]
        self.bytes-=sum(len(x[0].data) for x in group.values())
        self.seq+=1
        pair={'key':key,'seq':self.seq,'streams':group}
        self.completed[key]=pair
        # Store lineage only; the caller owns images after return.
        self.completed[key]={'key':key,'seq':self.seq,'rgb_seq':group['rgb'][1],
            'depth_seq':group['depth'][1]}
        while len(self.completed)>256:
            self.completed.popitem(last=False)
        return pair,expired

    def document(self,ident,source,pair):
        capture=pair['key']
        calibration=json.loads(json.dumps(self.calibration))
        cfg_id=hashlib.sha256(json.dumps(source,sort_keys=True).encode()).hexdigest()[:24]
        cal_id=hashlib.sha256(json.dumps(calibration,sort_keys=True).encode()).hexdigest()[:24] if calibration else 'unavailable'
        d={'input_contract':CONTRACT,'capture_clock_id':'ros','header_timestamp_ns':capture,
            'source_id':ident,'source_seq':pair['seq'],'pair_seq':pair['seq'],
            'capture_time_ns':capture,'source_timestamp_ns':capture,'clock_id':'ros_driver_header',
            'clock_epoch':0,'clock_model_id':'ros_driver_header_direct','timestamp_quality':'ros_driver_header',
            'clock_uncertainty_ns':None,'clock_evidence':'','receive_clock_id':'ros',
            'receive_time_ns':max(x[2] for x in pair['streams'].values()),
            'receive_steady_ns':max(x[3] for x in pair['streams'].values()),
            'pairing_method':'driver_header_exact','pairing_basis':'configured RGB/depth topics, identical driver Header stamp',
            'camera_config_id':cfg_id,'calibration_id':cal_id,'calibration':calibration,
            'exposure_validation':'disabled','physical_midpoint_verified':False,
            'valid':True,'invalid_reasons':[],'sync_evidence_version':'unverified_driver_association'}
        for name,(image,seq,_,_,_) in pair['streams'].items():
            d[name]={'input_contract':CONTRACT,'source_seq':seq,'capture_time_ns':capture,
                'header_timestamp_ns':capture,'source_timestamp_ns':capture,'exposure_midpoint_ns':capture,
                'timestamp_semantics':'ros_driver_header','timestamp_quality':'ros_driver_header',
                'clock_id':d['clock_id'],'clock_epoch':0,'clock_model_id':d['clock_model_id'],
                'clock_uncertainty_ns':None,'grid_uncertainty_ns':None,
                'coordinate_frame':image.header.frame_id,'physical_midpoint_verified':False}
        d['depth'].update(ros_image_metadata(pair['streams']['depth'][0],source.get('depth_source')))
        # Retain explicitly configured legacy facts, with UNKNOWN evidence status.
        # No default scale is attached to a source whose units were never declared.
        if 'depth_source' not in source:
            d['depth'].update(depth_scale=source.get('depth_scale'),invalid_value=source.get('depth_invalid_value'))
        return d
