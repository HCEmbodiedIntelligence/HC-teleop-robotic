"""Actual DDS: no separate metadata publishers; unmatched frames stay archived."""
from array import array
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import time

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import ImageMetadata
from std_msgs.msg import Header
from humanoid_camera.transport import capture_qos
from middleware.core.capture.config import validate
from middleware.core.capture.hub import CaptureHub
from middleware.core.capture.originals import camera_topics

script=Path(os.environ['HC_CAMERA_SOURCE'])/'scripts/timestamp_adapter.py'
spec=importlib.util.spec_from_file_location('atomic_case_adapter',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
domain=int(os.environ.get('ROS_DOMAIN_ID','222'))
depth_enabled=sys.argv[1]=='rgbd'
rclpy.init(args=['--ros-args','-r','__ns:=/front','-p','device_type:=d405',
                '-p','source_depth_format:=Z16','-p','bundled_input:=true',
                '-p','depth_enabled:='+str(depth_enabled).lower()],domain_id=domain)
node=Node('atomic_messages_test_only');adapter=module.TimestampAdapter()
executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(adapter)
root=Path(tempfile.mkdtemp(prefix='ros-atomic-retention-'))
hub=CaptureHub(root,{'enabled':True,'domain_id':domain})
camera={'namespace':'front','camera_name':'camera','depth_enabled':depth_enabled}
topics=camera_topics(camera,bundled=True)
source={'kind':'rgbd','transport':'ros_rgbd','topic':'/front/normalized/rgbd',
        'packet_topic':'/front/normalized/capture','depth_enabled':depth_enabled,'original_topics':topics}
hub.cfg=validate({'alignment':{'strict':False},'sources':{'front':source},
                  'storage':{'minimum_free_bytes':0,'numeric_flush_seconds':.01,'retain_original_ros':True}})
hub.prepare_adapters()
pubs={name:node.create_publisher(ImageMetadata,topic,capture_qos(10)) for name,topic in topics.items()}
expected=[];normal=0


def wait_until(predicate,seconds=8):
    end=time.monotonic()+seconds
    while not predicate():
        assert time.monotonic()<end,hub.state()
        executor.spin_once(timeout_sec=.002)


def send(number,*,retain=False,complete=True):
    stamp=time.time_ns()
    h=Header(frame_id='original_optical');h.stamp.sec,h.stamp.nanosec=divmod(stamp,10**9)
    for name,pub in pubs.items():
        if name=='depth_frame' and not complete:continue
        rgb=name=='rgb_frame'
        img=Image(header=h,height=4,width=4,encoding='rgb8' if rgb else '16UC1',
                  step=12 if rgb else 8,data=array('B',[number%255]*(48 if rgb else 32)))
        raw={'clock_domain':'global_time','frame_timestamp':str(stamp//10**6)+'.'+str(stamp%10**6).zfill(6),
             'sensor_timestamp':1000000+number*33333,'hw_timestamp':1000000+number*33333,
             'frame_number':number,'actual_exposure':4000}
        msg=ImageMetadata(image=img,camera_info=CameraInfo(header=h,width=4,height=4),json_data=json.dumps(raw))
        pub.publish(msg)
        if retain:expected.append((topics[name],serialize_message(msg)))


try:
    wait_until(lambda:all(p.get_subscription_count()==2 for p in pubs.values()) and
               adapter.packet_pub.get_subscription_count()==1)
    assert adapter.pair_pub.get_subscription_count()==adapter.meta_pub.get_subscription_count()==0
    assert all(not n.endswith('_meta') for n in adapter.capture_subscriptions)
    send(0);wait_until(lambda:hub.state()['preflight']['ready']);hub.start()
    for number in range(1,51):
        complete=number%3!=0
        send(number,retain=True,complete=complete)
        normal+=int(complete or not depth_enabled)
        until=time.monotonic()+1/30
        while time.monotonic()<until:executor.spin_once(timeout_sec=.002)
    wait_until(lambda:sum(hub.session.originals.counts.values())==len(expected) and
               hub.session.source_counts['front']==normal)
    hub.session.stop()
    assert hub.session.state=='completed',hub.session.status()
    with (hub.session.path/'originals.mcap').open('rb') as stream:
        actual=[(c.topic,m.data) for _,c,m in make_reader(stream,validate_crcs=True).iter_messages()]
    for topic in topics.values():
        assert [deserialize_message(d,ImageMetadata) for t,d in actual if t==topic]==[
            deserialize_message(d,ImageMetadata) for t,d in expected if t==topic]
    # A generic MCAP reader must decode the embedded schema, without ROS types.
    with (hub.session.path/'originals.mcap').open('rb') as stream:
        decoded=list(make_reader(stream,decoder_factories=[DecoderFactory()]).iter_decoded_messages())
    assert len(decoded)==len(expected)
    assert decoded[0].decoded_message.image.width==4
    assert hub.ros_bridge.rejected==0
    assert hub.session.originals.queue.rejected==0
    assert all('paired message/metadata' not in str(e) for e in hub.session.errors)
    print(json.dumps({'atomic_transport':True,'depth_enabled':depth_enabled,
        'original_frames':len(actual),'normalized_frames':normal,'standalone_metadata_publishers':0,'root':str(root)}))
finally:
    hub.shutdown();executor.shutdown();adapter.destroy_node();node.destroy_node();rclpy.try_shutdown()
