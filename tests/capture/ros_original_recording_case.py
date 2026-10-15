"""Real ROS transport, deliberately missing metadata; no hardware or motion."""
from array import array
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import time

from mcap.reader import make_reader
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import Metadata
from std_msgs.msg import Header

from humanoid_camera.transport import capture_qos
from middleware.core.capture.config import validate
from middleware.core.capture.hub import CaptureHub
from middleware.core.capture.originals import camera_topics


script=Path(os.environ['HC_CAMERA_SOURCE'])/'scripts/timestamp_adapter.py'
spec=importlib.util.spec_from_file_location('original_case_adapter',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
domain=int(os.environ.get('ROS_DOMAIN_ID','221'))
rclpy.init(args=['--ros-args','-r','__ns:=/front','-p','device_type:=d405',
                '-p','source_depth_format:=Z16'],domain_id=domain)
node=Node('original_messages_test_only')
adapter=module.TimestampAdapter()
executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(adapter)
root=Path(tempfile.mkdtemp(prefix='ros-original-retention-'))
hub=CaptureHub(root,{'enabled':True,'domain_id':domain})
camera={'namespace':'front','camera_name':'camera','rgb_topic':'/front/camera/color/image_raw',
        'depth_topic':'/front/camera/depth/image_rect_raw'}
topics=camera_topics(camera)
source={'kind':'rgbd','transport':'ros_rgbd','topic':'/front/normalized/rgbd',
        'metadata_topic':'/front/normalized/metadata','original_topics':topics}
hub.cfg=validate({'alignment':{'strict':False},'sources':{'front':source},
                  'storage':{'minimum_free_bytes':0,'numeric_flush_seconds':.01,'retain_original_ros':True}})
hub.prepare_adapters()
publishers={name:node.create_publisher(Metadata if name.endswith('_meta') else Image,topic,capture_qos(100))
            for name,topic in topics.items() if not name.endswith('_info')}
expected=[]


def wait_until(predicate,seconds=5):
    end=time.monotonic()+seconds
    while not predicate():
        assert time.monotonic()<end,hub.state()
        executor.spin_once(timeout_sec=.002)


def send(number,*,complete=False,retain=False):
    stamp=time.time_ns()
    h=Header(frame_id='original_optical');h.stamp.sec,h.stamp.nanosec=divmod(stamp,10**9)
    for name in ('rgb','depth'):
        if name=='depth' and number%3==0 and not complete:continue
        msg=Image(header=h,height=4,width=4,encoding='rgb8' if name=='rgb' else '16UC1',
                  step=12 if name=='rgb' else 8,data=array('B',[number%255]*(48 if name=='rgb' else 32)))
        publishers[name].publish(msg)
        if retain:expected.append((topics[name],serialize_message(msg)))
        # Only the complete group gets metadata; all other images must still survive.
        if complete:
            raw={'clock_domain':'global_time','frame_timestamp':str(stamp//10**6)+'.'+str(stamp%10**6).zfill(6),
                 'sensor_timestamp':1000000+number*33333,'hw_timestamp':1000000+number*33333,
                 'frame_number':number,'actual_exposure':4000}
            meta=Metadata(header=h,json_data=json.dumps(raw))
            publishers[name+'_meta'].publish(meta)
            if retain:expected.append((topics[name+'_meta'],serialize_message(meta)))


try:
    wait_until(lambda:all(p.get_subscription_count()>=2 for p in publishers.values()) and
               adapter.pair_pub.get_subscription_count()>=1 and adapter.meta_pub.get_subscription_count()>=1)
    send(0,complete=True)
    wait_until(lambda:hub.state()['preflight']['ready'])
    hub.start()
    send(1,complete=True,retain=True)
    wait_until(lambda:hub.session.source_counts['front']==1)
    for number in range(2,82):
        send(number,retain=True)
        # Test pairing loss at 30 Hz, not an unpaced burst that overruns DDS
        # KEEP_LAST before callbacks can receive it.
        until=time.monotonic()+1/30
        while time.monotonic()<until:executor.spin_once(timeout_sec=.002)
    wait_until(lambda:sum(hub.session.originals.counts.values())==len(expected))
    wait_until(lambda:adapter.diagnostics.discarded['buffer_limit']>0)
    hub.session.stop()
    assert hub.session.state=='completed',hub.session.status()
    with (hub.session.path/'originals.mcap').open('rb') as stream:
        actual=[(c.topic,m.data) for _,c,m in make_reader(stream,validate_crcs=True).iter_messages()]
    # Cross-topic arrival order can differ. Per-topic bytes/counts cannot.
    for topic in topics.values():
        typ=Metadata if topic.endswith('/metadata') else Image
        # DDS/rclpy may serialize different CDR alignment padding. Compare the
        # actual message including Image.data row padding; archive byte identity
        # across compression is independently checked by test_original_recording.
        assert [deserialize_message(data,typ) for t,data in actual if t==topic]==[
            deserialize_message(data,typ) for t,data in expected if t==topic],topic
    assert hub.session.originals.queue.rejected==0
    assert hub.session.source_counts['front']==1
    print(json.dumps({'original_messages':len(actual),'normalized_pairs':1,
        'adapter_evictions':adapter.diagnostics.discarded['buffer_limit'],
        'unpaired_images_preserved':True,'root':str(root)}))
finally:
    hub.shutdown();executor.shutdown();adapter.destroy_node();node.destroy_node();rclpy.try_shutdown()
