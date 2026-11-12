"""Live DDS bounded-history test: recorder pauses must not erase small state frames."""
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from middleware.core.ros_qos import subscription_qos, recording_subscription_qos


@pytest.mark.parametrize('kind,cls',[('sensor_msgs/msg/JointState',JointState),
                                    ('geometry_msgs/msg/PoseStamped',PoseStamped)])
def test_recording_keeps_continuous_source_during_short_callback_pause(kind,cls):
    context=Context()
    rclpy.init(context=context,domain_id=197)
    node=rclpy.create_node('recording_pause_boundary',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    received=[]
    publisher=node.create_publisher(cls,'/test/recording_pause',QoSProfile(depth=100))
    # The production capture process must use the same recording-only policy.
    subscriber=node.create_subscription(cls,'/test/recording_pause',
        lambda msg:received.append(msg.header.frame_id),recording_subscription_qos(dict(topic='/test/recording_pause',type=kind)))
    try:
        deadline=time.monotonic()+5
        while publisher.get_subscription_count()!=1 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
        assert publisher.get_subscription_count()==1
        warm=cls();warm.header.frame_id='warm';publisher.publish(warm)
        deadline=time.monotonic()+3
        while not received and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        assert received==['warm'];received.clear()
        # Reception continues in DDS while the Python capture executor is busy.
        for i in range(20):
            msg=cls();msg.header.frame_id=str(i);msg.header.stamp=node.get_clock().now().to_msg()
            publisher.publish(msg);time.sleep(.01)
        deadline=time.monotonic()+.6
        while len(received)<20 and time.monotonic()<deadline:executor.spin_once(timeout_sec=.002)
        assert received==[str(i) for i in range(20)], received
    finally:
        node.destroy_subscription(subscriber)
        executor.shutdown();node.destroy_node();context.try_shutdown()


def test_images_and_live_control_keep_their_existing_small_queues():
    for kind in ['sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage','sensor_msgs/msg/PointCloud2']:
        for reliability in ['best_effort','reliable']:
            item=dict(topic='/camera',type=kind,qos_reliability=reliability)
            assert recording_subscription_qos(item).depth==subscription_qos(item).depth
    assert subscription_qos(dict(topic='/observation/joint_states',type='sensor_msgs/msg/JointState')).depth==5
