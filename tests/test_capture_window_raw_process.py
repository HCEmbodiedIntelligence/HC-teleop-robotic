"""Run the production raw callback loop with deterministic simulated DDS."""
import multiprocessing as mp
import queue
import struct
import sys
import threading
from types import SimpleNamespace
from unittest.mock import patch
import pytest

from middleware.core.recording import ros_recording_executor as module
from middleware.core.recording.capture_window import CaptureWindow, header_stamp_ns


@pytest.mark.parametrize('ordinary', [True, False])
def test_warm_readers_survive_generation_and_late_rgbd_is_recorded_once(ordinary):
    color = '/observation/cameras/head/color/compressed'
    depth = color.replace('/color/', '/depth/')
    state = '/observation/joint_states'
    items = [dict(topic=t, type='sensor_msgs/msg/CompressedImage') for t in (color, depth)]
    items += [dict(topic=state, type='sensor_msgs/msg/JointState'),
              dict(topic='/other', type='std_msgs/msg/String')]
    accepting, stop, reset, ready = [threading.Event() for _ in range(4)]
    window = CaptureWindow(mp.get_context('spawn'))
    window.reset(ordinary)
    callbacks, ids, creations, recorded, qos_seen = {}, {}, {}, [], {}
    clock, phase = [100.], [0]

    def raw(stamp):
        sec, nano = divmod(round(stamp*1e9), 1_000_000_000)
        return b'\0\1\0\0'+struct.pack('<iI', sec, nano)

    class Node:
        def create_subscription(self, kind, topic, callback, qos, **kwargs):
            creations[topic] = creations.get(topic, 0)+1
            qos_seen.setdefault(topic, []).append(qos)
            identifier = (topic, creations[topic])
            ids[identifier], callbacks[topic] = topic, callback
            return identifier
        def destroy_subscription(self, identifier):
            callbacks.pop(ids.pop(identifier), None)
        def destroy_node(self):
            pass

    class Executor:
        def __init__(self, **kwargs): pass
        def add_node(self, node): pass
        def remove_node(self, node): pass
        def shutdown(self, **kwargs): pass
        def spin_once(self, **kwargs):
            n = phase[0]
            clock[0] = 100+n*.01
            if n in (18, 25, 30):
                reset.set()
            if n == 31:
                window.open(round(clock[0]*1e9))
                accepting.set()
            if n <= 35:
                callbacks[state](raw(clock[0]))
                if n % 3 == 2 or n == 31:
                    callbacks[color](raw(clock[0]-.04))
                    if n != 35:
                        callbacks[depth](raw(clock[0]-.04))
            if n == 36:
                window.request_stop(round(clock[0]*1e9))
            if n == 38:
                # Same original exposure as n=35 color, later receipt.
                callbacks[depth](raw(100.31))
                callbacks[color](raw(clock[0]))  # Post-stop exposure is outside.
            if n == 40:
                window.close_raw(round(clock[0]*1e9))
                accepting.clear()
            if n == 43:
                stop.set()
            phase[0] += 1

    node = Node()
    modules = {
        'rclpy': SimpleNamespace(init=lambda **kw:None, create_node=lambda *a,**kw:node),
        'rclpy.context': SimpleNamespace(Context=lambda:SimpleNamespace(try_shutdown=lambda:None)),
        'rclpy.executors': SimpleNamespace(SingleThreadedExecutor=Executor),
        'rclpy.qos': SimpleNamespace(qos_profile_sensor_data=object()),
        'rosidl_runtime_py.utilities': SimpleNamespace(get_message=lambda kind:bytes),
        'middleware.core.control.ros_qos': SimpleNamespace(recording_subscription_qos=lambda item:'configured',
            warm_recording_subscription_qos=lambda item:'best_effort'),
    }
    with patch.dict(sys.modules, modules), \
         patch.object(module, '_lower_recording_priority'), \
         patch.object(module, '_health_snapshot', return_value={}), \
         patch('middleware.core.recording.topic_recorder.enqueue_record', side_effect=lambda resources,event,**kw:recorded.append(event) or True), \
         patch.object(module.time, 'monotonic', side_effect=lambda:clock[0]), \
         patch.object(module.time, 'time_ns', side_effect=lambda:round(clock[0]*1e9)):
        module._raw_recording_process({'legacy_control_profile':'x1'}, items, None,
            accepting, None, stop, reset, ready, queue.Queue(), None, window)
    if not ordinary:
        # Snapshot/correction mode still replaces idle readers with configured
        # recording QoS, then returns to idle after close. No override is
        # silently applied to its active recording policy.
        assert all(qos_seen[t] == ['best_effort', 'configured', 'best_effort']
                   for t in (state, color, depth))
        assert window.snapshot() == {}
        return
    assert {topic:creations[topic] for topic in (state,color,depth)} == {state:1,color:1,depth:1}
    keys = [(event['topic'], event['_raw']) for event in recorded]
    assert len(keys) == len(set(keys))  # First live packet never duplicated by flush.
    first_source = min(header_stamp_ns(e['_raw'], e['msg_type']) for e in recorded)
    assert first_source >= 100_060_000_000  # start(100.31)-250ms.
    colors = [header_stamp_ns(e['_raw'],e['msg_type']) for e in recorded if e['topic']==color]
    depths = [header_stamp_ns(e['_raw'],e['msg_type']) for e in recorded if e['topic']==depth]
    assert 100_310_000_000 in colors and 100_310_000_000 in depths
    assert 100_380_000_000 not in colors
    assert sorted(colors) == sorted(depths)
