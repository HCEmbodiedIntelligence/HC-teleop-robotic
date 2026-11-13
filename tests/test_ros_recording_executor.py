import unittest
import queue
import struct
import sys
import threading
from types import SimpleNamespace
from unittest.mock import patch

from middleware.core import ros_recording_executor as recording_module

from middleware.core.ros_recording_executor import (
    RateGate,
    StatePreroll,
    RosRecordingExecutor,
    _should_release_subscriptions,
    idle_monitoring_subscriptions,
    recording_subscriptions,
)


class FakeRecorder:
    def __init__(self, active=True):
        self.active = active
        self.events = []

    def is_recording(self):
        return self.active

    def record(self, event):
        self.events.append(event)
        return True


class RosRecordingExecutorTests(unittest.TestCase):
    def test_selects_only_enabled_record_topics(self):
        selected = recording_subscriptions(
            {
                "subscriptions": [
                    {"topic": "/keep", "enabled": True, "outputs": ["record"]},
                    {"topic": "/disabled", "enabled": False, "outputs": ["record"]},
                    {"topic": "/web", "enabled": True, "outputs": ["websocket"]},
                ]
            }
        )
        self.assertEqual([item["topic"] for item in selected], ["/keep"])

    def test_idle_monitor_keeps_only_dashboard_and_proprioception(self):
        subscriptions = [
            {
                "topic": "/observation/joint_states",
                "type": "sensor_msgs/msg/JointState",
                "outputs": ["record"],
            },
            {
                "topic": "/control/solver/joint_targets",
                "type": "sensor_msgs/msg/JointState",
                "outputs": ["record"],
            },
            {
                "topic": "/action/joint_states",
                "type": "sensor_msgs/msg/JointState",
                "outputs": ["record"],
            },
            {
                "topic": "/observation/cameras/head/color/compressed",
                "type": "sensor_msgs/msg/CompressedImage",
                "outputs": ["record"],
            },
        ]

        selected = idle_monitoring_subscriptions(subscriptions)

        self.assertEqual(
            [item["topic"] for item in selected],
            ["/observation/joint_states", "/control/solver/joint_targets"],
        )

    def test_preroll_preserves_real_cdr_and_both_timestamps(self):
        buffer = StatePreroll()
        event = dict(topic='/observation/joint_states', _raw=b'exact-cdr',
                     stamp_ns=123456789, msg_type='sensor_msgs/msg/JointState')
        buffer.remember(10.0, event)
        self.assertEqual(buffer.snapshot(10.04), [(10.0, event)])
        self.assertIs(buffer.snapshot(10.04)[0][1], event)
        self.assertEqual(buffer.snapshot(10.04)[0][1]['_raw'], b'exact-cdr')
        self.assertEqual(buffer.snapshot(10.04)[0][1]['stamp_ns'], 123456789)

    def test_preroll_bounds_time_bytes_and_never_buffers_images(self):
        buffer = StatePreroll(byte_budget=8)
        state = lambda n: dict(topic='/observation/joint_states', _raw=n)
        buffer.remember(1.0, state(b'12345'))
        buffer.remember(1.1, state(b'67890'))
        self.assertEqual([x[1]['_raw'] for x in buffer.snapshot(1.1)], [b'67890'])
        buffer.remember(1.15, dict(topic='/camera', _raw=b'x' * 100000))
        self.assertEqual(buffer.buffered_bytes, 5)
        self.assertEqual(buffer.snapshot(1.36), [])
        self.assertEqual(buffer.buffered_bytes, 0)

    def test_preroll_repeated_snapshot_is_not_a_deferred_old_capture(self):
        buffer = StatePreroll()
        buffer.remember(1.0, dict(topic='/observation/joint_states', _raw=b'a'))
        self.assertEqual(len(buffer.snapshot(1.1)), 1)
        # A later recording gets only genuinely recent history, not the earlier
        # capture's sample or a synthetic replacement.
        self.assertEqual(buffer.snapshot(2.0), [])

    def test_preroll_configured_rate_uses_original_receive_times(self):
        buffer = StatePreroll()
        for n in range(6):
            buffer.remember(1 + n * .01,
                dict(topic='/observation/joint_states', _raw=bytes([n])))
        gate = RateGate()
        selected = [event['_raw'] for received, event in buffer.snapshot(1.06)
                    if gate.allow(event['topic'], 50, received)]
        self.assertEqual(selected, [b'\0', b'\2', b'\4'])

    def test_real_process_callback_prerolls_before_delayed_first_camera_once(self):
        """Simulate only ROS transport, exercise the actual raw process flow."""
        accepting = threading.Event()
        stop = threading.Event()
        reset = threading.Event()
        ready = threading.Event()
        subscriptions = {}
        recorded = []
        clock = [100.]
        phase = [0]
        capture = [0]
        state = '/observation/joint_states'
        camera = '/observation/cameras/head/color/compressed'

        class Node:
            def create_subscription(self, kind, topic, callback, qos, **kwargs):
                subscriptions[topic] = callback
                return topic
            def destroy_subscription(self, subscription):
                subscriptions.pop(subscription, None)
            def destroy_node(self):
                pass

        def raw_stamp(value):
            ns = round(value * 1_000_000_000)
            sec, nano = divmod(ns, 1_000_000_000)
            return b'\0\1\0\0' + struct.pack('<iI', sec, nano)

        class Executor:
            def __init__(self, **kwargs):
                pass
            def add_node(self, node):
                pass
            def remove_node(self, node):
                pass
            def shutdown(self, **kwargs):
                pass
            def spin_once(self, **kwargs):
                n = phase[0]
                clock[0] = 100 + n * .01
                if n < 30:
                    subscriptions[state](raw_stamp(clock[0]))
                elif n == 30:
                    reset.set()  # Normal prepare_recording before start.
                elif n == 31:
                    capture[0] = 1
                    accepting.set()
                    # Acquisition predates recording by 40 ms, like the real
                    # image transport. The corresponding state was received idle.
                    subscriptions[camera](raw_stamp(clock[0] - .04))
                elif n < 37:
                    subscriptions[state](raw_stamp(clock[0]))
                elif n == 37:
                    accepting.clear()
                elif n < 70:
                    subscriptions[state](raw_stamp(clock[0]))
                elif n == 70:
                    reset.set()
                elif n == 71:
                    capture[0] = 2
                    accepting.set()
                    subscriptions[camera](raw_stamp(clock[0] - .04))
                elif n < 77:
                    subscriptions[state](raw_stamp(clock[0]))
                else:
                    accepting.clear()
                    stop.set()
                phase[0] += 1

        node = Node()
        modules = {
            'rclpy': SimpleNamespace(init=lambda **kw: None,
                                     create_node=lambda *a, **kw: node),
            'rclpy.context': SimpleNamespace(Context=lambda: SimpleNamespace(
                try_shutdown=lambda: None)),
            'rclpy.executors': SimpleNamespace(SingleThreadedExecutor=Executor),
            'rclpy.qos': SimpleNamespace(qos_profile_sensor_data=object()),
            'rosidl_runtime_py.utilities': SimpleNamespace(get_message=lambda x: bytes),
            'middleware.core.ros_qos': SimpleNamespace(
                recording_subscription_qos=lambda item: object()),
        }
        items = [dict(topic=state, type='sensor_msgs/msg/JointState'),
                 dict(topic=camera, type='sensor_msgs/msg/CompressedImage')]
        with patch.dict(sys.modules, modules), \
             patch.object(recording_module, '_lower_recording_priority'), \
             patch.object(recording_module, '_health_snapshot', return_value={}), \
             patch('middleware.core.topic_recorder.enqueue_record',
                   side_effect=lambda resources, event: recorded.append(
                       {**event, '_test_capture': capture[0]}) or True), \
             patch.object(recording_module.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(recording_module.time, 'time_ns',
                          side_effect=lambda: round(clock[0] * 1_000_000_000)):
            recording_module._raw_recording_process(
                {}, items, None, accepting, None, stop, reset, ready, queue.Queue(), None)
        state_events = [event for event in recorded if event['topic'] == state]
        def stamp(event):
            sec, nano = struct.unpack_from('<iI', event['_raw'], 4)
            return sec + nano * 1e-9
        self.assertTrue(state_events)
        for capture_id in (1, 2):
            camera_event = next(event for event in recorded
                if event['topic'] == camera and event['_test_capture'] == capture_id)
            captured_states = [event for event in state_events
                               if event['_test_capture'] == capture_id]
            self.assertLessEqual(min(abs(stamp(event) - stamp(camera_event))
                                     for event in captured_states), .02)
            # Raw CDR/header and historical ingress receipt times are retained.
            self.assertTrue(any(event['stamp_ns'] < camera_event['stamp_ns']
                                for event in captured_states))
            self.assertTrue(all(event['stamp_ns'] >= camera_event['stamp_ns']
                                - 250_000_000 for event in captured_states))
        stamps = [stamp(event) for event in state_events]
        self.assertEqual(len(stamps), len(set(stamps)))

    def test_all_profiles_use_page_selection_without_historical_topic_filter(self):
        subscriptions = [
            dict(topic='/action/base/command', type='std_msgs/msg/Float64MultiArray', outputs=['record']),
            dict(topic='/observation/hands/triggers', type='geometry_msgs/msg/PoseArray', outputs=['record']),
            dict(topic='/action/joint_states', type='std_msgs/msg/String', outputs=['record']),
            dict(topic='/events/control/intervention', type='std_msgs/msg/String', outputs=['record']),
            dict(topic='/custom/sensor', type='std_msgs/msg/String', outputs=['record']),
            dict(topic='/disabled', type='std_msgs/msg/String', outputs=['record'], enabled=False),
            dict(topic='/unselected', type='std_msgs/msg/String', outputs=['websocket']),
        ]
        for profile in ('/robots/x1', '/robots/other', ''):
            with self.subTest(profile=profile):
                self.assertEqual(recording_subscriptions(dict(legacy_control_profile=profile,
                                                              subscriptions=subscriptions)), subscriptions[:5])

    def test_rate_gate_does_not_halve_jittery_30_hz_input(self):
        gate = RateGate()
        timestamps = [0.0, 0.0325, 0.0650, 0.0975, 0.1300]
        self.assertTrue(all(gate.allow("/camera", 30.0, now) for now in timestamps))

    def test_rate_gate_still_limits_60_hz_input_to_30_hz(self):
        gate = RateGate()
        timestamps = [0.0, 0.0167, 0.0334, 0.0501, 0.0668]
        accepted = [gate.allow("/camera", 30.0, now) for now in timestamps]
        self.assertEqual(accepted, [True, False, True, False, True])

    def test_subscriptions_release_as_soon_as_capture_stops(self):
        self.assertTrue(
            _should_release_subscriptions(
                has_subscriptions=True,
                capture_seen=True,
                accepting=False,
                now=5.0,
                prepared_until=30.0,
            )
        )

    def test_precheck_subscriptions_wait_until_grace_period_expires(self):
        arguments = dict(
            has_subscriptions=True,
            capture_seen=False,
            accepting=False,
            prepared_until=30.0,
        )
        self.assertFalse(_should_release_subscriptions(now=29.9, **arguments))
        self.assertTrue(_should_release_subscriptions(now=30.0, **arguments))

    def test_forwards_raw_cdr_only_while_recording(self):
        recorder = FakeRecorder(active=False)
        executor = RosRecordingExecutor(
            {"enabled": True, "domain_id": 14, "subscriptions": []}, recorder
        )
        executor._handle_message(
            b"ignored-cdr",
            topic="/camera",
            msg_type="sensor_msgs/msg/CompressedImage",
            max_hz=0.0,
        )
        self.assertEqual(recorder.events, [])

        recorder.active = True
        executor._handle_message(
            b"cdr",
            topic="/camera",
            msg_type="sensor_msgs/msg/CompressedImage",
            max_hz=0.0,
        )
        self.assertEqual(recorder.events[0]["_raw"], b"cdr")
        self.assertEqual(executor.status()["recorded"], 1)


if __name__ == "__main__":
    unittest.main()
