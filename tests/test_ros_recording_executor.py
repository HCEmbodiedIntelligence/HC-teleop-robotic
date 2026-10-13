import unittest

from middleware.core.ros_recording_executor import (
    RateGate,
    RosRecordingExecutor,
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

    def test_rate_gate_does_not_halve_jittery_30_hz_input(self):
        gate = RateGate()
        timestamps = [0.0, 0.0325, 0.0650, 0.0975, 0.1300]
        self.assertTrue(all(gate.allow("/camera", 30.0, now) for now in timestamps))

    def test_rate_gate_still_limits_60_hz_input_to_30_hz(self):
        gate = RateGate()
        timestamps = [0.0, 0.0167, 0.0334, 0.0501, 0.0668]
        accepted = [gate.allow("/camera", 30.0, now) for now in timestamps]
        self.assertEqual(accepted, [True, False, True, False, True])

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
