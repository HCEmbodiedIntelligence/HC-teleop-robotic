import asyncio
import json
import time
from pathlib import Path
import socket
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from uuid import uuid4

from middleware.core.protocol import decode_pose_packet
from middleware.platform_device.bench import frame
from middleware.platform_device.input import InputWindow
from middleware.platform_device.local import Recording
from middleware.platform_device.sink import Live


class InputTests(unittest.TestCase):
    def test_true_delay_bounds_and_rearm(self):
        clock = [100.0]
        w = InputWindow(80, 160, lambda: clock[0])
        w.arm()
        clock[0] += 0.015
        w.sample(frame("s", 1, 90000), "hc-pico-v2")
        clock[0] += 0.040
        w.sample(frame("s", 3, 90020), "hc-pico-v2")
        self.assertEqual((w.accepted, w.skipped), (2, 1))
        self.assertGreaterEqual(w.age.report()["max"], 35)
        self.assertIsNone(w.sample(frame("s", 2, 90010), "hc-pico-v2"))
        clock[0] += 0.075
        with self.assertRaises(TimeoutError):
            w.sample(frame("s", 4, 90021), "hc-pico-v2")
        clock[0] += 0.10
        self.assertTrue(w.expire())
        w.arm()
        self.assertIsNone(w.sample(frame("s", 8, 90080), "hc-pico-v2"))
        clock[0] += 0.035
        self.assertIsNotNone(w.sample(frame("s", 9, 90252), "hc-pico-v2"))

    def test_native_wire_mapping_and_local_shortcuts(self):
        m = frame("s", 1, 1000)
        hand = m["controllers"][1]
        hand.update(buttons=[0.75, 0.25, 0, 1, 1, 1], axes=[0.2, 0.3, -0.5, 1])
        w = InputWindow(100, 180)
        w.arm()
        native = w.sample(m, "hc-pico-v2")
        parsed = decode_pose_packet(native)
        self.assertEqual(parsed.right_input.primary_axis, (-0.5, 1))
        self.assertAlmostEqual(parsed.right_input.secondary_axis[0], 0.2)
        with (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as emitter,
        ):
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            live = Live()
            live.state = {"pose_port": receiver.getsockname()[1]}
            live.udp = emitter
            live.send(native)
            packet = decode_pose_packet(receiver.recv(1024))
            self.assertEqual(packet.right_input.held_mask, 8)
            self.assertEqual(packet.right_input.trigger, 0.75)
            self.assertEqual(packet.right_input.grip, 0.25)

    def test_invalid_pose_and_calibrated_webxr(self):
        m = frame("s", 1, 1000)
        w = InputWindow(100, 180)
        w.arm()
        m["head"] = {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]}
        self.assertEqual(
            decode_pose_packet(w.sample(m, "webxr", True)).head.position, (1, 2, -3)
        )
        self.assertNotIn("webxr", Live().profiles)
        m = frame("s", 2, 1001)
        m["controllers"][0]["grip"]["orientation"][0] = float("nan")
        with self.assertRaises(ValueError):
            w.sample(m, "hc-pico-v2")


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.sid = str(uuid4())
        self.rec = rec = Mock(active=None, lock=threading.RLock(), jobs={})
        self.rt = rt = SimpleNamespace(
            recorder=rec,
            session_profile_metadata={},
            platform=Mock(directory=Path(self.tmp.name)),
        )
        rt.platform.recording_selection.return_value = {
            "mode": "platform",
            "task_id": "t",
            "connection_id": "c",
        }
        rt.platform.preflight.return_value = {
            "task": {"ownership": {"project_id": "p"}}
        }

        def start(name, meta, sel):
            rec.active = "record"
            rec.jobs["record"] = {"state": "recording", "request_id": sel["request_id"]}

        def stop():
            rec.jobs["record"]["state"] = "finalizing"
            rec.active = None

        rec.start.side_effect = start
        rec.stop.side_effect = stop
        rec.mark_current_or_latest.return_value = {"marked": True}
        self.cap = Recording(rt, "p")
        self.addCleanup(lambda: self.cap.db and self.cap.db.close())

    def command(self, action, sid=None):
        return dict(
            session_id=sid or self.sid,
            request_id=str(uuid4()),
            revision=0,
            project_id="p",
            task_id="t",
            action=action,
        )

    def test_dedup_restart_and_upload(self):
        start = self.command("start")
        receipt = self.cap.handle(start)
        self.assertEqual(receipt["state"], "recording")
        self.assertEqual(self.cap.handle(start), receipt)
        self.rec.start.assert_called_once()
        self.cap.db.close()
        self.cap = Recording(self.rt, "p")
        self.assertEqual(self.cap.next_receipt(), [self.sid, receipt])
        self.cap.confirm(receipt)
        self.assertIsNone(self.cap.next_receipt())
        mark = self.command("mark")
        self.assertEqual(self.cap.handle(mark), self.cap.handle(mark))
        self.rec.mark_current_or_latest.assert_called_once()
        self.cap.confirm(self.cap.handle(mark))
        stop = self.cap.handle(self.command("stop"))
        self.assertEqual(stop["record_id"], "record")
        self.assertEqual(stop["state"], "saved")
        self.cap.confirm(stop)
        self.rec.jobs["record"].update(
            state="queued", artifacts={"mcap": {"artifact_id": "a"}}
        )
        self.rt.platform.upload.queue.jobs.return_value = [
            dict(artifact_id="a", raw_source_id="registered")
        ]
        self.assertEqual(self.cap.next_receipt()[1]["upload"], "uploaded")

    def test_no_cross_session_or_late_start(self):
        self.cap.handle(self.command("start"))
        other = self.cap.handle(self.command("stop", str(uuid4())))
        self.assertEqual(other["state"], "error")
        self.rec.stop.assert_not_called()
        self.cap.confirm(other)
        self.cap.end_session(self.sid)
        self.rec.stop.assert_called_once()
        self.assertEqual(self.cap.next_receipt()[1]["state"], "interrupted")
        late = self.cap.handle(self.command("start"))
        self.assertEqual(late["state"], "error")
        self.rec.start.assert_called_once()


class BurstTests(unittest.IsolatedAsyncioTestCase):
    async def test_burst_and_queued_disable(self):
        from middleware.platform_device.peer import Peer
        from middleware.platform_device.bench import BenchSink
        from middleware.platform_device.__main__ import TestPattern

        sid = str(uuid4())
        sink = BenchSink()
        p = Peer(sid, [], TestPattern, sink, {"max_lag_ms": 100, "watchdog_ms": 180})
        try:
            p.profile = "hc-pico-v2"
            p.track.last = time.monotonic()
            p.window.arm()
            stamp = time.monotonic() * 1000
            for seq in range(80):
                p.receive(json.dumps(frame(sid, seq, stamp + seq * 0.01)))
            await asyncio.sleep(0)
            self.assertEqual(sink.packets, 1)
            self.assertEqual(p.coalesced, 79)
            self.assertEqual(p.window.last_values["seq"], 79)
            p.receive(json.dumps(frame(sid, 80, stamp + 1)))
            p.stop("operator disabled")
            await asyncio.sleep(0)
            self.assertEqual(sink.packets, 1)
            p.window.arm()
            p.lease_valid = lambda: False
            p.receive(json.dumps(frame(sid, 90, time.monotonic() * 1000)))
            await asyncio.sleep(0)
            self.assertFalse(p.window.enabled)
            self.assertEqual(sink.packets, 1)
        finally:
            await p.close()


def test_native_capture_and_restart_receipt(tmp_path, monkeypatch):
    from test_platform_task_recording import connected, choice, finish, LOCAL

    _, platform, recorder = connected(tmp_path, monkeypatch)
    platform.set_recording_selection(choice(platform, "A"))
    cap = Recording(
        SimpleNamespace(
            platform=platform,
            recorder=recorder,
            session_profile_metadata={"profile_id": LOCAL["profile_id"]},
        ),
        "project-A",
    )
    sid = str(uuid4())

    def command(action):
        return dict(
            session_id=sid,
            request_id=str(uuid4()),
            revision=0,
            project_id="project-A",
            task_id="A",
            action=action,
        )

    try:
        start = command("start")
        begun = cap.handle(start)
        assert begun["state"] == "recording", begun
        ident = begun["record_id"]
        path = Path(recorder.jobs[ident]["path"])
        for i in range(4):
            recorder.record(
                {"topic": "/test", "payload": {"frame": i}, "stamp_ns": time.time_ns()}
            )
        marker = cap.handle(command("mark"))
        assert marker["detail"] == "marker saved", marker
        stop = command("stop")
        saved = cap.handle(stop)
        assert saved["state"] == "saved" and saved["record_id"] == ident, saved
        finish(recorder)
        assert path.is_file() and path.stat().st_size > 128
        assert path.read_bytes()[:6] == b"\x89MCAP0"
        # Simulate process death immediately after the native stop and before
        # commit of the receipt, then recover without a second recorder call.
        with cap.db:
            cap.db.execute(
                "UPDATE commands SET receipt=NULL WHERE id=?", (stop["request_id"],)
            )
        cap.db.close()
        cap.db = None
        assert cap.next_receipt() == [sid, saved]
        assert cap.handle(stop) == saved
    finally:
        recorder.close()
        if cap.db:
            cap.db.close()
