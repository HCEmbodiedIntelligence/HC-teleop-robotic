import time
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from middleware.core.camera import CameraService


class DepthDisplayTests(unittest.TestCase):
    def test_depth_display_preserves_original_payload(self):
        camera = CameraService(
            {"enabled": True, "topic": "/depth", "width": 6, "height": 2})
        camera._pcs.add(object())
        camera.start()
        depth = np.array([[0, 500, 1000, 3000, 5000, 6000]] * 2, dtype=np.uint16)
        success, encoded = cv2.imencode(".png", depth)
        self.assertTrue(success)
        payload = encoded.tobytes()
        try:
            camera.handle_ros_message(
                "/depth", SimpleNamespace(format="16UC1; png", data=payload))
            deadline = time.monotonic() + 2
            while camera._latest is None and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertIsNotNone(camera._latest)
            self.assertEqual(camera._latest.dtype, np.uint8)
            self.assertEqual(camera._latest.shape, (2, 6, 3))
            self.assertTrue(np.all(camera._latest[:, 0] == 0))
            self.assertFalse(np.array_equal(camera._latest[0, 1], camera._latest[0, 3]))
            np.testing.assert_array_equal(
                cv2.imdecode(np.frombuffer(payload, np.uint8), -1), depth)
        finally:
            camera._stop.set()
            camera._decode_event.set()
            camera._decode_thread.join(2)
