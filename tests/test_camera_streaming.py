import fractions
import unittest

try:
    import av
    import numpy as np
except ImportError:  # pragma: no cover - optional camera dependencies
    av = None
    np = None

from middleware.core.camera import CameraService, cv2


@unittest.skipIf(av is None or np is None, "camera dependencies are unavailable")
class FastH264EncoderTests(unittest.TestCase):
    def test_periodic_refresh_is_bounded(self):
        from middleware.core.fast_h264 import FastH264Encoder

        encoder = FastH264Encoder()
        base = np.random.default_rng(3).integers(
            0, 256, (400, 640, 3), dtype=np.uint8
        )
        encoded_sizes = []
        for index in range(70):
            image = np.roll(base, index * 2, axis=1)
            frame = av.VideoFrame.from_ndarray(image, format="bgr24")
            frame.pts = index * 3000
            frame.time_base = fractions.Fraction(1, 90000)
            encoded_sizes.append(
                sum(len(nal) for nal in encoder._encode_frame(frame, False))
            )

        # The previous 60-frame GOP produced a 100-200 KB IDR here. The VBV
        # and periodic intra refresh remove that periodic burst.
        self.assertLess(encoded_sizes[60], 20_000)

    def test_small_remb_change_does_not_reopen_encoder(self):
        from middleware.core.fast_h264 import FastH264Encoder

        encoder = FastH264Encoder()
        image = np.zeros((400, 640, 3), dtype=np.uint8)

        def encode(index):
            frame = av.VideoFrame.from_ndarray(image, format="bgr24")
            frame.pts = index * 3000
            frame.time_base = fractions.Fraction(1, 90000)
            list(encoder._encode_frame(frame, False))

        encode(0)
        codec = encoder.codec
        encoder.target_bitrate = 850_000
        encode(1)
        self.assertIs(encoder.codec, codec)


@unittest.skipIf(cv2 is None or np is None, "OpenCV is unavailable")
class CameraFrameTests(unittest.TestCase):
    def test_ros_frame_is_resized_to_configured_encoder_dimensions(self):
        camera = CameraService(
            {"enabled": True, "width": 640, "height": 400, "fps": 30}
        )
        camera.set_frame(np.zeros((1080, 1920, 3), dtype=np.uint8))
        self.assertEqual(camera.latest().shape, (400, 640, 3))


if __name__ == "__main__":
    unittest.main()
