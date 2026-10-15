"""The reference LeRobot single-channel depth video encoding (HEVC gray12le)."""
from fractions import Fraction
import math

import av
import numpy as np

DEPTH_INFO = {'is_depth_map': True, 'depth_unit': 'mm', 'video.codec': 'hevc',
              'video.pix_fmt': 'gray12le', 'video.channels': 1,
              'video.depth_min': .01, 'video.depth_max': 10., 'video.shift': 3.5,
              'video.use_log': True, 'video.video_backend': 'pyav', 'has_audio': False}


def codes(array, scale):
    # Same quantization domain/parameters as the agreed hc_lerobot depth decoder.
    depth_mm = np.asarray(array, dtype=np.float32) * np.float32(scale * 1000)
    depth_mm = np.where(np.isfinite(depth_mm) & (depth_mm > 0), depth_mm, 0)
    norm = (np.log(depth_mm + np.float32(3500)) - math.log(3510)) / (math.log(13500) - math.log(3510))
    return np.rint(norm * 4095).clip(0, 4095).astype(np.uint16)


class DepthVideo:
    def __init__(self, path, fps, shape):
        if min(shape[:2]) < 16:
            raise ValueError('HEVC depth video requires width and height of at least 16 pixels')
        path.parent.mkdir(parents=True, exist_ok=True)
        self.container = av.open(str(path), 'w')
        self.stream = self.container.add_stream('libx265', rate=fps)
        self.stream.height, self.stream.width = shape[:2]
        self.stream.pix_fmt = 'gray12le'
        self.stream.codec_context.thread_count = 1
        self.stream.options = {'preset': 'ultrafast', 'x265-params':
            'lossless=1:bframes=0:open-gop=0:repeat-headers=1:pools=none:frame-threads=1:log-level=error'}
        self.fps, self.frames = fps, 0

    def write(self, array, scale):
        quantized = codes(array, scale)
        frame = av.VideoFrame(quantized.shape[1], quantized.shape[0], 'gray12le')
        plane = frame.planes[0]
        padded = np.zeros((quantized.shape[0], plane.line_size // 2), dtype='<u2')
        padded[:, :quantized.shape[1]] = quantized
        plane.update(padded.tobytes())
        frame.pts, frame.time_base = self.frames, Fraction(1, self.fps)
        for packet in self.stream.encode(frame):
            self.container.mux(packet)
        self.frames += 1
        return quantized

    def close(self):
        try:
            if self.frames:
                for packet in self.stream.encode(None):
                    self.container.mux(packet)
        finally:
            self.container.close()
