"""Standalone network probe or an opt-in session on the middleware camera worker."""

import argparse
import asyncio
import logging
import os

import av
from aiortc import VideoStreamTrack

from .agent import Agent, settings
from .sink import Live, Observe


class TestPattern(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.n = 0

    async def recv(self):
        pts, base = await self.next_timestamp()
        self.n += 1
        frame = av.VideoFrame(640, 360, "yuv420p")
        # Moving vertical bar makes a frozen preview distinguishable on site.
        pos = (self.n * 8) % 576
        line = bytes([38]) * pos + bytes([208]) * 64 + bytes([38]) * (640 - pos - 64)
        y = frame.planes[0]
        y.update((line + bytes(y.line_size - 640)) * 360)
        for p in frame.planes[1:]:
            p.update(bytes([128]) * p.buffer_size)
        frame.pts, frame.time_base = pts, base
        return frame


async def run_camera(camera, cfg):
    from middleware.core.fast_h264 import encoder_backend, install_fast_h264_encoder

    install_fast_h264_encoder()
    token = encoder_backend.set(camera.config.get("encoder_backend", "x264"))
    try:
        sink = Live(cfg) if cfg.get("mode") == "live" else Observe()
        agent = Agent(cfg, camera.platform_video_track, sink)
        camera.platform_status = lambda: agent.status
        await agent.run()
    finally:
        encoder_backend.reset(token)


def camera_task(camera):
    path = os.environ.get("HC_TELEOP_DEVICE_CONFIG")
    if not path:
        return None
    cfg = settings(path)
    if cfg.get("camera_id", "main") != camera.config.get("id", "main"):
        return None
    logger = logging.getLogger("platform-device")
    logger.setLevel(logging.INFO)
    if not logger.hasHandlers():
        logger.addHandler(logging.StreamHandler())
    task = asyncio.create_task(run_camera(camera, cfg))

    def stopped(future):
        if not future.cancelled() and future.exception() is not None:
            error = future.exception()
            camera.platform_status = lambda: {
                "state": "failed",
                "error": type(error).__name__,
            }
            logger.error(
                "device agent failed",
                exc_info=(type(error), error, error.__traceback__),
            )

    task.add_done_callback(stopped)
    return task


def main():
    p = argparse.ArgumentParser(description="Platform teleoperation device test")
    p.add_argument("--config", required=True, help="device JSON configuration")
    args = p.parse_args()
    cfg = settings(args.config)
    if cfg.get("mode") == "live":
        p.error("hardware mode must run in the selected middleware camera worker")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    asyncio.run(Agent(cfg, TestPattern, Observe()).run())


if __name__ == "__main__":
    main()
