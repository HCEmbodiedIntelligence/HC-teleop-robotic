"""List platform robots or read video observations inside one exclusive session.

Run from the repository root with python -m examples.platform_observer --help.
The API token belongs to the operator or actor. It is read from the environment.
"""

import argparse
import asyncio
import json
import os

from middleware.platform_sessions import OperatorPlatform


async def observe(platform, robot_id, frame_count):
    from middleware.platform_device.operator import operator_peer

    def video_only(peer, _session_id):
        peer.addTransceiver("video", direction="recvonly")

    async with operator_peer(platform, robot_id, video_only) as (peer, session_id):
        track = next(
            r.track for r in peer.getReceivers() if r.track and r.track.kind == "video"
        )
        for index in range(frame_count):
            frame = await asyncio.wait_for(track.recv(), timeout=8)
            # A policy can consume frame.to_ndarray(format="rgb24") here.
            print(
                json.dumps(
                    {
                        "session_id": session_id,
                        "index": index,
                        "width": frame.width,
                        "height": frame.height,
                        "pts": frame.pts,
                        "time_base": str(frame.time_base),
                    }
                ),
                flush=True,
            )


async def run(args):
    token = os.environ.get("HC_PLATFORM_OPERATOR_TOKEN")
    if not token:
        raise SystemExit(
            "set HC_PLATFORM_OPERATOR_TOKEN to an authorized operator token"
        )
    async with OperatorPlatform(args.base_url, token, args.organization) as platform:
        if args.robot is None:
            print(json.dumps(await platform.robots(), ensure_ascii=False, indent=2))
        else:
            await observe(platform, args.robot, args.frames)


def main():
    parser = argparse.ArgumentParser(
        description="Discover robots or sample their camera over WebRTC"
    )
    parser.add_argument("--base-url", required=True)
    parser.add_argument("--organization", required=True)
    parser.add_argument("--robot", help="reserve this robot and read video frames")
    parser.add_argument("--frames", type=int, default=5)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
