"""Record HC source timestamps and validity using the installed LeRobot dataset writer."""

import argparse
from pathlib import Path

from lerobot.configs import RGBEncoderConfig
from lerobot.datasets import LeRobotDataset

from . import HCTeleop, HCTeleopConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--task", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frames", type=int, required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--record-hand", action="append", default=[])
    args = parser.parse_args()
    if not 1 <= args.fps <= 120 or min(args.frames, args.episodes) <= 0:
        parser.error(
            "fps must be in [1,120] and frame and episode counts must be positive"
        )
    if args.destination.exists():
        parser.error("the dataset destination already exists")
    robot = HCTeleop(
        HCTeleopConfig(record_fps=args.fps, record_hands=tuple(args.record_hand))
    )
    dataset = None
    try:
        features = robot.connect_demonstration(args.fps, True)
        dataset = LeRobotDataset.create(
            "local/hc_demonstration",
            root=args.destination,
            fps=args.fps,
            robot_type=robot.robot_type,
            features=features,
            use_videos=True,
            video_backend="pyav",
            rgb_encoder=RGBEncoderConfig(
                vcodec="h264",
                g=args.fps,
                crf=23,
                preset="fast",
                extra_options={"bf": "0", "tune": "zerolatency"},
            ),
            encoder_threads=1,
        )
        for _ in range(args.episodes):
            robot.begin_demonstration_episode()
            for _ in range(args.frames):
                dataset.add_frame({**robot.read_demonstration(), "task": args.task})
            dataset.save_episode()
    finally:
        try:
            if dataset is not None:
                dataset.finalize()
        finally:
            robot.disconnect()


if __name__ == "__main__":
    main()
