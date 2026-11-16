"""Offline, bounded-memory MCAP → independent MP4 worker.

The fixed FPS output uses causal frame hold. Every output frame has a sidecar
entry describing its original timestamp, repetition and freshness. Raw MCAP is
preserved, so interpolation/repetition is never represented as raw acquisition.
"""

import argparse
import hashlib
import json
import os
import shutil
import signal
import subprocess
from pathlib import Path

from .artifacts import atomic_json
from .errors import DataError
from .sample_codec import decode_image, is_image, iter_samples


class CameraEncoder:
    def __init__(self, root, topic, image, start_ns, fps, max_age_ns, max_frames):
        self.root, self.topic, self.fps = root, topic, fps
        self.camera_id = "camera_" + hashlib.sha256(topic.encode()).hexdigest()[:12]
        self.size = image.size
        self.start_ns, self.next_ns = start_ns, start_ns
        self.period = round(1e9 / fps)
        self.max_age_ns, self.max_frames = max_age_ns, max_frames
        self.frames = self.repeats = self.stale = 0
        self.previous = None
        self.previous_timestamp = self.previous_source = None
        self.last_encoded_source = None
        self.path = root / "videos" / (self.camera_id + ".mp4")
        self.sidecar_path = root / "videos" / (self.camera_id + ".timestamps.jsonl")
        self.sidecar = self.sidecar_path.open("w")
        self.log = (root.parent / (self.camera_id + ".ffmpeg.log")).open("wb")
        self.process = subprocess.Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-nostdin",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                f"{image.width}x{image.height}",
                "-framerate",
                str(fps),
                "-i",
                "pipe:0",
                "-an",
                "-vf",
                "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                "-c:v",
                "libx264",
                "-threads",
                "1",
                "-filter_threads",
                "1",
                "-preset",
                "veryfast",
                "-crf",
                "23",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(self.path),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self.log,
        )

    def add(self, sample, image):
        if image.size != self.size:
            raise DataError("CAMERA_RESOLUTION_CHANGED", self.topic)
        stamp = sample["timestamp_ns"]
        if self.previous_timestamp is not None and stamp < self.previous_timestamp:
            raise DataError("CAMERA_TIME_REVERSED", self.topic)
        while self.previous is not None and self.next_ns < stamp:
            self._write()
        self.previous = image.tobytes()
        self.previous_timestamp = stamp
        self.previous_source = sample["source_ns"]

    def _write(self):
        if self.frames >= self.max_frames:
            raise DataError(
                "VIDEO_FRAME_LIMIT",
                "Increase the explicitly configured offline frame budget",
            )
        age = self.next_ns - self.previous_timestamp
        repeated = self.last_encoded_source == self.previous_timestamp
        valid = 0 <= age <= self.max_age_ns
        self.process.stdin.write(self.previous)
        self.sidecar.write(
            json.dumps(
                {
                    "frame_index": self.frames,
                    "video_time_ns": self.frames * self.period,
                    "export_time_ns": self.next_ns,
                    "recorded_time_ns": self.previous_timestamp,
                    "source_time_ns": self.previous_source,
                    "age_ns": age,
                    "repeated": repeated,
                    "valid": valid,
                }
            )
            + "\n"
        )
        self.frames += 1
        self.repeats += int(repeated)
        self.stale += int(not valid)
        self.last_encoded_source = self.previous_timestamp
        self.next_ns += self.period

    def finish(self):
        if self.previous is not None and self.next_ns <= self.previous_timestamp:
            self._write()
        self.process.stdin.close()
        try:
            result = self.process.wait(timeout=60)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
            raise DataError("VIDEO_ENCODER_TIMEOUT") from None
        self.sidecar.flush()
        os.fsync(self.sidecar.fileno())
        self.sidecar.close()
        self.log.close()
        if result != 0 or self.frames == 0:
            raise DataError("VIDEO_ENCODER_FAILED", self.topic)
        return {
            "camera_id": self.camera_id,
            "topic": self.topic,
            "path": self.path.relative_to(self.root).as_posix(),
            "timestamps_path": self.sidecar_path.relative_to(self.root).as_posix(),
            "codec": "h264",
            "width": (self.size[0] + 1) // 2 * 2,
            "height": (self.size[1] + 1) // 2 * 2,
            "fps": self.fps,
            "frame_count": self.frames,
            "repeated_frames": self.repeats,
            "stale_frames": self.stale,
            "started_ns": self.start_ns,
            "ended_ns": self.start_ns + self.frames * self.period,
            "clock_domain": "host_record_time",
            "synchronization": "causal_hold",
            "max_age_ns": self.max_age_ns,
        }

    def abort(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        for stream in (self.process.stdin, self.sidecar, self.log):
            try:
                stream.close()
            except (OSError, ValueError):
                pass


def extract(source, output, options):
    if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        raise DataError("VIDEO_ENCODER_UNAVAILABLE", "ffmpeg and ffprobe are required")
    fps = float(options.get("fps", 30))
    if not 1 <= fps <= 120:
        raise DataError("VIDEO_FPS_INVALID")
    output = Path(output)
    output.mkdir(parents=True)
    (output / "videos").mkdir()
    cameras = {}
    try:
        for sample in iter_samples(source, topics=options.get("topics")):
            if not is_image(sample):
                continue
            image = decode_image(sample["payload"])
            topic = sample["topic"]
            if topic not in cameras:
                if len(cameras) >= int(options.get("max_cameras", 4)):
                    raise DataError("VIDEO_CAMERA_LIMIT")
                cameras[topic] = CameraEncoder(
                    output,
                    topic,
                    image,
                    sample["timestamp_ns"],
                    fps,
                    int(options.get("max_age_ns", 100_000_000)),
                    int(options.get("max_output_frames", 2_000_000)),
                )
            cameras[topic].add(sample, image)
        if not cameras:
            raise DataError(
                "NO_IMAGE_STREAMS",
                "No supported Image/CompressedImage or JSON RGB/JPEG samples",
            )
        metadata = [camera.finish() for camera in cameras.values()]
        source_name = Path(source).name
        shutil.copyfile(source, output / source_name)
        result = {
            "schema_version": "hc-video-bundle/v1",
            "fps": fps,
            "cameras": metadata,
            "total_frames": sum(c["frame_count"] for c in metadata),
            "source_preserved": True,
            "source_mcap": source_name,
            "time_rule": "host log time, causal hold; original publication/source time preserved per frame",
        }
        atomic_json(output / "video-info.json", result)
        validate_bundle(output)
        return result
    finally:
        for camera in cameras.values():
            camera.abort()


def validate_bundle(root):
    root = Path(root)
    info = json.loads((root / "video-info.json").read_text())
    if info.get("schema_version") != "hc-video-bundle/v1" or not info.get("cameras"):
        raise DataError("VIDEO_BUNDLE_INVALID")
    from .artifacts import safe_asset

    if not safe_asset(root, info.get("source_mcap", "capture.mcap")).is_file():
        raise DataError("VIDEO_BUNDLE_INVALID")
    for camera in info["cameras"]:
        path = safe_asset(root, camera["path"])
        probe = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=nb_frames,width,height",
                "-of",
                "json",
                str(path),
            ],
            capture_output=True,
            check=True,
            timeout=30,
        )
        stream = json.loads(probe.stdout)["streams"][0]
        if (
            int(stream["nb_frames"]) != camera["frame_count"]
            or stream["width"] != camera["width"]
            or stream["height"] != camera["height"]
        ):
            raise DataError("VIDEO_FRAME_COUNT_MISMATCH")
        with safe_asset(root, camera["timestamps_path"]).open() as records:
            count = sum(1 for _ in records)
        if count != camera["frame_count"]:
            raise DataError("VIDEO_TIMELINE_MISMATCH")
    return info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--options", required=True)
    args = parser.parse_args()

    def interrupted(*_):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, interrupted)
    try:
        result = extract(
            args.input, args.output, json.loads(Path(args.options).read_text())
        )
        print(json.dumps(result), flush=True)
    except BaseException as error:
        atomic_json(
            Path(args.output).parent / "conversion-error.json",
            {
                "code": getattr(error, "code", "VIDEO_EXTRACTION_FAILED"),
                "type": type(error).__name__,
            },
        )
        raise SystemExit(1)


if __name__ == "__main__":
    main()
