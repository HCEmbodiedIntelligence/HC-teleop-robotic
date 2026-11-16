import asyncio
import base64
import json
import shutil
from pathlib import Path

import pytest

from hc_platform.data import DataService
from hc_platform.data.video_worker import validate_bundle


@pytest.mark.skipif(not shutil.which("ffmpeg"), reason="ffmpeg is optional")
def test_real_mcap_to_mp4_job_and_frame_timeline(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session(
                {"local_format": "mcap_mp4", "video": {"fps": 20}}, "video"
            )
            for value in (0, 100, 200):
                await service.append_sample(
                    session["id"],
                    "/camera/rgb",
                    {
                        "encoding": "rgb8",
                        "width": 16,
                        "height": 16,
                        "data_base64": base64.b64encode(
                            bytes([value, 0, 0]) * 16 * 16
                        ).decode(),
                    },
                )
                await asyncio.sleep(0.06)
            raw = await service.stop_session(session["id"])
            for _ in range(400):
                jobs = await service.list_jobs()
                if jobs and jobs[0]["state"] in {"SUCCEEDED", "FAILED"}:
                    break
                await asyncio.sleep(0.025)
            assert jobs[0]["state"] == "SUCCEEDED", jobs
            artifact = await service.get_dataset(jobs[0]["result"]["artifact_id"])
            assert artifact["format"] == "mcap_mp4"
            info = validate_bundle(artifact["root"])
            assert info["total_frames"] >= 2
            sidecar = Path(artifact["root"]) / info["cameras"][0]["timestamps_path"]
            frames = [json.loads(line) for line in sidecar.read_text().splitlines()]
            assert len(frames) == info["total_frames"]
            assert all(
                {"source_time_ns", "recorded_time_ns", "valid", "repeated"}
                <= row.keys()
                for row in frames
            )
            assert (await service.get_dataset(raw["artifact_id"]))["state"] == "READY"
        finally:
            await service.close()

    asyncio.run(scenario())


@pytest.mark.skipif(not shutil.which("ffmpeg"), reason="ffmpeg is optional")
def test_no_image_source_fails_without_deleting_mcap(tmp_path):
    async def scenario():
        service = await DataService(tmp_path).open()
        try:
            session = await service.start_session({}, "no-camera")
            await service.append_sample(
                session["id"], "/joints", {"name": ["a"], "position": [0]}
            )
            raw = await service.stop_session(session["id"])
            job = await service.enqueue_video(raw["artifact_id"], "extract")
            for _ in range(200):
                value = await service.get_job(job["id"])
                if value["state"] == "FAILED":
                    break
                await asyncio.sleep(0.025)
            assert value["error"]["code"] == "NO_IMAGE_STREAMS"
            assert (await service.get_dataset(raw["artifact_id"]))["state"] == "READY"
        finally:
            await service.close()

    asyncio.run(scenario())
