"""Real optional LeRobot/FFmpeg integration, isolated from the ROS interpreter."""

import asyncio
import base64
import json
import os
from pathlib import Path
import struct
import subprocess

import pytest

from hc_platform.data import DataService
from hc_platform.configuration import compile_package


def test_real_x1_mcap_to_lerobot_video_validity_and_named_mapping(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    interpreter = Path(
        os.environ.get(
            "HC_CONVERTER_PYTHON", repo / ".venv-lerobot-platform/bin/python"
        )
    )
    if not interpreter.exists():
        pytest.skip("Install the optional pinned LeRobot converter environment")
    from mcap.writer import Writer
    from PIL import Image

    config = compile_package(repo / "adapters/robots/x1", "x1")
    names = [
        name
        for name in config["joint_names"]
        if any(name in group for group in config["groups"].values())
    ]
    camera = config["camera"]["streams"][0]["topic"]
    source = tmp_path / "generated-x1.mcap"
    with source.open("wb") as stream:
        writer = Writer(stream)
        writer.start(profile="ros2")
        schema = writer.register_schema(
            "std_msgs/msg/String", "ros2msg", b"string data\n"
        )
        channels = {
            topic: writer.register_channel(topic, "cdr", schema)
            for topic in (
                "/platform/control/state",
                "/platform/control/commands",
                camera,
            )
        }
        for index in range(13):
            timestamp = 1_700_000_000_000_000_000 + index * 100_000_000
            state = {
                "joint_names": names[::-1],
                "positions": [float(names.index(name)) / 100 for name in names[::-1]],
            }
            action = {
                "stage": "core_gate",
                "owner": "program",
                "epoch": 42,
                "seq": index + 1,
                "joint_names": names[::-1],
                "positions": [
                    float(names.index(name)) / 100 + 0.1 for name in names[::-1]
                ],
            }
            messages = [
                ("/platform/control/state", {"state": state}),
                ("/platform/control/commands", action),
            ]
            if index not in {4, 5, 6, 7}:
                rgb = Image.new("RGB", (64, 48), (index * 15, 50, 200)).tobytes()
                messages.append(
                    (
                        camera,
                        {
                            "width": 64,
                            "height": 48,
                            "encoding": "rgb8",
                            "step": 192,
                            "data_base64": base64.b64encode(rgb).decode(),
                        },
                    )
                )
            for topic, payload in messages:
                body = json.dumps(
                    {"schema_version": "hc.sample/v1", "payload": payload}
                ).encode()
                cdr = b"\0\1\0\0" + struct.pack("<I", len(body) + 1) + body + b"\0"
                writer.add_message(
                    channels[topic], timestamp, cdr, timestamp - 1_000_000
                )
        writer.finish()

    async def scenario():
        service = await DataService(
            tmp_path / "state",
            converter_command=[
                str(interpreter),
                "-m",
                "hc_platform.data.converter_worker",
            ],
        ).open()
        try:
            artifact = await service.import_mcap(
                str(source),
                "x1-image-import",
                {
                    "robot_id": "x1",
                    "robot_config": config,
                    "dataset_fps": 20,
                    "task_description": "synthetic converter verification",
                },
            )
            job = await service.enqueue_conversion(artifact["id"], "real-conversion")
            for _ in range(1200):
                current = await service.get_job(job["id"])
                if current["state"] in {"SUCCEEDED", "FAILED", "CANCELED"}:
                    break
                await asyncio.sleep(0.1)
            log = tmp_path / "state/staging" / (job["id"] + ".partial") / "worker.log"
            assert current["state"] == "SUCCEEDED", (
                current,
                log.read_text() if log.exists() else "no log",
            )
            result = await service.get_dataset(current["result"]["artifact_id"])
            root = Path(result["root"])
            info = json.loads((root / "meta/info.json").read_text())
            assert info["codebase_version"] == "v3.0"
            assert info["robot_type"] == "x1" and info["fps"] == 20
            assert info["features"]["action"]["names"] == names
            assert info["features"]["observation.source_valid"]["dtype"] == "bool"
            provenance = [
                json.loads(line)
                for line in (root / "sample-provenance.jsonl").read_text().splitlines()
            ]
            skipped = [row for row in provenance if row.get("event") == "skipped_tick"]
            assert skipped and any(
                not row["sources"]["observation.images.head"]["valid"]
                for row in skipped
            )
            assert any(
                row.get("sources", {}).get("action", {}).get("control", {}).get("stage")
                == "core_gate"
                for row in provenance
            )
            conversion = json.loads((root / "conversion.json").read_text())
            assert conversion["skipped_ticks"] == len(skipped)
            assert conversion["task_success"] is None
            assert conversion["exported_frames"] == info["total_frames"]
            assert result["summary"]["camera_count"] == 1
            for path in (root / "videos").rglob("*.mp4"):
                probe = subprocess.run(
                    [
                        "ffprobe",
                        "-v",
                        "error",
                        "-count_frames",
                        "-select_streams",
                        "v:0",
                        "-show_entries",
                        "stream=nb_read_frames,width,height",
                        "-of",
                        "json",
                        str(path),
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                video = json.loads(probe.stdout)["streams"][0]
                assert (video["width"], video["height"]) == (64, 48) and int(
                    video["nb_read_frames"]
                ) > 0
            environment = {
                key: value
                for key, value in os.environ.items()
                if key not in {"PYTHONPATH", "PYTHONHOME"}
            }
            verified = subprocess.run(
                [
                    str(interpreter),
                    "-c",
                    "import json,sys; from lerobot.datasets.lerobot_dataset import LeRobotDataset; "
                    "d=LeRobotDataset('local/hc_export',root=sys.argv[1],video_backend='pyav'); "
                    "f=d[0]; print(json.dumps({'state':f['observation.state'].tolist(),'action':f['action'].tolist(),"
                    "'mask':f['observation.source_valid'].tolist(),'image':list(f['observation.images.head'].shape)}))",
                    str(root),
                ],
                env=environment,
                capture_output=True,
                text=True,
                timeout=60,
                check=True,
            )
            row = json.loads(verified.stdout.strip().splitlines()[-1])
            assert row["state"] == pytest.approx(
                [index / 100 for index in range(len(names))]
            )
            assert row["action"] == pytest.approx(
                [index / 100 + 0.1 for index in range(len(names))]
            )
            assert row["mask"] == [True, True, True] and row["image"] == [3, 48, 64]
        finally:
            await service.close()

    asyncio.run(scenario())
