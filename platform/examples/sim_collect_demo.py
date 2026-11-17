"""End-to-end local simulation data collection demo: MCAP + camera + LeRobot v3 export."""

import argparse
import asyncio
import json
import math
import time

from hc_platform.sdk import RobotClient


async def main(url: str, duration: float, output_format: str, filename: str):
    import httpx

    async with RobotClient(url) as robot:
        robot.http.timeout = httpx.Timeout(30.0)
        status = await robot.call("GET", "/api/status")
        if status.get("runtime_mode") != "sim":
            raise RuntimeError("This demo requires ./run.sh sim")

        preflight = await robot.call(
            "POST",
            "/api/recordings/preflight",
            {"format": output_format, "local_format": output_format},
        )
        if not preflight.get("ready"):
            raise RuntimeError(f"Preflight not ready: {preflight}")

        await robot.operation("recover")
        for _ in range(100):
            state = await robot.state()
            if state["safety"] == "ready" and not state["transition_pending"]:
                break
            await asyncio.sleep(0.02)

        started_rec = await robot.call(
            "POST",
            "/api/recordings",
            {
                "format": output_format,
                "local_format": output_format,
                "filename": filename,
                "dataset_fps": 30,
                "task_description": "simulation reach trajectory",
                "request_id": f"sim-demo-{time.time_ns()}",
            },
        )
        session_id = started_rec["id"]
        print(f"Recording started: session_id={session_id} format={output_format}")

        info = await robot.info()
        names = info["groups"]["right_arm"]
        await robot.operation("session_start", mode="program", groups=["right_arm"])
        for _ in range(100):
            state = await robot.state()
            if state["owner"] == "program" and not state["transition_pending"]:
                break
            await asyncio.sleep(0.02)

        initial = dict(zip(state["joint_names"], state["positions"]))
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < duration:
                state = await robot.state()
                if state["safety"] != "ready" or state.get("paused"):
                    break
                phase = time.monotonic() - t0
                target = [initial[n] for n in names]
                target[0] += 0.18 * math.sin(phase * math.pi * 0.6)
                target[1] += 0.12 * math.sin(phase * math.pi * 0.6)
                target[3] += 0.15 * math.sin(phase * math.pi * 0.6)
                await robot.joint_target(
                    names, target, state=state, sample_local_ns=time.monotonic_ns()
                )
                await asyncio.sleep(0.02)
        finally:
            try:
                await robot.operation("session_end")
            except Exception:
                await robot.operation("stop")
            stopped = await robot.call("POST", "/api/recordings/stop")
            await robot.operation("stop")

        print(f"MCAP finalized: {stopped.get('root')}")
        dataset = await robot.call("GET", f"/api/datasets/{session_id}")
        if output_format != "mcap":
            for _ in range(300):
                jobs = await robot.call("GET", "/api/jobs")
                matching = [j for j in jobs if j.get("artifact_id") == session_id]
                if matching and matching[-1]["state"] in {"SUCCEEDED", "FAILED", "CANCELED"}:
                    print("Postprocess job:", json.dumps(matching[-1], ensure_ascii=False, indent=2))
                    break
                await asyncio.sleep(0.2)
            datasets = await robot.call("GET", "/api/datasets")
            derived = [d for d in datasets if d.get("session_id") == session_id]
            print("Datasets:", json.dumps(derived, ensure_ascii=False, indent=2))
        else:
            print("Dataset:", json.dumps(dataset.get("summary"), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:7876")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument(
        "--format",
        default="mcap_lerobot",
        choices=("mcap", "lerobot", "mcap_lerobot", "mcap_mp4"),
    )
    parser.add_argument("--filename", default="sim_demo.mcap")
    args = parser.parse_args()
    asyncio.run(main(args.url, args.duration, args.format, args.filename))
