"""The Motion task actually reaches feedback through C++ fencing and X1 motors."""

import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import pytest

from hc_platform.configuration import compile_package
from hc_platform.ipc import CoreClient
from hc_platform.motion import MotionService
from hc_platform.adapters.model import UrdfModel, matrix_quaternion


def test_motion_core_and_x1_simulation_feedback():
    pytest.importorskip("pybullet")
    repo = Path(__file__).resolve().parents[2]
    binary = repo / "platform/control/build/hc-control"
    if not binary.exists():
        pytest.skip("build C++ control authority first")

    async def run(root):
        config = compile_package(repo / "adapters/robots/x1")
        config.update(
            control_backend="motion",
            allowed_groups=["right_arm"],
            enable_base=False,
            enable_grippers=False,
        )
        config["motion"] = {
            "collision_enabled": False,
            "stream_hz": 50,
            "settle_timeout_s": 8,
        }
        config_path = root / "config.json"
        config_path.write_text(json.dumps(config))
        sock = root / "control.sock"
        env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
        env["PYTHONPATH"] = (
            str(repo / ".platform-deps")
            + os.pathsep
            + str(repo / "platform/python")
            + os.pathsep
            + os.environ.get("PYTHONPATH", "")
        )
        streams = []
        children = []
        api = CoreClient(str(sock))
        program = CoreClient(str(sock), "program")
        service = MotionService(
            SimpleNamespace(
                config=config,
                core=api,
                source_clients={"program": program},
                run_dir=root,
                environment=lambda: env,
            )
        )

        def launch(name, args):
            output = (root / (name + ".log")).open("w+")
            streams.append(output)
            children.append(
                subprocess.Popen(
                    args, cwd=repo, env=env, stdout=output, stderr=subprocess.STDOUT
                )
            )

        async def state_until(predicate, timeout=6):
            deadline = asyncio.get_running_loop().time() + timeout
            state = {}
            while asyncio.get_running_loop().time() < deadline:
                state = (await api.request("status"))["state"]
                if predicate(state):
                    return state
                await asyncio.sleep(0.02)
            raise AssertionError(state)

        async def task_until(task_id):
            deadline = asyncio.get_running_loop().time() + 12
            task = {}
            while asyncio.get_running_loop().time() < deadline:
                task = await service.get_task(task_id)
                if task["status"] in ("succeeded", "failed", "aborted", "cancelled"):
                    return task
                await asyncio.sleep(0.02)
            raise AssertionError(task)

        try:
            launch(
                "core",
                [str(binary), "--socket", str(sock), "--config", str(config_path)],
            )
            for _ in range(250):
                if sock.exists():
                    break
                await asyncio.sleep(0.02)
            await api.connect()
            launch(
                "backend",
                [
                    sys.executable,
                    "-m",
                    "hc_platform.adapters.runtime",
                    "--role",
                    "backend",
                    "--backend",
                    "simulation",
                    "--config",
                    str(config_path),
                    "--socket",
                    str(sock),
                    "--state-dir",
                    str(root),
                ],
            )
            await state_until(
                lambda s: s["backend_ready"] and "right_arm" in s["available_groups"]
            )
            await service.describe()
            await api.request("recover")
            await state_until(
                lambda s: s["safety"] == "ready" and not s["transition_pending"]
            )
            await api.request("session_start", mode="program", groups=["right_arm"])
            initial = await state_until(lambda s: s["owner"] == "program")
            q = dict(zip(initial["joint_names"], initial["positions"]))
            name = config["groups"]["right_arm"][0]
            goal = q[name] + 0.025
            task = await service.move_j(
                {
                    "group": "right_arm",
                    "joint_names": [name],
                    "positions": [goal],
                    "speed_scale": 0.5,
                }
            )
            result = await task_until(task["id"])
            assert result["status"] == "succeeded", result
            final = await state_until(lambda s: s["paused"] and s["owner"] == "none")
            measured = dict(zip(final["joint_names"], final["positions"]))
            assert abs(measured[name] - goal) < 0.015
            assert abs(measured[name] - q[name]) > 0.01
            # Cartesian motion is solved outside the API and executed through
            # exactly the same core source gate; completion uses motor feedback.
            model = UrdfModel(config["urdf"], config["package_root"])
            arm = config["arms"]["right"]
            pose = model.fk(measured, arm["ee_link"], arm["base_link"])
            pose[0, 3] += 0.002
            target = list(map(float, pose[:3, 3])) + list(
                map(float, matrix_quaternion(pose[:3, :3]))
            )
            await api.request("resume")
            await state_until(
                lambda s: s["owner"] == "program" and not s["transition_pending"]
            )
            task = await service.move_l(
                {
                    "group": "right_arm",
                    "pose": target,
                    "frame_id": arm["base_link"],
                    "speed_scale": 0.5,
                    "linear_velocity": 0.05,
                }
            )
            result = await task_until(task["id"])
            assert result["status"] == "succeeded", result
            assert result["result"]["feedback_confirmed"]
            assert result["result"]["cancel_boundary"]["revoked"]
        except Exception:
            for stream in streams:
                stream.flush()
                stream.seek(0)
                print(stream.name + "\n" + stream.read())
            raise
        finally:
            await service.close()
            await program.close()
            await api.close()
            for process in reversed(children):
                process.terminate()
            for process in reversed(children):
                try:
                    await asyncio.to_thread(process.wait, 3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    await asyncio.to_thread(process.wait, 2)
            for stream in streams:
                stream.close()

    with tempfile.TemporaryDirectory(prefix="hc-motion-e2e-") as directory:
        asyncio.run(run(Path(directory)))
