import asyncio
import copy
import os
from types import SimpleNamespace

import pytest

from hc_platform.motion import MotionService, MotionError
from test_motion_planner import motion_config


class Core:
    def __init__(self, config):
        self.state = {
            "boot_id": "test",
            "epoch": 1,
            "lease_id": 1,
            "reference_epoch": 1,
            "safety": "ready",
            "mode": "program",
            "owner": "program",
            "transition_pending": False,
            "paused": False,
            "joint_names": ["x", "y"],
            "positions": [0.0, 0.0],
            "fresh_joint_names": ["x", "y"],
            "available_groups": ["right_arm"],
            "leased_groups": ["right_arm"],
            "feedback_seq": 1,
            "last_command_seq": 0,
        }
        self.pauses = 0
        self.commands = []

    async def request(self, op, **payload):
        if op == "command":
            if (
                self.state["owner"] != "program"
                or payload["epoch"] != self.state["epoch"]
            ):
                raise MotionError("CONTROL_EPOCH_CHANGED")
            assert payload["sample_mono_ns"] <= payload["deadline_mono_ns"]
            values = dict(zip(self.state["joint_names"], self.state["positions"]))
            values.update(zip(payload["joint_names"], payload["positions"]))
            self.state["positions"] = [
                values[name] for name in self.state["joint_names"]
            ]
            self.state["feedback_seq"] += 1
            self.state["last_command_seq"] = payload["seq"]
            self.commands.append(payload)
        elif op == "pause":
            assert payload["expected_epoch"] == self.state["epoch"]
            self.pauses += 1
            self.state.update(
                owner="none", paused=True, epoch=self.state["epoch"] + 1, lease_id=0
            )
        elif op == "resume":
            self.state.update(
                owner="program",
                paused=False,
                epoch=self.state["epoch"] + 1,
                lease_id=self.state["epoch"] + 1,
                reference_epoch=self.state["reference_epoch"] + 1,
            )
        return {"ok": True, "state": copy.deepcopy(self.state)}


def runtime(config, tmp_path):
    core = Core(config)
    return SimpleNamespace(
        config=config,
        core=core,
        run_dir=tmp_path,
        source_clients={"program": core},
        environment=lambda: dict(os.environ),
    )


async def wait_task(service, task_id, seconds=8):
    end = asyncio.get_running_loop().time() + seconds
    while asyncio.get_running_loop().time() < end:
        task = await service.get_task(task_id)
        if task["status"] in ("succeeded", "failed", "aborted", "cancelled"):
            return task
        await asyncio.sleep(0.02)
    raise AssertionError(await service.get_task(task_id))


def test_v23_has_no_false_motion_capabilities(motion_config, tmp_path):
    async def run():
        motion_config["control_backend"] = "v23"
        service = MotionService(runtime(motion_config, tmp_path))
        capabilities = await service.capabilities()
        assert all(not value["backend_supported"] for value in capabilities.values())
        assert service.worker.process is None
        with pytest.raises(MotionError):
            await service.move_j({})
        await service.close()

    asyncio.run(run())


def test_real_worker_move_servo_cancel_and_reuse(motion_config, tmp_path):
    async def run():
        rt = runtime(motion_config, tmp_path)
        service = MotionService(rt)
        try:
            description = await service.describe()
            assert description["groups"]["right_arm"]["commands"] == [
                "ServoJ",
                "ServoP",
                "MoveJ",
                "MoveL",
                "MoveP",
            ]
            assert service.worker.process.pid != os.getpid()
            servo = await service.servo_j(
                {
                    "group": "right_arm",
                    "joint_names": ["x"],
                    "positions": [0.01],
                    "ttl_ms": 500,
                }
            )
            assert servo["status"] == "accepted"
            pose = await service.servo_p(
                {
                    "group": "right_arm",
                    "pose": [0.02, 0.01, 0, 0, 0, 0, 1],
                    "ttl_ms": 500,
                }
            )
            assert pose["status"] == "accepted"
            task = await service.move_j(
                {
                    "group": "right_arm",
                    "joint_names": ["x", "y"],
                    "positions": [0.04, 0.02],
                    "speed_scale": 0.5,
                }
            )
            with pytest.raises(MotionError) as caught:
                await service.servo_j(
                    {"group": "right_arm", "joint_names": ["x"], "positions": [0]}
                )
            assert caught.value.code == "MOTION_BUSY"
            result = await wait_task(service, task["id"])
            assert result["status"] == "succeeded", result
            assert result["result"]["feedback_confirmed"] and rt.core.state["paused"]
            assert not service.busy
            await rt.core.request("resume")
            task = await service.move_l(
                {
                    "group": "right_arm",
                    "pose": [0.045, 0.025, 0, 0, 0, 0, 1],
                    "speed_scale": 1,
                    "linear_velocity": 0.1,
                }
            )
            result = await wait_task(service, task["id"])
            assert result["status"] == "succeeded", result
            await rt.core.request("resume")
            task = await service.move_p(
                {
                    "group": "right_arm",
                    "pose": [0.7, 0.7, 0, 0, 0, 0, 1],
                    "speed_scale": 0.2,
                }
            )
            await asyncio.sleep(0.08)
            result = await service.cancel(task["id"])
            assert result["status"] == "cancelled", result
            assert result["result"]["cancel_boundary"]["revoked"]
            await service.close()
            await rt.core.request("resume")
            task = await service.move_j(
                {
                    "group": "right_arm",
                    "joint_names": ["x"],
                    "positions": [0.05],
                    "speed_scale": 1,
                }
            )
            result = await wait_task(service, task["id"])
            assert result["status"] == "succeeded", result
        finally:
            await service.close()

    asyncio.run(run())


def test_handover_aborts_old_trajectory_without_pausing_human(motion_config, tmp_path):
    async def run():
        rt = runtime(motion_config, tmp_path)
        service = MotionService(rt)
        try:
            await service.describe()
            task = await service.move_j(
                {
                    "group": "right_arm",
                    "joint_names": ["x"],
                    "positions": [0.5],
                    "speed_scale": 0.1,
                }
            )
            await asyncio.sleep(0.12)
            rt.core.state.update(owner="human", epoch=2, lease_id=2, reference_epoch=2)
            count = len(rt.core.commands)
            result = await wait_task(service, task["id"])
            assert result["status"] == "aborted", result
            assert rt.core.state["owner"] == "human" and rt.core.pauses == 0
            assert len(rt.core.commands) == count
        finally:
            await service.close()

    asyncio.run(run())


def test_raw_validation_in_v23_keeps_original_deadline(motion_config, tmp_path):
    async def run():
        import time

        motion_config["control_backend"] = "v23"
        rt = runtime(motion_config, tmp_path)
        service = MotionService(rt)
        sample = time.monotonic_ns() - 10_000_000
        payload = {
            "source": "program",
            "joint_names": ["x"],
            "positions": [0.1],
            "ttl_ms": 100,
            "seq": 1,
            "sample_mono_ns": sample,
            **{
                key: rt.core.state[key]
                for key in ("boot_id", "epoch", "lease_id", "reference_epoch")
            },
        }
        checked = await service.validate_external(payload)
        assert checked["sample_mono_ns"] == sample
        assert checked["deadline_mono_ns"] == sample + 100_000_000
        assert service.worker.process is None
        with pytest.raises(MotionError) as caught:
            await service.validate_external(
                dict(payload, sample_mono_ns=sample - 200_000_000)
            )
        assert caught.value.code == "TARGET_EXPIRED"
        with pytest.raises(MotionError) as caught:
            await service.validate_external(dict(payload, positions=[2]))
        assert caught.value.code == "JOINT_LIMIT"
        rt.core.state.update(
            owner="none", handback_waiting=True, requested_owner="program"
        )
        assert (await service.validate_external(payload))["source"] == "program"
        await service.close()

    asyncio.run(run())


def test_home_validation_pauses_and_requires_stable_fresh_feedback(
    motion_config, tmp_path
):
    async def run():
        class HeartbeatCore(Core):
            async def request(self, op, **payload):
                if op == "status":
                    self.state["feedback_seq"] += 1
                return await super().request(op, **payload)

        class Checker:
            def __init__(self):
                self.requests = []

            async def request(self, op, **payload):
                assert op == "validate_home"
                self.requests.append(payload)
                return {"validated": True}

            async def close(self):
                pass

        rt = runtime(motion_config, tmp_path)
        rt.core = HeartbeatCore(motion_config)
        rt.source_clients["program"] = rt.core
        motion_config["motion"]["collision_enabled"] = True
        motion_config["home_joint_names"] = ["x"]
        service = MotionService(rt)
        checker = Checker()
        service.worker = checker
        tokens = await service.validate_home()
        assert rt.core.pauses == 1 and rt.core.state["paused"]
        assert tokens == {"expected_boot_id": "test", "expected_epoch": 2}
        assert checker.requests[0]["request"]["joint_names"] == ["x"]
        assert set(checker.requests[0]["feedback"]) == {"x", "y"}
        await service.close()

    asyncio.run(run())
