import asyncio
from pathlib import Path

from middleware.core.app import MiddlewareRuntime
from middleware.core.config import validate_config


def test_high_rate_websocket_events_keep_latest_without_dropping_state_events(tmp_path: Path):
    async def exercise():
        runtime = MiddlewareRuntime(validate_config({}), tmp_path)
        delivered = []

        async def capture(event):
            delivered.append(event)

        runtime._broadcast = capture
        for sequence in range(100):
            runtime._schedule_broadcast(
                {"kind": "vr_pose", "payload": {"sequence": sequence}}
            )
        runtime._schedule_broadcast(
            {"kind": "safety_stop", "payload": {"reason": "test"}}
        )
        await asyncio.sleep(0)
        await asyncio.sleep(0)

        poses = [event for event in delivered if event["kind"] == "vr_pose"]
        stops = [event for event in delivered if event["kind"] == "safety_stop"]
        assert len(poses) == 1
        assert poses[0]["payload"]["sequence"] == 99
        assert len(stops) == 1

    asyncio.run(exercise())
