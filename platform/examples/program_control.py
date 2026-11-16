"""Bounded X1 simulation example; no automatic recovery after a fault or pause."""

import argparse
import asyncio
import math
import time
from hc_platform.sdk import RobotClient


async def main(url):
    async with RobotClient(url) as robot:
        runtime = await robot.call("GET", "/api/status")
        if runtime["runtime_mode"] != "sim":
            raise RuntimeError("This executable example only runs in simulation")
        info = await robot.info()
        names = info["groups"]["right_arm"]
        await robot.operation("recover")
        for _ in range(100):
            state = await robot.state()
            if state["safety"] == "ready":
                break
            await asyncio.sleep(0.02)
        await robot.operation(
            "session_start", mode="program", groups=["right_arm"], dagger_enabled=True
        )
        start = time.monotonic()
        initial = dict(zip(state["joint_names"], state["positions"]))
        try:
            while time.monotonic() - start < 4:
                state = await robot.state()
                if state["safety"] != "ready" or state.get("paused"):
                    break
                if state["owner"] == "program" or state.get("handback_waiting"):
                    sampled = time.monotonic_ns()
                    target = [initial[name] for name in names]
                    target[0] += 0.01 * math.sin(
                        (time.monotonic() - start) * math.pi / 2
                    )
                    await robot.joint_target(
                        names, target, state=state, sample_local_ns=sampled
                    )
                await asyncio.sleep(0.02)
        finally:
            await robot.operation("stop")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:7876")
    asyncio.run(main(parser.parse_args().url))
