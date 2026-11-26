"""Loopback-only browser QA fixture. Takes a bundle of the deployed platform reader/sender."""

import asyncio
import json
import os
from pathlib import Path
import signal
import tempfile
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
from aiohttp import web

from middleware.core.camera import CameraService
from middleware.platform_device.bench import Rendezvous
from middleware.platform_device.__main__ import camera_task
from middleware.platform_device.local import LocalServer
from middleware.core.protocol import decode_pose_packet


async def main(bundle):
    relay = Rendezvous()
    app = web.Application()
    app.router.add_post("/api/v1/robot-ingest/teleoperation/sync", relay.sync)

    async def page(r):
        return web.Response(
            text='<video autoplay muted playsinline></video><script src="/web.js"></script><script src="/qa.js"></script>',
            content_type="text/html",
        )

    async def offer(r):
        relay.offer = (await r.json())["sdp"]
        return web.json_response(dict(session=relay.sid))

    async def answer(r):
        return web.json_response(dict(sdp=relay.answer))

    app.router.add_get("/", page)
    app.router.add_get("/web.js", lambda r: web.FileResponse(bundle))
    app.router.add_get(
        "/qa.js",
        lambda r: web.FileResponse(
            Path(__file__).with_name("platform_device_browser.js")
        ),
    )
    app.router.add_post("/offer", offer)
    app.router.add_get("/answer", answer)
    app.router.add_get("/session", lambda r: web.json_response({"id": relay.sid}))
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    site = web.TCPSite(
        runner, "127.0.0.1", int(sys.argv[2]) if len(sys.argv) > 2 else 0
    )
    await site.start()
    port = runner.addresses[0][1]
    print("PORT", port, flush=True)
    os.environ["HC_TELEOP_ROBOT_TOKEN"] = relay.token
    camera = CameraService(
        dict(id="fixture", enabled=True, source="ros", topic="/qa", fps=30), 14
    )
    camera.start()
    directory = tempfile.TemporaryDirectory()
    os.environ["XDG_RUNTIME_DIR"] = directory.name
    wire = {"packets": 0, "stops": 0}

    class FakeActuator(asyncio.DatagramProtocol):
        def datagram_received(self, data, addr):
            packet = decode_pose_packet(data)
            if addr[0] != "127.0.0.2":
                return
            wire.update(packets=wire["packets"] + 1, trigger=packet.right_input.trigger)

    udp, _ = await asyncio.get_running_loop().create_datagram_endpoint(
        FakeActuator, local_addr=("127.0.0.1", 0)
    )

    def stop(reason):
        wire["stops"] += 1
        print("WIRE-STOP", json.dumps(wire), flush=True)

    rt = SimpleNamespace(
        platform=None,
        config={"vr": {"pose_port": udp.get_extra_info("sockname")[1]}},
        vr=SimpleNamespace(
            status=lambda: {
                "state": "running",
                "active_peer": "127.0.0.2" if wire["packets"] else None,
            }
        ),
        ros=SimpleNamespace(
            status=lambda: {
                "state": "running",
                "command_mux": {
                    "enabled": True,
                    "source": "vr",
                    "output_enabled": True,
                    "hardware_ready": True,
                }
            }
        ),
        _on_vr_timeout=stop,
    )
    cfg = dict(
        mode="live",
        name="fake-hardware",
        base_url=f"http://127.0.0.1:{port}",
        camera_id="fixture",
        max_lag_ms=120,
        watchdog_ms=200,
    )
    path = Path(directory.name) / "fixture.json"
    path.write_text(json.dumps(cfg))
    os.environ["HC_TELEOP_DEVICE_CONFIG"] = str(path)
    local = LocalServer(rt, cfg)
    await local.start()
    platform_task = camera_task(camera)

    async def publish():
        n = 0
        image = np.zeros((360, 640, 3), np.uint8)
        while True:
            image[:] = 12
            cv2.rectangle(image, (n % 560, 40), (n % 560 + 70, 300), (220, 155, 70), -1)
            n += 6
            ok, jpeg = cv2.imencode(".jpg", image)
            assert ok
            ns = time.time_ns()
            msg = SimpleNamespace(
                format="jpeg",
                data=jpeg.tobytes(),
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9)
                ),
            )
            camera.handle_ros_message("/qa", msg)
            await asyncio.sleep(1 / 30)

    tasks = [asyncio.create_task(publish()), platform_task]
    done = asyncio.Event()
    asyncio.get_running_loop().add_signal_handler(signal.SIGTERM, done.set)
    try:
        while not done.is_set():
            if hasattr(camera, "platform_status"):
                print("METRICS", json.dumps(camera.platform_status()), flush=True)
            try:
                await asyncio.wait_for(done.wait(), 2)
            except asyncio.TimeoutError:
                pass
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await local.close()
        udp.close()
        directory.cleanup()
        await camera.stop()
        await runner.cleanup()


if __name__ == "__main__":
    asyncio.run(main(Path(sys.argv[1]).resolve()))
