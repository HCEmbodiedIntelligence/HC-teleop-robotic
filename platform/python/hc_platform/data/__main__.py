import argparse
import asyncio
import json
import signal

from . import DataServer, DataService


async def run(args):
    config = (
        json.loads(args.platform_config.read_text()) if args.platform_config else None
    )
    command = (
        [str(args.converter_python), "-m", "hc_platform.data.converter_worker"]
        if args.converter_python
        else None
    )
    service = await DataService(
        args.state_dir, platform_config=config, converter_command=command
    ).open()
    server = DataServer(service, args.socket)
    stop = asyncio.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(signum, stop.set)
    try:
        await server.start()
        print(
            json.dumps(
                {"service": "jobs-data", "state": "ready", "socket": args.socket}
            ),
            flush=True,
        )
        await stop.wait()
    finally:
        await server.close()
        await service.close()


def main():
    from pathlib import Path

    parser = argparse.ArgumentParser(
        description="HC data service (single SQLite owner, private Unix IPC)"
    )
    parser.add_argument("--state-dir", required=True)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--platform-config", type=Path)
    parser.add_argument(
        "--converter-python",
        type=Path,
        help="Pinned LeRobot worker interpreter; inputs stay in the data service",
    )
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
