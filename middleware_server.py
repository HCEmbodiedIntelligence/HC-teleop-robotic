#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS 2 / VR teleoperation middleware")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("middleware.yaml")),
        help="YAML configuration path",
    )
    parser.add_argument("--host", help="override HTTP listen host")
    parser.add_argument("--port", type=int, help="override HTTP listen port")
    args = parser.parse_args()

    try:
        from aiohttp import web
    except ImportError:
        print(
            "Missing aiohttp. Run ./install.sh, then start this program again.",
            file=sys.stderr,
        )
        raise SystemExit(2)

    from hc_teleop_middleware.app import create_app, local_ip
    from hc_teleop_middleware.config import ConfigStore

    store = ConfigStore(args.config)
    config = store.load()
    host = args.host or config["server"]["host"]
    port = args.port or config["server"]["port"]
    print(f"Dashboard: http://{local_ip()}:{port}/dashboard/")
    print(f"WebSocket: ws://{local_ip()}:{port}/ws")
    print(f"Config: {store.path}")
    web.run_app(create_app(store), host=host, port=port)


if __name__ == "__main__":
    main()
