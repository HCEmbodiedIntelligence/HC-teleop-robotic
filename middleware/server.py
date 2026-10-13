#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS 2 / VR teleoperation middleware")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "config.yaml"),
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

    from middleware.core.app import create_app
    from middleware.core.network import local_addresses
    from middleware.core.config import ConfigStore

    store = ConfigStore(args.config)
    config = store.load()
    host = args.host or config["server"]["host"]
    port = args.port or config["server"]["port"]
    addresses = local_addresses() if host == "0.0.0.0" else [("bound", host)]
    for interface, address in addresses:
        print(f"Dashboard [{interface}]: http://{address}:{port}/dashboard/")
        print(f"WebSocket [{interface}]: ws://{address}:{port}/ws")
    print("请选择与 PICO 互通的网卡地址；VPN/TUN 地址不一定能被头显访问。")
    print(f"Config: {store.path}")
    web.run_app(create_app(store), host=host, port=port)


if __name__ == "__main__":
    main()
