#!/usr/bin/env python3
"""Command-line access to the solver plugin registry."""
from __future__ import annotations

import argparse

from .registry import available_plugins, motion_server_resources, resolve_plugin


def main() -> None:
    parser = argparse.ArgumentParser(description="HC teleop solver plugin registry")
    subparsers = parser.add_subparsers(dest="command", required=True)
    resolve = subparsers.add_parser("resolve")
    resolve.add_argument("--config", required=True)
    resolve.add_argument("--override")
    resources = subparsers.add_parser("motion-resources")
    resources.add_argument("--config", required=True)
    subparsers.add_parser("list")
    args = parser.parse_args()
    if args.command == "list":
        for plugin in available_plugins():
            print(f"{plugin.name}\t{plugin.display_name}")
        return
    if args.command == "motion-resources":
        try:
            paths = motion_server_resources(args.config)
        except ValueError as error:
            parser.error(str(error))
        for path in paths:
            print(path)
        return
    try:
        plugin = resolve_plugin(args.config, args.override)
    except ValueError as error:
        parser.error(str(error))
    print(plugin.name)


if __name__ == "__main__":
    main()
