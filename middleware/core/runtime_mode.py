"""Choose a ROS transport domain before any middleware or child ROS node starts."""
from __future__ import annotations

import argparse
import os
import secrets
from collections.abc import MutableMapping


def _domain(value: str | int) -> int:
    domain = int(value)
    if not 0 <= domain <= 232:
        raise ValueError("ROS domain must be between 0 and 232")
    return domain


def configure_environment(
    configured_domain: int,
    environ: MutableMapping[str, str] | None = None,
) -> int:
    """Set one shared domain for this stack, isolating simulation from hardware.

    Call before constructing any ROS context or launching child processes. The
    simulation domain is cached in the environment so the gateway, controllers,
    and PyBullet reuse it. Only explicit ``HC_RUNTIME_MODE=teleop`` opts into the
    hardware domain; unknown modes fail before ROS starts.
    """
    env = os.environ if environ is None else environ
    mode = env.get("HC_RUNTIME_MODE", "simulation")
    if mode not in {"simulation", "teleop"}:
        raise ValueError(f"Unsupported HC_RUNTIME_MODE: {mode}")
    configured = _domain(configured_domain)
    cached = env.get("HC_SIMULATION_DOMAIN_ID")
    current = env.get("ROS_DOMAIN_ID")
    # A child already inherits the chosen simulation ROS_DOMAIN_ID. Preserve the
    # original hardware ID instead of mistaking this inherited domain for it.
    if cached and current == cached:
        hardware = _domain(env.get("HC_HARDWARE_DOMAIN_ID", configured))
    else:
        hardware = _domain(current if current is not None else configured)
    env["HC_HARDWARE_DOMAIN_ID"] = str(hardware)
    env["HC_RUNTIME_MODE"] = mode
    if mode == "teleop":
        env.pop("HC_SIMULATION_DOMAIN_ID", None)
        env["ROS_DOMAIN_ID"] = str(hardware)
        return hardware

    forbidden = {hardware}
    if str(configured) != cached or current != cached:
        forbidden.add(configured)
    try:
        simulated = _domain(cached) if cached is not None else None
    except (TypeError, ValueError):
        simulated = None
    if simulated is None or not 100 <= simulated <= 220 or simulated in forbidden:
        simulated = secrets.choice([item for item in range(100, 221) if item not in forbidden])
    env["HC_SIMULATION_DOMAIN_ID"] = str(simulated)
    env["ROS_DOMAIN_ID"] = str(simulated)
    env["ROS_LOCALHOST_ONLY"] = "1"
    return simulated


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config")
    parser.add_argument("--shell-values", action="store_true")
    args = parser.parse_args()
    import yaml

    with open(args.config, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    domain = configure_environment(config.get("ros", {}).get("domain_id", 14))
    if args.shell_values:
        print(domain, os.environ["HC_HARDWARE_DOMAIN_ID"])
    else:
        print(domain)


if __name__ == "__main__":
    main()
