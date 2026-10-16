import os
import subprocess
import sys
from pathlib import Path

import pytest

from middleware.core.runtime_mode import configure_environment


def test_default_mode_isolates_explicit_hardware_domain():
    env = {"ROS_DOMAIN_ID": "14", "ROS_LOCALHOST_ONLY": "0"}
    domain = configure_environment(14, env)
    assert domain != 14
    assert 100 <= domain <= 220
    assert env["HC_RUNTIME_MODE"] == "simulation"
    assert env["HC_HARDWARE_DOMAIN_ID"] == "14"
    assert env["HC_SIMULATION_DOMAIN_ID"] == str(domain)
    assert env["ROS_LOCALHOST_ONLY"] == "1"


def test_configured_and_environment_hardware_domains_both_excluded():
    env = {"ROS_DOMAIN_ID": "110", "HC_SIMULATION_DOMAIN_ID": "120"}
    assert configure_environment(120, env) not in {110, 120}
    assert env["HC_HARDWARE_DOMAIN_ID"] == "110"


def test_children_and_runtime_restart_keep_one_simulation_domain():
    env = {"ROS_DOMAIN_ID": "14"}
    domain = configure_environment(14, env)
    assert configure_environment(14, env) == domain
    # Runtime's effective config also contains the private domain.
    assert configure_environment(domain, env) == domain
    assert env["HC_HARDWARE_DOMAIN_ID"] == "14"


def test_config_domain_used_without_explicit_domain():
    env = {}
    assert configure_environment(17, env) != 17
    assert env["HC_HARDWARE_DOMAIN_ID"] == "17"


def test_teleop_keeps_hardware_domain_and_localhost_setting():
    env = {"HC_RUNTIME_MODE": "teleop", "ROS_DOMAIN_ID": "14", "ROS_LOCALHOST_ONLY": "0"}
    assert configure_environment(17, env) == 14
    assert env["ROS_LOCALHOST_ONLY"] == "0"
    assert "HC_SIMULATION_DOMAIN_ID" not in env


def test_teleop_clears_simulation_cache_and_restores_hardware_domain():
    env = {"ROS_DOMAIN_ID": "14"}
    configure_environment(14, env)
    env["HC_RUNTIME_MODE"] = "teleop"
    assert configure_environment(14, env) == 14
    assert "HC_SIMULATION_DOMAIN_ID" not in env


def test_invalid_mode_fails_before_ros_setup():
    with pytest.raises(ValueError, match="HC_RUNTIME_MODE"):
        configure_environment(14, {"HC_RUNTIME_MODE": "typo"})


def test_cli_emits_effective_and_hardware_domain(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text("ros:\n  domain_id: 14\n")
    env = {key: value for key, value in os.environ.items() if not key.startswith("HC_")}
    env["ROS_DOMAIN_ID"] = "17"
    result = subprocess.run(
        [sys.executable, "-m", "middleware.core.runtime_mode", str(config), "--shell-values"],
        check=True, capture_output=True, text=True, env=env,
    )
    effective, hardware = result.stdout.split()
    assert int(effective) not in {14, 17}
    assert hardware == "17"


def test_bare_launcher_ignores_inherited_teleop_and_only_starts_middleware(tmp_path):
    root = Path(__file__).resolve().parents[1]
    launcher = tmp_path / "run.sh"
    launcher.write_bytes((root / "run.sh").read_bytes())
    launcher.chmod(0o755)
    middleware = tmp_path / "middleware"
    middleware.mkdir()
    (middleware / "config.yaml").write_text("ros: {domain_id: 14}\n")
    stub = middleware / "start.sh"
    stub.write_text(
        '#!/usr/bin/env bash\n'
        'printf "%s %s %s\\n" "$HC_RUNTIME_MODE" "${HC_EXTERNAL_STACK:-unset}" "${HC_TELEOP_MODE:-unset}"\n'
    )
    stub.chmod(0o755)
    env = dict(os.environ, HC_RUNTIME_MODE="teleop", HC_EXTERNAL_STACK="1", HC_TELEOP_MODE="teleop")
    env.pop("HC_MIDDLEWARE_CONFIG", None)
    result = subprocess.run([str(launcher)], env=env, check=True, capture_output=True, text=True)
    assert result.stdout.strip() == "simulation unset unset"
