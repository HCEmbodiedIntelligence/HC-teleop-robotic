"""The HC launcher must not own an HTTP gateway process."""
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_teleop_launcher_has_no_holobrain_gateway_lifecycle():
    launcher = (ROOT / "run.sh").read_text(encoding="utf-8")
    forbidden = (
        "GATEWAY_ENTRY",
        "GATEWAY_PID",
        "HC_TJ_GATEWAY_ROOT",
        "HC_TJ_GATEWAY_PORT",
        "start_hc_tj_dagger_gateway.sh",
        "holobrain_gateway.log",
    )
    assert not any(token in launcher for token in forbidden)


def test_real_robot_uses_the_hardware_fastdds_profile():
    launcher = (ROOT / "run.sh").read_text(encoding="utf-8")
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE=' in launcher
    assert '${HOME}/shm_profile.xml' in launcher
    assert 'export ROS_LOCALHOST_ONLY=0' in launcher


def test_launcher_shell_syntax():
    subprocess.run(
        ["bash", "-n", str(ROOT / "run.sh")],
        check=True,
        capture_output=True,
        text=True,
    )
