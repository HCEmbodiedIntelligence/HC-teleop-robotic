#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROLLER_PREFIX="${HC_CONTROLLER_PREFIX:-${HOME}/miniconda3/envs/hc-teleop-controller}"
MIDDLEWARE_CONFIG="${HC_MIDDLEWARE_CONFIG:-${SCRIPT_DIR}/middleware/config.yaml}"
export PYTHONPATH="${SCRIPT_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
ROBOT_CONFIG_ROOT="${HC_ROBOT_CONFIG_ROOT:-$(/usr/bin/python3 -m middleware.profile_cli root --config "${MIDDLEWARE_CONFIG}")}"
ROBOT_NAME="${HC_ROBOT_NAME:-$(/usr/bin/python3 -m middleware.profile_cli active --config "${MIDDLEWARE_CONFIG}")}"
V23_DIR="${SCRIPT_DIR}/adapters/v23"
MODE="${1:---check}"

if [[ ! -x "${CONTROLLER_PREFIX}/bin/python" ]]; then
  echo "Generic controller environment not found: ${CONTROLLER_PREFIX}" >&2
  echo "Run: ${SCRIPT_DIR}/install.sh --sim" >&2
  exit 2
fi
CONFIG_V23="${HC_CONTROLLER_CONFIG:-${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}/controller_v23.yml}"
if [[ "${MODE}" != "--check" && ! -f "${CONFIG_V23}" ]]; then
  echo "Reconstructed controller YAML not found: ${CONFIG_V23}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

# ROS installs another Pinocchio on PYTHONPATH. The copied solver requires the
# Conda build because it includes pinocchio.casadi, so keep this path first.
export PYTHONPATH="${CONTROLLER_PREFIX}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="${CONTROLLER_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONDONTWRITEBYTECODE=1

if [[ "${MODE}" == "--check" ]]; then
  PYTHONPATH="${V23_DIR}/src:${PYTHONPATH}" "${CONTROLLER_PREFIX}/bin/python" - <<'PY'
import casadi
import pinocchio
import rclpy
from pinocchio import casadi as pinocchio_casadi
from controller_v2_3 import ControllerV23
print(
    "controller environment ready: "
    f"pinocchio={pinocchio.__version__}, casadi={casadi.__version__}, "
    f"rclpy={rclpy.__file__}, reconstructed_v23=ready"
)
PY
  exit 0
fi

CONTROLLER_COMMAND=(
  "${CONTROLLER_PREFIX}/bin/python" -u
  "${V23_DIR}/script/control_v2_3_ros2.py"
  "${CONFIG_V23}"
)

# X1 reserves CPUs 1-3 with isolcpus/nohz_full.  Keep the 100 Hz solver away
# from camera, dashboard and desktop scheduling jitter when CPU 2 is part of
# that reserved set. Other machines retain the scheduler's normal placement.
# Set HC_V23_CPU to a CPU number to override, or to an empty value to disable.
V23_CPU="${HC_V23_CPU-auto}"
if [[ "${V23_CPU}" == auto ]]; then
  V23_CPU=""
fi
if [[ -z "${HC_V23_CPU+x}" && -r /sys/devices/system/cpu/isolated ]]; then
  isolated_cpus="$(cat /sys/devices/system/cpu/isolated)"
  if /usr/bin/python3 - "${isolated_cpus}" <<'PY'
import sys

value = sys.argv[1]
cpus = set()
for part in value.split(','):
    part = part.strip()
    if not part:
        continue
    if '-' in part:
        start, end = (int(item) for item in part.split('-', 1))
        cpus.update(range(start, end + 1))
    else:
        cpus.add(int(part))
raise SystemExit(0 if 2 in cpus else 1)
PY
  then
    V23_CPU=2
  fi
fi
if [[ -n "${V23_CPU}" ]]; then
  exec taskset -c "${V23_CPU}" "${CONTROLLER_COMMAND[@]}"
fi
exec "${CONTROLLER_COMMAND[@]}"
