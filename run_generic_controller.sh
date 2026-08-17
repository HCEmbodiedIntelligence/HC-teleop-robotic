#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HC_IO_ROOT="${HC_IO_ROOT:-/home/maple/hc_io_suit}"
CONTROLLER_PREFIX="${HC_CONTROLLER_PREFIX:-/home/maple/miniconda3/envs/hc-teleop-controller}"
ROBOT_CONFIG_ROOT="${HC_ROBOT_CONFIG_ROOT:-${SCRIPT_DIR}/robot_configs}"
ROBOT_NAME="${HC_ROBOT_NAME:-hc_tj_description}"
CONTROL_DIR="${SCRIPT_DIR}/vendor/io_unicontroller_ros2/control"
V23_DIR="${SCRIPT_DIR}/vendor/io_unicontroller_ros2/control_v23_reconstructed"
MODE="${1:---check}"

if [[ ! -x "${CONTROLLER_PREFIX}/bin/python" ]]; then
  echo "Generic controller environment not found: ${CONTROLLER_PREFIX}" >&2
  echo "Run: ${SCRIPT_DIR}/install.sh --sim" >&2
  exit 2
fi
CONFIG_V2="${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}/controller_v2.yml"
CONFIG_V23="${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}/controller_v23.yml"
if [[ "${MODE}" == "v23" && ! -f "${CONFIG_V23}" ]]; then
  echo "Reconstructed controller YAML not found: ${CONFIG_V23}" >&2
  exit 2
elif [[ "${MODE}" != "v23" && "${MODE}" != "--check" && ! -f "${CONFIG_V2}" ]]; then
  echo "Original controller YAML not found: ${CONFIG_V2}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${HC_IO_ROOT}/install/setup.bash" ]]; then
  source "${HC_IO_ROOT}/install/setup.bash"
fi
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

case "${MODE}" in
  v23)
    exec "${CONTROLLER_PREFIX}/bin/python" -u \
      "${V23_DIR}/script/control_v2_3_ros2.py" \
      "${CONFIG_V23}"
    ;;
  sol_q)
    exec "${CONTROLLER_PREFIX}/bin/python" -u \
      "${SCRIPT_DIR}/generic_sol_q_node.py" \
      --config "${CONFIG_V2}"
    ;;
  sol_q_original) ENTRYPOINT=control_sol_q.py ;;
  pid) ENTRYPOINT=control_pid.py ;;
  *)
    echo "Usage: $0 [--check|v23|sol_q|sol_q_original|pid]" >&2
    exit 2
    ;;
esac

cd "${CONTROL_DIR}/script"
exec "${CONTROLLER_PREFIX}/bin/python" -u "${ENTRYPOINT}" \
  "${ROBOT_CONFIG_ROOT}" "${ROBOT_NAME}"
