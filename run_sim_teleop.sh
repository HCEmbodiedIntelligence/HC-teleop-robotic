#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HC_IO_ROOT="${HC_IO_ROOT:-/home/maple/hc_io_suit}"
MIDDLEWARE_CONFIG="${HC_MIDDLEWARE_CONFIG:-${SCRIPT_DIR}/middleware.yaml}"
ROBOT_CONFIG_ROOT="${HC_ROBOT_CONFIG_ROOT:-$(/usr/bin/python3 "${SCRIPT_DIR}/robot_profile_cli.py" root --config "${MIDDLEWARE_CONFIG}")}"
ROBOT_NAME="${HC_ROBOT_NAME:-$(/usr/bin/python3 "${SCRIPT_DIR}/robot_profile_cli.py" active --config "${MIDDLEWARE_CONFIG}")}"
PROFILE_DIR="${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}"

if [[ ! -d "${SCRIPT_DIR}/.deps/pybullet-3.2.6.dist-info" ]]; then
  echo "Simulation dependencies not found. Run ${SCRIPT_DIR}/install.sh --sim first." >&2
  exit 2
fi
if [[ ! -f "${HC_IO_ROOT}/src/scripts/general_sim_robot_control_node_ros2.py" ]]; then
  echo "HC IO Suit not found at ${HC_IO_ROOT}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${HC_IO_ROOT}/install/setup.bash" ]]; then
  source "${HC_IO_ROOT}/install/setup.bash"
fi
set -u
export PYTHONPATH="${SCRIPT_DIR}/.deps${PYTHONPATH:+:${PYTHONPATH}}"

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  DOMAIN_ID="$(/usr/bin/python3 -c "import yaml; cfg=yaml.safe_load(open('${MIDDLEWARE_CONFIG}')) or {}; print(cfg.get('ros',{}).get('domain_id', 0))" 2>/dev/null || echo 0)"
  export ROS_DOMAIN_ID="${DOMAIN_ID}"
fi

SIM_ENTRY="${HC_IO_ROOT}/src/scripts/general_sim_robot_control_node_ros2.py"
SIM_ARGS=(--robot_name "${ROBOT_NAME}")
TELEOP_CONFIG="${HC_ARM_TELEOP_CONFIG:-}"
if [[ -f "${PROFILE_DIR}/vr_configs.yml" ]]; then
  SIM_ENTRY="${SCRIPT_DIR}/profile_sim_robot_control_node.py"
  SIM_ARGS=(--robot_name "${PROFILE_DIR}")
elif [[ ! -f "${HC_IO_ROOT}/src/io_teleop_robot_descriptions/${ROBOT_NAME}/vr_configs.yml" ]]; then
  echo "Active robot profile has no simulation config: ${ROBOT_NAME}" >&2
  echo "Import an HC robot YAML profile or set HC_ROBOT_NAME explicitly." >&2
  exit 2
fi
if [[ -z "${TELEOP_CONFIG}" ]]; then
  if [[ -f "${PROFILE_DIR}/arm_teleop.yaml" ]]; then
    TELEOP_CONFIG="${PROFILE_DIR}/arm_teleop.yaml"
  elif [[ -f "${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}/arm_teleop.yaml" ]]; then
    TELEOP_CONFIG="${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}/arm_teleop.yaml"
  elif [[ -f "${SCRIPT_DIR}/arm_teleop.yaml" ]]; then
    TELEOP_CONFIG="${SCRIPT_DIR}/arm_teleop.yaml"
  else
    echo "Active robot profile has no arm_teleop.yaml: ${ROBOT_NAME}" >&2
    exit 2
  fi
fi
export HC_IO_ROOT HC_ROBOT_CONFIG_ROOT="${ROBOT_CONFIG_ROOT}" HC_ROBOT_NAME="${ROBOT_NAME}"
# The external simulator still declares its historical topic names internally.
# ROS remapping keeps the public graph entirely under the HC namespace.
SIM_ROS_ARGS=(
  --ros-args
  -r /io_teleop/joint_states:=/hc_teleop/joint_states
  -r /io_teleop/joint_cmd:=/hc_teleop/joint_cmd
  -r /io_teleop/target_joint_from_vr:=/hc_teleop/target_joint_from_vr
  -r /io_teleop/target_finger_joints:=/hc_teleop/target_finger_joints
  -r /io_teleop/target_ee_poses:=/hc_teleop/target_ee_poses
  -r /io_teleop/target_gripper_status:=/hc_teleop/target_gripper_status
  -r /io_teleop/target_base_move:=/hc_teleop/target_base_move
)
BACKEND="${HC_TELEOP_BACKEND:-v23}"
for argument in "$@"; do
  case "${argument}" in
    --headless) SIM_ARGS+=(--headless) ;;
    --v23|--reconstructed) BACKEND=v23 ;;
    --legacy) BACKEND=legacy ;;
    --generic) BACKEND=generic ;;
    *)
      echo "Usage: $0 [--headless] [--v23|--generic|--legacy]" >&2
      exit 2
      ;;
  esac
done

if [[ "${BACKEND}" == "generic" || "${BACKEND}" == "v23" ]]; then
  "${SCRIPT_DIR}/run_generic_controller.sh" --check
fi

SIM_PID=""
TELEOP_PID=""
SOL_Q_PID=""
PID_PID=""
DIAGNOSTICS_PID=""
cleanup() {
  trap - INT TERM EXIT
  [[ -z "${DIAGNOSTICS_PID}" ]] || kill -INT "${DIAGNOSTICS_PID}" 2>/dev/null || true
  [[ -z "${TELEOP_PID}" ]] || kill -INT "${TELEOP_PID}" 2>/dev/null || true
  [[ -z "${PID_PID}" ]] || kill -INT "${PID_PID}" 2>/dev/null || true
  [[ -z "${SOL_Q_PID}" ]] || kill -INT "${SOL_Q_PID}" 2>/dev/null || true
  [[ -z "${SIM_PID}" ]] || kill -INT "${SIM_PID}" 2>/dev/null || true
  [[ -z "${DIAGNOSTICS_PID}" ]] || wait "${DIAGNOSTICS_PID}" 2>/dev/null || true
  [[ -z "${TELEOP_PID}" ]] || wait "${TELEOP_PID}" 2>/dev/null || true
  [[ -z "${PID_PID}" ]] || wait "${PID_PID}" 2>/dev/null || true
  [[ -z "${SOL_Q_PID}" ]] || wait "${SOL_Q_PID}" 2>/dev/null || true
  [[ -z "${SIM_PID}" ]] || wait "${SIM_PID}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

/usr/bin/python3 "${SIM_ENTRY}" "${SIM_ARGS[@]}" "${SIM_ROS_ARGS[@]}" &
SIM_PID=$!
/usr/bin/python3 "${SCRIPT_DIR}/teleop_arm_controller.py" \
  --config "${TELEOP_CONFIG}" --backend "${BACKEND}" &
TELEOP_PID=$!
if [[ "${BACKEND}" == "generic" || "${BACKEND}" == "v23" ]]; then
  # Start IK immediately. Both external backends safely hold until joint
  # feedback and all targets arrive; blocking here used to create a startup
  # deadlock where the marker moved but no controller produced joint commands.
  if [[ "${BACKEND}" == "v23" ]]; then
    "${SCRIPT_DIR}/run_generic_controller.sh" v23 &
    SOL_Q_PID=$!
  else
    "${SCRIPT_DIR}/run_generic_controller.sh" sol_q &
    SOL_Q_PID=$!
    "${SCRIPT_DIR}/run_generic_controller.sh" pid &
    PID_PID=$!
  fi
fi

LOG_PATH=""
if [[ "${TELEOP_DIAGNOSTICS:-1}" != "0" ]]; then
  mkdir -p "${SCRIPT_DIR}/runtime/teleop_logs"
  LOG_PATH="${TELEOP_LOG_PATH:-${SCRIPT_DIR}/runtime/teleop_logs/teleop_$(date +%Y%m%d_%H%M%S).csv}"
  /usr/bin/python3 "${SCRIPT_DIR}/teleop_diagnostics.py" \
    --output "${LOG_PATH}" \
    --rate "${TELEOP_LOG_RATE:-30}" &
  DIAGNOSTICS_PID=$!
fi

echo "Simulation PID=${SIM_PID}; robot=${ROBOT_NAME}; VR adapter PID=${TELEOP_PID}; backend=${BACKEND}; ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
if [[ "${BACKEND}" == "generic" ]]; then
  echo "Original generic controller: sol_q PID=${SOL_Q_PID}; PID smoother=${PID_PID}"
elif [[ "${BACKEND}" == "v23" ]]; then
  echo "Reconstructed controller_v2_3 PID=${SOL_Q_PID}"
fi
if [[ -n "${DIAGNOSTICS_PID}" ]]; then
  echo "Diagnostics PID=${DIAGNOSTICS_PID}; log=${LOG_PATH}"
fi
echo "Left Grip: base+waist | Right Grip: both arms+grippers | Ctrl+C: stop"
WAIT_PIDS=("${SIM_PID}" "${TELEOP_PID}")
[[ -z "${SOL_Q_PID}" ]] || WAIT_PIDS+=("${SOL_Q_PID}")
[[ -z "${PID_PID}" ]] || WAIT_PIDS+=("${PID_PID}")
wait -n "${WAIT_PIDS[@]}"
