#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/home/maple/.nvm/versions/node/v20.20.2/bin:/usr/bin:${PATH}"

PROFILE="${1:-openarmx}"
FORCE=false
if [[ "${PROFILE}" == "--force" || "${PROFILE}" == "-f" ]]; then
  FORCE=true
  PROFILE="${2:-openarmx}"
fi

if [[ ! -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  echo "HC workspace is not built; run ./bootstrap_colcon.sh build first" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -u

ACTIVE_DOMAIN_FILE="${SCRIPT_DIR}/runtime/active_ros_domain"
if [[ -n "${HC_ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID="${HC_ROS_DOMAIN_ID}"
elif [[ -s "${ACTIVE_DOMAIN_FILE}" ]]; then
  read -r ROS_DOMAIN_ID < "${ACTIVE_DOMAIN_FILE}"
  export ROS_DOMAIN_ID
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
fi
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

FASTDDS_CFG="${SCRIPT_DIR}/config/fastdds_udp.xml"
if [[ -f "${FASTDDS_CFG}" && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_CFG}"
fi

if [[ ! "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] || (( ROS_DOMAIN_ID > 232 )); then
  echo "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'" >&2
  exit 2
fi

RVIZ_CFG="${SCRIPT_DIR}/install/hc_robot_${PROFILE}/share/hc_robot_${PROFILE}/config/teleop.rviz"
if [[ ! -f "${RVIZ_CFG}" ]]; then
  RVIZ_CFG="${SCRIPT_DIR}/src/hc_robot_${PROFILE}/config/teleop.rviz"
fi

if [[ ! -f "${RVIZ_CFG}" ]]; then
  echo "RViz configuration not found for profile '${PROFILE}' at: ${RVIZ_CFG}" >&2
  exit 1
fi

DESCRIPTION_TOPIC="/robot_description"
ALT_DESCRIPTION_TOPIC="/robots/${PROFILE}/robot_description"
JOINT_STATE_TOPIC="/hc_teleop/joint_states"
ALT_JOINT_STATE_TOPIC="/robots/${PROFILE}/state/joints"
READY_TIMEOUT_SECONDS="${HC_RVIZ_READY_TIMEOUT_SECONDS:-6}"

if [[ "${FORCE}" != true ]]; then
  echo "Waiting for ${PROFILE} visualization data on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
  description_ready=false
  for ((attempt = 0; attempt < READY_TIMEOUT_SECONDS; attempt++)); do
    for topic in "${DESCRIPTION_TOPIC}" "${ALT_DESCRIPTION_TOPIC}"; do
      topic_info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
      if [[ "${topic_info}" =~ Publisher\ count:\ [1-9][0-9]* ]]; then
        description_ready=true
        DESCRIPTION_TOPIC="${topic}"
        break 2
      fi
    done
    sleep 1
  done

  if [[ "${description_ready}" != true ]]; then
    echo "No publisher found for robot_description on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." >&2
    echo "Start the stack first with: ./run.sh profile:=${PROFILE} mode:=sim" >&2
    echo "To launch RViz anyway without waiting, use: ./rviz.sh --force ${PROFILE}" >&2
    exit 3
  fi

  joint_ready=false
  for topic in "${JOINT_STATE_TOPIC}" "${ALT_JOINT_STATE_TOPIC}"; do
    if timeout 2 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      joint_ready=true
      JOINT_STATE_TOPIC="${topic}"
      break
    fi
  done

  if [[ "${joint_ready}" != true ]]; then
    echo "Notice: Waiting for joint state on ${JOINT_STATE_TOPIC}... (RViz will launch anyway)" >&2
  fi
fi

echo "Launching RViz with profile=${PROFILE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
exec ros2 run rviz2 rviz2 -d "${RVIZ_CFG}"
