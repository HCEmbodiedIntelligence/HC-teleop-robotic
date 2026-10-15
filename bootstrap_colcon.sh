#!/usr/bin/env bash
# Optional motion workspace; the main middleware/install.sh remain unchanged.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-build}"
if [[ "${MODE}" == --help || "${MODE}" == -h ]]; then
  echo 'Usage: ./bootstrap_colcon.sh [deps|build|test]'; exit 0
fi
[[ "${MODE}" == deps || "${MODE}" == build || "${MODE}" == test ]] || { echo "Unknown mode: ${MODE}" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS 2 Humble is required.' >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
set -u
export PATH="/usr/bin:${PATH}"
export COLCON_DEFAULTS_FILE="${PROJECT_ROOT}/colcon_defaults.yaml"
PACKAGES=(humanoid_motion_interfaces humanoid_motion_server)
SOURCE="${PROJECT_ROOT}/src/humanoid_motion_server"
if [[ ! -f "${SOURCE}/package.xml" || ! -f "${PROJECT_ROOT}/src/humanoid_motion_interfaces/package.xml" ]]; then
  git -C "${PROJECT_ROOT}" submodule update --init -- src/humanoid_motion_interfaces src/humanoid_motion_server
fi
# The unified source owns its pinned SDK dependencies. Do not apply patches for
# the older HC snapshot to a different upstream tree or mutate it during builds.
MOTION_BUILD="${HC_MOTION_BUILD:-${PROJECT_ROOT}/build_motion}"
MOTION_INSTALL="${HC_MOTION_INSTALL:-${PROJECT_ROOT}/install_motion}"
MOTION_LOG="${HC_MOTION_LOG:-${PROJECT_ROOT}/log_motion}"
if [[ "${MODE}" == deps ]]; then
  # Upstream bundled full-stack launches are optional, not built in this branch.
  rosdep install --from-paths "${PROJECT_ROOT}/src/humanoid_motion_interfaces" "${SOURCE}" \
    --ignore-src --skip-keys 'humanoid_driver_runtime teleop_vr_recv' -r -y
  exit
fi
if [[ "${MODE}" == build ]]; then
  colcon --log-base "${MOTION_LOG}" build \
    --base-paths "${SOURCE}" "${PROJECT_ROOT}/src/humanoid_motion_interfaces" --packages-select "${PACKAGES[@]}" \
    --build-base "${MOTION_BUILD}" --install-base "${MOTION_INSTALL}"
else
  [[ -f "${MOTION_INSTALL}/setup.bash" ]] || { echo 'Run ./bootstrap_colcon.sh build first.' >&2; exit 2; }
  set +u
  source "${MOTION_INSTALL}/setup.bash"
  set -u
  colcon --log-base "${MOTION_LOG}" test \
    --base-paths "${SOURCE}" "${PROJECT_ROOT}/src/humanoid_motion_interfaces" --packages-select "${PACKAGES[@]}" \
    --build-base "${MOTION_BUILD}" --install-base "${MOTION_INSTALL}"
  colcon test-result --test-result-base "${MOTION_BUILD}" --verbose
fi
