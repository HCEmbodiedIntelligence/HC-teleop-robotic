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
if [[ -d "${PROJECT_ROOT}/.git" ]]; then
  git -C "${PROJECT_ROOT}" submodule update --init -- src/humanoid_motion_interfaces src/humanoid_motion_server
fi
SOURCE="${PROJECT_ROOT}/src/humanoid_motion_server"
for PATCH in "${PROJECT_ROOT}"/patches/humanoid_motion_server/*.patch; do
  if git -C "${SOURCE}" apply --check "${PATCH}" >/dev/null 2>&1; then
    git -C "${SOURCE}" apply "${PATCH}"
  elif ! git -C "${SOURCE}" apply --reverse --check "${PATCH}" >/dev/null 2>&1; then
    echo "Motion compatibility patch does not match this submodule: ${PATCH}" >&2; exit 2
  fi
done
if [[ "${MODE}" == deps ]]; then
  # Upstream bundled full-stack launches are optional, not built in this branch.
  rosdep install --from-paths "${PROJECT_ROOT}/src/humanoid_motion_interfaces" "${SOURCE}" \
    --ignore-src --skip-keys 'humanoid_driver_runtime teleop_vr_recv' -r -y
  exit
fi
export HUMANOID_MOTION_SDK_DEPS_PREFIX="${HUMANOID_MOTION_SDK_DEPS_PREFIX:-${PROJECT_ROOT}/.deps/robo_manip}"
[[ -d "${HUMANOID_MOTION_SDK_DEPS_PREFIX}" ]] || {
  echo 'Set HUMANOID_MOTION_SDK_DEPS_PREFIX to your qualified RoboManip SDK dependencies.' >&2; exit 2;
}
export CMAKE_PREFIX_PATH="${HUMANOID_MOTION_SDK_DEPS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${HUMANOID_MOTION_SDK_DEPS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
if [[ "${MODE}" == build ]]; then
  colcon --log-base "${PROJECT_ROOT}/log_motion" build \
    --base-paths "${PROJECT_ROOT}/src" --packages-select "${PACKAGES[@]}" \
    --build-base "${PROJECT_ROOT}/build_motion" --install-base "${PROJECT_ROOT}/install_motion"
else
  [[ -f "${PROJECT_ROOT}/install_motion/setup.bash" ]] || { echo 'Run ./bootstrap_colcon.sh build first.' >&2; exit 2; }
  set +u
  source "${PROJECT_ROOT}/install_motion/setup.bash"
  set -u
  colcon --log-base "${PROJECT_ROOT}/log_motion" test \
    --base-paths "${PROJECT_ROOT}/src" --packages-select "${PACKAGES[@]}" \
    --build-base "${PROJECT_ROOT}/build_motion" --install-base "${PROJECT_ROOT}/install_motion"
  colcon test-result --test-result-base "${PROJECT_ROOT}/build_motion" --verbose
fi
