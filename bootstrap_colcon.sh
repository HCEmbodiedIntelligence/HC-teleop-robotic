#!/usr/bin/env bash
# Build only the decomposed native ROS 2 packages under src/.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MOTION_SERVER_SOURCE="${PROJECT_ROOT}/src/humanoid_motion_server"
MOTION_INTERFACES_SOURCE="${PROJECT_ROOT}/src/humanoid_motion_interfaces"
MOTION_SERVER_PATCH="${PROJECT_ROOT}/patches/humanoid_motion_server/humble-hpp-fcl-2.4.5.patch"
DRIVER_RUNTIME_SOURCE="${PROJECT_ROOT}/src/humanoid_driver_runtime"
DRIVER_RUNTIME_PATCH="${PROJECT_ROOT}/patches/humanoid_driver_runtime/class_libraries.patch"

export PATH="/home/maple/.nvm/versions/node/v20.20.2/bin:/usr/bin:${PATH}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

prepare_motion_server_source() {
  if [[ -d "${PROJECT_ROOT}/.git" ]]; then
    git -C "${PROJECT_ROOT}" submodule update --init --recursive
  fi

  if [[ ! -f "${MOTION_SERVER_SOURCE}/package.xml" ||
        ! -f "${MOTION_INTERFACES_SOURCE}/package.xml" ]]; then
    echo "Motion Server submodules are missing; run git submodule update --init --recursive" >&2
    exit 2
  fi

  # The pinned upstream commit requires hpp-fcl 2.4.4, while the qualified
  # ROS 2 Humble installation provides 2.4.5. Keep this small compatibility
  # delta in the parent workspace until it is merged upstream.
  if git -C "${MOTION_SERVER_SOURCE}" apply --check "${MOTION_SERVER_PATCH}" >/dev/null 2>&1; then
    git -C "${MOTION_SERVER_SOURCE}" apply "${MOTION_SERVER_PATCH}"
  elif ! git -C "${MOTION_SERVER_SOURCE}" apply --reverse --check \
      "${MOTION_SERVER_PATCH}" >/dev/null 2>&1; then
    echo "Motion Server compatibility patch does not match the pinned commit" >&2
    exit 2
  fi

  if [[ -f "${DRIVER_RUNTIME_PATCH}" && -d "${DRIVER_RUNTIME_SOURCE}" ]]; then
    if git -C "${DRIVER_RUNTIME_SOURCE}" apply --check "${DRIVER_RUNTIME_PATCH}" >/dev/null 2>&1; then
      git -C "${DRIVER_RUNTIME_SOURCE}" apply "${DRIVER_RUNTIME_PATCH}"
    fi
  fi

  if [[ -z "${HUMANOID_MOTION_SDK_DEPS_PREFIX:-}" ]]; then
    local project_sdk_deps="${PROJECT_ROOT}/.deps/robo_manip"
    if [[ ! -d "${project_sdk_deps}" && -d "/home/maple/test/humanoid/.sdk_deps" ]]; then
      mkdir -p "${PROJECT_ROOT}/.deps"
      ln -sfn "/home/maple/test/humanoid/.sdk_deps" "${project_sdk_deps}"
    fi
    if [[ -d "${project_sdk_deps}" ]]; then
      export HUMANOID_MOTION_SDK_DEPS_PREFIX="${project_sdk_deps}"
    else
      echo "RoboManip dependencies not found at ${project_sdk_deps}." >&2
      echo "Set HUMANOID_MOTION_SDK_DEPS_PREFIX to the prefix containing ruckig 0.17.3 and related SDK dependencies." >&2
      exit 2
    fi
  fi
}

prepare_motion_server_source

export COLCON_DEFAULTS_FILE="${PROJECT_ROOT}/colcon_defaults.yaml"

case "${1:-build}" in
  deps)
    rosdep install --from-paths "${PROJECT_ROOT}/src" --ignore-src -r -y
    ;;
  build)
    colcon --log-base "${PROJECT_ROOT}/log" build --base-paths "${PROJECT_ROOT}/src" \
      --build-base "${PROJECT_ROOT}/build" \
      --install-base "${PROJECT_ROOT}/install"
    ;;
  test)
    if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
      set +u
      source "${PROJECT_ROOT}/install/setup.bash"
      set -u
    fi
    colcon --log-base "${PROJECT_ROOT}/log" test --base-paths "${PROJECT_ROOT}/src" \
      --build-base "${PROJECT_ROOT}/build" \
      --install-base "${PROJECT_ROOT}/install"
    colcon test-result --test-result-base "${PROJECT_ROOT}/build" --verbose
    ;;
  *)
    echo "usage: $0 [deps|build|test]" >&2
    exit 2
    ;;
esac
