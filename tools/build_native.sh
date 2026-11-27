#!/usr/bin/env bash
# Build all C++17 native accelerations for HC-teleop-robotic:
#   1. adapters/v23/cpp/build/controller_v2_3_ros2_cpp (+ _v23_ik_cpp pybind11 module)
#   2. adapters/core/_arm_teleop_math_cpp*.so & _collision_guard_cpp*.so
#   3. middleware/core/media/_rtp_fec_cpp*.so
#   4. middleware/core/recording/_recording_cdr_cpp*.so
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SYS_PYTHON="/usr/bin/python3"
SYS_PY_CONFIG="/usr/bin/python3-config"

# Use clean system PATH so Conda Python never hijacks ROS 2 Humble Python 3.10 ABI.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

if [[ ! -x "${SYS_PYTHON}" || ! -x "${SYS_PY_CONFIG}" ]]; then
  echo "[build_native] Missing system Python 3.10 dev tools (/usr/bin/python3-config)" >&2
  exit 1
fi

PY_INCLUDES="$("${SYS_PY_CONFIG}" --includes)"
EXT_SUFFIX="$("${SYS_PY_CONFIG}" --extension-suffix)"
PYBIND11_INC="/usr/include"

echo "[build_native] (1/4) Building C++17 v23 IK core & rclcpp node..."
(
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cmake -S "${PROJECT_ROOT}/adapters/v23/cpp" \
        -B "${PROJECT_ROOT}/adapters/v23/cpp/build" \
        -DCMAKE_BUILD_TYPE=Release
  cmake --build "${PROJECT_ROOT}/adapters/v23/cpp/build" -j"$(nproc)"
)

echo "[build_native] (2/4) Building C++17 _arm_teleop_math_cpp & _collision_guard_cpp..."
(
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cmake -S "${PROJECT_ROOT}/adapters/core/cpp" \
        -B "${PROJECT_ROOT}/adapters/core/cpp/build" \
        -DCMAKE_BUILD_TYPE=Release
  cmake --build "${PROJECT_ROOT}/adapters/core/cpp/build" -j"$(nproc)"
)

echo "[build_native] (3/4) Building C++17 _rtp_fec_cpp extension..."
# shellcheck disable=SC2086
g++ -O3 -Wall -Wextra -shared -std=c++17 -fPIC \
  ${PY_INCLUDES} -I"${PYBIND11_INC}" \
  "${PROJECT_ROOT}/middleware/core/media/cpp/rtp_fec_cpp.cpp" \
  -o "${PROJECT_ROOT}/middleware/core/media/_rtp_fec_cpp${EXT_SUFFIX}"

echo "[build_native] (4/4) Building C++17 _recording_cdr_cpp extension..."
# shellcheck disable=SC2086
g++ -O3 -Wall -Wextra -shared -std=c++17 -fPIC \
  ${PY_INCLUDES} -I"${PYBIND11_INC}" \
  "${PROJECT_ROOT}/middleware/core/recording/cpp/recording_cdr_cpp.cpp" \
  -o "${PROJECT_ROOT}/middleware/core/recording/_recording_cdr_cpp${EXT_SUFFIX}"

echo "[build_native] Build complete:"
ls -lh \
  "${PROJECT_ROOT}/adapters/v23/cpp/build/controller_v2_3_ros2_cpp" \
  "${PROJECT_ROOT}/adapters/v23/src/_v23_ik_cpp${EXT_SUFFIX}" \
  "${PROJECT_ROOT}/adapters/core/_arm_teleop_math_cpp${EXT_SUFFIX}" \
  "${PROJECT_ROOT}/adapters/core/_collision_guard_cpp${EXT_SUFFIX}" \
  "${PROJECT_ROOT}/middleware/core/media/_rtp_fec_cpp${EXT_SUFFIX}" \
  "${PROJECT_ROOT}/middleware/core/recording/_recording_cdr_cpp${EXT_SUFFIX}"

