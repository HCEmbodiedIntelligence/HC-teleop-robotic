#!/usr/bin/env bash
# Explicit external hardware boundary entrypoint. Never invoked by middleware.
# The existing vendor bringup remains separately owned by the hardware project.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash tools/run_robot_driver.sh [--profile DIRECTORY] [--fake]'
  echo 'Consumes profile connection parameters. --fake selects the OpenArmX fake health contract.'
  exit 0
fi
if [[ "${1:-}" == --profile ]]; then
  PROFILE="$(cd -- "$2" && pwd)"; shift 2
else
  PROFILE="$(/usr/bin/python3 "${PROJECT_ROOT}/middleware/profile_cli.py" profile-dir)"
fi
RUNTIME_DIR="${HC_ROBOT_API_DRIVER_RUNTIME_DIR:-$(mktemp -d /tmp/hc-robot-driver.XXXXXX)}"
/usr/bin/python3 "${PROJECT_ROOT}/middleware/profile_cli.py" generate --profile "${PROFILE}" --output "${RUNTIME_DIR}" >&2
DRIVER_KIND="$(/usr/bin/python3 -c 'import sys,yaml; print(next(iter(yaml.safe_load(open(sys.argv[1])))))' "${RUNTIME_DIR}/driver.yaml")"
ARGS=(--ros-args --params-file "${RUNTIME_DIR}/driver.yaml")
if [[ "${1:-}" == --fake ]]; then
  if [[ "$DRIVER_KIND" == openarmx_hc_robot_adapter ]]; then ARGS+=(-p hardware_mode:=fake); fi
  shift
fi
[[ $# == 0 ]] || { echo 'Unrecognized driver argument' >&2; exit 2; }
case "$DRIVER_KIND" in
  openarmx_hc_robot_adapter) exec ros2 run openarmx_hc_robot_adapter openarmx_hc_robot_adapter "${ARGS[@]}" ;;
  humanoid_driver_runtime) exec ros2 run humanoid_driver_runtime humanoid_driver_runtime_node "${ARGS[@]}" ;;
  *) echo "Unsupported profile driver: $DRIVER_KIND" >&2; exit 2 ;;
esac
