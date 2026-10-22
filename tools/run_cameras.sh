#!/usr/bin/env bash
# Physical cameras only; one profile-owned configuration for all three devices.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash tools/run_cameras.sh [--profile DIRECTORY]'
  echo 'Defaults to the selected Dashboard profile; reads its cameras.yaml.'
  exit 0
fi
if [[ "${1:-}" == --profile && $# == 2 ]]; then
  camera_profile="$(cd -- "$2" && pwd)"
elif [[ $# == 0 ]]; then
  camera_profile="$(/usr/bin/python3 "${project_root}/middleware/profile_cli.py" profile-dir)"
else
  echo 'Usage: bash tools/run_cameras.sh [--profile DIRECTORY]' >&2; exit 2
fi
[[ -f "${camera_profile}/cameras.yaml" ]] || { echo "Missing ${camera_profile}/cameras.yaml" >&2; exit 2; }
source "${project_root}/../../setup_hc.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
exec ros2 launch humanoid_camera multi_camera.launch.py camera_config:="${camera_profile}/cameras.yaml"
