#!/usr/bin/env bash
# PyBullet backend only. Middleware, IK, and teleoperation are started by
# start_teleop.sh and remain independent from the selected robot backend.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${PROJECT_ROOT}/run_sim_teleop.sh" --sim-only "$@"
