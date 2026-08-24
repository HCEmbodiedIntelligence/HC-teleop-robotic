#!/usr/bin/env bash
# Backward-compatible alias. The common stack is not specific to real robots.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${PROJECT_ROOT}/start_teleop.sh" "$@"
