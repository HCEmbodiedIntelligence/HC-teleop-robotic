#!/usr/bin/env bash
set -euo pipefail
HC_PLATFORM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HC_PLATFORM_ROOT/platform/scripts/install.sh" "$@"
