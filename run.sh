#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Historical middleware-only entry point. Keep it as a thin compatibility
# wrapper so existing deployments continue to work after the component split.
exec "${SCRIPT_DIR}/middleware/start.sh" "$@"
