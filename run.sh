#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -d "${SCRIPT_DIR}/.deps/aiohttp" ]] && ! /usr/bin/python3 -c "import aiohttp" 2>/dev/null; then
  echo "Dependencies not found. Run ${SCRIPT_DIR}/install.sh first." >&2
  exit 2
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

export PYTHONPATH="${SCRIPT_DIR}/.deps${PYTHONPATH:+:${PYTHONPATH}}"
exec /usr/bin/python3 "${SCRIPT_DIR}/middleware_server.py" "$@"
