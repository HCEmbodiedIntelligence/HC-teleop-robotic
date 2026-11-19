#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
DEPS_DIR="${SCRIPT_DIR}/.deps"
mkdir -p "${DEPS_DIR}"

install_upload_client() {
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" "${SCRIPT_DIR}/upload_client"
}

if [[ "${1:-}" == "--upload" ]]; then
  install_upload_client
elif [[ "${1:-}" == "--camera" ]]; then
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements-camera.txt"
elif [[ "${1:-}" == "--sim" ]]; then
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements-sim.txt"
  install_upload_client
else
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements.txt"
  install_upload_client
fi

echo "Installed project-local dependencies. Start with: ${SCRIPT_DIR}/run.sh"
