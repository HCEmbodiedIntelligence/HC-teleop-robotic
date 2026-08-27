#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
DEPS_DIR="${SCRIPT_DIR}/.deps"
mkdir -p "${DEPS_DIR}"

if [[ "${1:-}" == "--camera" ]]; then
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements-camera.txt"
elif [[ "${1:-}" == "--sim" ]]; then
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements-sim.txt"
  CONTROLLER_PREFIX="${HC_CONTROLLER_PREFIX:-${HOME}/miniconda3/envs/hc-teleop-controller}"
  CONDA_BIN="${CONDA_BIN:-${HOME}/miniconda3/bin/conda}"
  if [[ ! -x "${CONTROLLER_PREFIX}/bin/python" ]]; then
    if [[ ! -x "${CONDA_BIN}" ]]; then
      echo "Conda not found: ${CONDA_BIN}" >&2
      exit 2
    fi
    "${CONDA_BIN}" create -y -p "${CONTROLLER_PREFIX}" -c conda-forge \
      python=3.10 pinocchio=3.7.0 casadi=3.7.0 numpy=2.2 scipy=1.15 pyyaml pip
  elif ! "${CONTROLLER_PREFIX}/bin/python" -m pip --version >/dev/null 2>&1; then
    "${CONDA_BIN}" install -y -p "${CONTROLLER_PREFIX}" -c conda-forge pip
  fi
  "${CONTROLLER_PREFIX}/bin/python" -m pip install --no-deps \
    "${SCRIPT_DIR}/adapters/v23"
  HC_CONTROLLER_PREFIX="${CONTROLLER_PREFIX}" \
    bash "${SCRIPT_DIR}/run_generic_controller.sh" --check
else
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements.txt"
fi

echo "Installed project-local dependencies. Start with: ${SCRIPT_DIR}/start_teleop.sh"
