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
  CONTROLLER_PREFIX="${HC_CONTROLLER_PREFIX:-/home/maple/miniconda3/envs/hc-teleop-controller}"
  CONDA_BIN="${CONDA_BIN:-/home/maple/miniconda3/bin/conda}"
  if [[ ! -x "${CONTROLLER_PREFIX}/bin/python" ]]; then
    if [[ ! -x "${CONDA_BIN}" ]]; then
      echo "Conda not found: ${CONDA_BIN}" >&2
      exit 2
    fi
    "${CONDA_BIN}" create -y -p "${CONTROLLER_PREFIX}" -c conda-forge \
      python=3.10 pinocchio=3.7.0 casadi=3.7.0 numpy=2.2 scipy=1.15 pyyaml
  fi
  HC_CONTROLLER_PREFIX="${CONTROLLER_PREFIX}" \
    bash "${SCRIPT_DIR}/run_generic_controller.sh" --check
else
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements.txt"
fi

echo "Installed project-local dependencies. Start with: ${SCRIPT_DIR}/run.sh"
