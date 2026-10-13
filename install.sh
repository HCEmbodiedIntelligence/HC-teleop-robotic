#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
DEPS_DIR="${SCRIPT_DIR}/.deps"
mkdir -p "${DEPS_DIR}"

install_controller_environment() {
  local controller_prefix="${HC_CONTROLLER_PREFIX:-${HOME}/miniconda3/envs/hc-teleop-controller}"
  local conda_bin="${CONDA_BIN:-${HOME}/miniconda3/bin/conda}"
  if [[ ! -x "${controller_prefix}/bin/python" ]]; then
    if [[ ! -x "${conda_bin}" ]]; then
      echo "Conda not found: ${conda_bin}" >&2
      echo "The Pinocchio controller is required by both teleop and sim modes." >&2
      exit 2
    fi
    "${conda_bin}" create -y -p "${controller_prefix}" -c conda-forge \
      python=3.10 pinocchio=3.7.0 casadi=3.7.0 numpy=2.2 scipy=1.15 pyyaml
  fi
  HC_CONTROLLER_PREFIX="${controller_prefix}" \
    bash "${SCRIPT_DIR}/run_generic_controller.sh" --check
}

if [[ "${1:-}" == "--camera" ]]; then
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements-camera.txt"
elif [[ "${1:-}" == "--sim" ]]; then
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements-sim.txt"
  install_controller_environment
else
  "${PYTHON_BIN}" -m pip install --upgrade --target "${DEPS_DIR}" -r "${SCRIPT_DIR}/requirements.txt"
  install_controller_environment
fi

echo "Installed project-local dependencies. Start with: ${SCRIPT_DIR}/run.sh"
