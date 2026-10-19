#!/usr/bin/env bash
set -euo pipefail
robot_ws="${1:?Pass D/robot workspace}"
cd "$robot_ws"
mkdir -p artifacts/g0/dist
.venv/bin/python -m pip wheel --no-deps --no-build-isolation --no-cache-dir \
  src/humanoid_manager/upload_client --wheel-dir artifacts/g0/dist
uv venv --python /usr/bin/python3 install/uploader --allow-existing
uv pip install --python install/uploader/bin/python --no-cache --no-index \
  --find-links artifacts/g0/wheelhouse --find-links artifacts/g0/dist \
  --reinstall-package humanoid-upload-client humanoid-upload-client==1.0.0
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH=src/humanoid_manager/upload_client
.venv/bin/python -m pytest src/humanoid_manager/upload_client/tests -q \
  --basetemp=artifacts/uploader-pytest --junitxml=artifacts/g0/uploader-tests.xml
