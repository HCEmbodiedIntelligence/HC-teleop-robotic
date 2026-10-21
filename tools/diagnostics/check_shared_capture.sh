#!/usr/bin/env bash
# Run from the robot repository. Requires Python 3.10+, pytest, aiohttp, jsonschema,
# mcap, numpy, PyYAML, OpenCV and pyarrow. Uses synthetic data and loopback HTTP only.
set -euo pipefail
capture_python="${CAPTURE_TEST_PYTHON:-python3}"
capture_evidence="${1:?Pass a dedicated evidence directory}"
mkdir -p "$capture_evidence/tmp"
export TMPDIR="$capture_evidence/tmp"
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/upload_client:$PWD/middleware:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
"$capture_python" -m pytest -p no:cacheprovider \
  tests/test_shared_capture_plans.py tests/test_platform_task_recording.py \
  tests/test_platform_api_errors.py tests/test_recording_upload_handoff.py \
  tests/test_lerobot_recording.py tests/test_lerobot_export.py \
  tests/test_ros_recording_executor.py tests/test_recording_datasets.py tests/test_main_upload_app.py \
  upload_client/tests/test_hc_artifacts.py upload_client/tests/test_connection.py \
  upload_client/tests/test_recording_bundle.py upload_client/tests/test_integrated_app.py \
  upload_client/tests/test_client.py::test_schema_provenance \
  upload_client/tests/test_client.py::test_crc64_vector \
  --basetemp="$capture_evidence/synthetic-data" --junitxml="$capture_evidence/python.xml" -q -ra \
  2>&1 | tee "$capture_evidence/python.log"
