# HC upload integration

This copy comes from humanoid_manager/upload_client at 26cbe8b7b751276cbcbc2e21d1ae2d1e627a7495. It retains the humanoid_upload package, schemas and raw protocol. Install only HC/upload_client for the new deployment; do not install two copies into one environment.

Install `python -m pip install ./upload_client`; the middleware also requires its existing aiohttp dependency. The uploader itself has no ROS, training-stack, manager or platform-source imports.

`core.upload_service.UploadService(config, *, artifact_state_dir, data_directory, request_raw_seal=None)` has synchronous start/stop, enqueue(artifact_id, binding_id=None), binding_snapshot(), seal_raw(recording_id, binding_id=None), artifact_usage() and cancel_artifact(). Run lifecycle and request methods with asyncio.to_thread. stop() returns `stopped:false` if a network request is still exiting; it never releases that request's file lease early.

Register `core.upload_api.register_upload_routes(app, service)`. GET /api/uploads returns status/jobs. POST /api/uploads/{binding,enqueue,seal-raw,retry,cancel,scan} uses JSON. /api/recordings/upload is unchanged (local file import). Serve the existing static directory, load upload_panel.js/css and call window.mountUploadPanel(container); the return value disposes its refresh timer.

Config uses existing humanoid_upload.common.DEFAULTS plus enabled, queue_directory, platform_base_url, credential_ref, robot_id, local_robot_id and task_id. Disable uploads by default. Resolve paths against the middleware configuration location before construction. Queue storage must be outside the recording data root. credential_ref accepts env:NAME or file:/absolute/path; secret files must be owned by the service user and mode 0600. Credentials never enter ready descriptors or public queue responses.

Bindings persist in queue_directory/bindings.json (0600). A page binding change creates/selects an immutable binding ID. RecordingService takes binding_snapshot() at capture creation when auto enqueue is enabled, then includes its binding_id in the descriptor. Unbound historical artifacts are never assigned to the new default by a scan; manual enqueue explicitly selects them. Raw awaiting binding uses request_raw_seal; the uploader does not seal raw files itself.

The ready descriptor is internal hc-artifact-ready/v1. Ordinary HC requires capture_started_at/capture_ended_at from actual capture lifecycle. completed_at is never substituted. Both modes have separate artifact IDs and one recording ID. The uploader checks files at enqueue and again while holding a shared artifact lease in the worker. It emits MCAP or LEROBOT_V3 manifests; these require platform acceptance testing. Raw still validates openarm-recording-complete/v2 plus assets and uses CAPTURE_BUNDLE, never a forged raw marker.

Deletion must use `with ArtifactLocks(artifact_state_dir).delete(artifact_id): ...` for the entire filesystem mutation. The exclusive lock writes a persistent tombstone before yielding. ARTIFACT_BUSY means retry after the reader exits; cancel intent alone does not permit deletion. A failed delete retains the tombstone. Do not unlink lock files. Multiple identities referring to the same physical files must all be coordinated by the recording/file owner. Unsupported out-of-band filesystem edits are detected by hashes but must not replace the shared lease protocol.

Independent process (optional, do not also run the embedded worker):
```
PYTHONPATH=middleware python -m core.upload_service --config /absolute/hc-upload-service.json
```
The JSON wrapper contains uploads (config object), artifact_state_dir, data_directory. It binds 127.0.0.1:8091 by default. This process has no recorder callback: seal-raw accurately returns RAW_SEAL_UNAVAILABLE. The embedded service supports the callback. Existing humanoid-upload CLI remains for legacy sealed bundles; use the HC service for ready descriptors.

Upload submitted, platform processing, awaiting slice and quality are separate states. Raw AWAITING_SLICE/SLICED never reports quality PASS. A cancelled upload may already have remote parts; retry reconciles using its original client_upload_id, without creating another raw source.

Independent tests: see tests/test_hc_artifacts.py. Synthetic transport fixture tests do not prove MCAP decoding, camera timing, platform processing or hardware behavior. Legacy tests referencing OPENARM_G0 need that separately supplied release; absence is an unexecuted prerequisite, not a passing test.

Persistent page bindings take precedence over startup defaults once bindings.json exists. Changing a startup task ID does not silently rewrite this store or old jobs; select the new default through the binding API. Service scheduling and rate values are validated on every startup, including when restoring bindings. A failed atomic binding write leaves the live default unchanged. Earlier HC queues rebuild their artifact identity index from the existing job payloads, rejecting conflicting identities instead of silently dropping jobs.

Development cross-component tests can use `HC_RECORDING_WORKTREE=/absolute/B` with tests/test_handoff_artifacts.py. They start the selected B RecordingService with ROS disabled, generate an actual synthetic MCAP, and test both deletion entry points while C holds the read lease. These tests are useful before merge, but cannot certify a stable integration commit. Actual B handoff files can separately be supplied via HC_HANDOFF_READY_DIR and HC_HANDOFF_DATA_DIR.
