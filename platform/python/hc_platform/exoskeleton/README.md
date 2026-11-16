# Optional ROS exoskeleton gateway

The supported HTTP integration remains `POST /api/control/command` with
`source: exo` and current core tokens. This package additionally accepts the
existing ROS 2 `sensor_msgs/msg/JointState` input topic in a separate process.

```bash
python -m hc_platform.exoskeleton --config /private/runtime/config.json \
  --api-url http://127.0.0.1:8000 --state-dir /private/runtime
```

The compiled configuration must explicitly set `exoskeleton.enabled: true`.
The default topic is `/control/exoskeleton/joint_states`. ROS dependencies are
imported only in this optional process. It uses the runtime's isolated ROS domain
and `HC_PLATFORM_TOKEN` for the local HTTP endpoint when authentication is enabled.
No device driver or external exoskeleton calibration procedure is implemented here.

A ROS executor thread validates and normalizes each sample into one bounded latest
snapshot. The asyncio loop reads `/api/control/state` and forwards only a new
sample after an explicitly selected `exo` epoch. It uses the existing checked HTTP
command route, including configured collision validation, instead of bypassing it
with direct hardware or direct core writes. It never calls recover, resume or
session_start. Source switching clears eligibility of cached samples.

A nonzero, increasing ROS Header timestamp is required. Duplicate/out-of-order
stamps, frozen producers, unknown or duplicate mapped names, nonfinite values,
wrong dimensions, out-of-limit values and stale/future samples are rejected.
`sample_mono_ns` is calculated at callback time from the observed ROS sample age;
queueing and HTTP transmission do not renew it. If the source freezes, publishing
stops and the core command lease expires. A producer clock reset requires a gateway
restart or a corrected monotonic ROS time stream; it is never silently accepted as
an old sequence.

Options:

```json
{"exoskeleton":{"enabled":true,"topic":"/control/exoskeleton/joint_states",
 "max_sample_age_ms":100,"future_tolerance_ms":5,"ttl_ms":100,
 "joint_mapping":{"device_right_1":{"name":"Joint1_R","unit":"deg","scale":-1,"offset":0}}}}
```

Omitted mappings use URDF model names and native SI units (`rad` or `m`). `deg` and
`mm` inputs can be explicitly converted; scale and offset apply after conversion
into SI. This configuration is only an interface mapping and does not replace a
physical calibration. Runtime status is written to `exoskeleton.json` without
blocking the ROS receive callback.

The normalizer/relay tests cover timestamp order, original-age preservation,
unit/name mapping, invalid targets, source ownership and epoch transitions. Actual
ROS/DDS communication still requires the target host's ROS 2 installation; no
physical exoskeleton was attached in these tests.
