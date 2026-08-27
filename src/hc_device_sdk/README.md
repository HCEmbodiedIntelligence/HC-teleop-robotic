# hc_device_sdk

`hc_device_sdk` is a vendor-free C++17 contract and joint-mapping library for modular HC device
adapters. It defines component identity/kind, capabilities, lifecycle, health, safe stop, logical
SI joint state/command DTOs, and strict vendor-to-logical conversion helpers. It contains no ROS
node, runtime, transport codec, or vendor SDK.

## Preferred integration path

For new actuator integrations, use ROS 2 Control first:

- arms, waist, grippers, dexterous hands, and joint-driven bases should normally implement
  `hardware_interface::SystemInterface`, `ActuatorInterface`, or `SensorInterface`;
- use `controller_manager`, standard broadcasters/controllers, and one combined joint-state view;
- cameras and high-bandwidth sensors should remain normal ROS driver processes publishing standard
  sensor messages;
- keep vendor libraries in the adapter repository, never in this SDK.

`DevicePlugin` and `JointDevicePlugin` are a compatibility seam for a device that cannot yet use
ROS 2 Control, or for an existing HC plugin that needs a stable migration boundary. They are not a
second controller manager. One plugin instance represents one component ID; a runtime may load
multiple instances and aggregate/split state and commands by component.

## Lifecycle and safety contract

The normal order is `configure -> connect -> activate`. Shutdown reverses it with
`deactivate -> disconnect`. Invalid transitions return `DeviceError::kInvalidState` rather than
silently succeeding. `stop()` is exceptional and must be idempotent: repeated calls leave the
device in an equal or safer state. For a position-controlled actuator this normally means holding
fresh measured position. A software stop does not replace a certified hardware emergency stop.

`health()` identifies the component and reports communication, lifecycle, and safe-stop state.
Drivers must use monotonic time for feedback/watchdog age. `capabilities()` is descriptive; it does
not grant control authority.

## Joint mapping

The public plugin boundary is logical and SI: rad, rad/s, rad/s^2, and N*m. `JointMappingTable`
contains the only vendor-unit conversion:

```text
logical_position = scale * vendor_position + offset_rad
logical_velocity = scale * vendor_velocity
logical_effort = vendor_effort / scale
```

The inverse command conversion follows the same position/velocity relationship and virtual work:
`vendor_effort = scale * logical_effort`. Negative scale therefore changes both axis direction and
the effort sign. State conversion requires all configured component joints, tolerates unrelated
vendor joints, and returns logical values in configured order. Command conversion supports named
partial commands and preserves their input order.

## Legacy `RobotDriverPlugin` migration

Do not add a compile dependency on the old humanoid driver interface here. Put the bridge in a
separate compatibility package:

1. expose one old driver instance as one `JointDevicePlugin` component;
2. translate the typed `DeviceConfiguration` to the legacy mapping/string parameters;
3. map `stop()` to legacy `stopAll()` and preserve its idempotent hold requirement;
4. convert legacy health/error values without weakening fault or stale states;
5. migrate the vendor implementation to ROS 2 Control, then remove only the bridge package.

This keeps old proprietary ABI dependencies outside the common SDK.

Pluginlib and ament-index registration examples are documented in
[docs/plugin_registration.md](docs/plugin_registration.md). The capability-manifest JSON schema is
installed from [schema/device_plugin_manifest.schema.json](schema/device_plugin_manifest.schema.json).

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hc_device_sdk
colcon test --packages-select hc_device_sdk
colcon test-result --verbose
```
