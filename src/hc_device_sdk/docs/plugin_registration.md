# Plugin and ament-index registration

Process-level ROS adapters and ROS 2 Control hardware plugins remain the preferred public extension
boundary. When an in-process HC compatibility plugin is justified, export it with standard
pluginlib and publish a separate capability manifest through the ament index.

The plugin implementation exports exactly one SDK base class:

```cpp
#include <pluginlib/class_list_macros.hpp>
#include <hc_device_sdk/device_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(
  example_arm_driver::ExampleArmDriver,
  hc_device_sdk::JointDevicePlugin)
```

Its `plugins/device_plugins.xml` follows
[the installed example](../examples/device_plugins.xml). Register the loader XML and the capability
manifest from the adapter's `CMakeLists.txt`:

```cmake
find_package(ament_cmake REQUIRED)
find_package(hc_device_sdk REQUIRED)
find_package(pluginlib REQUIRED)

pluginlib_export_plugin_description_file(
  hc_device_sdk plugins/device_plugins.xml)

ament_index_register_resource(
  hc_device_plugin_manifests
  CONTENT "share/${PROJECT_NAME}/device_plugin_manifest.yaml")

install(FILES device_plugin_manifest.yaml DESTINATION share/${PROJECT_NAME})
```

The ament marker named after the adapter package contains the installed relative path to its YAML
manifest. Discovery code enumerates the `hc_device_plugin_manifests` resource type, resolves each
path inside that package prefix, validates it against `device_plugin_manifest.schema.json`, and only
then displays or loads the declared classes. The YAML is discovery metadata; the plugin class must
still return the same information from `capabilities()` at runtime. A mismatch is a configuration
error.

The adapter's `package.xml` includes:

```xml
<depend>hc_device_sdk</depend>
<depend>pluginlib</depend>
<export>
  <build_type>ament_cmake</build_type>
</export>
```

On ROS 2, `pluginlib_export_plugin_description_file()` creates the pluginlib ament-index resource;
do not also add the legacy ROS 1-style `${prefix}` plugin export element.

Never put an address, password, token, serial number, calibration, or absolute filesystem path in
the discovery manifest. Those values belong to deployment configuration selected at launch.
