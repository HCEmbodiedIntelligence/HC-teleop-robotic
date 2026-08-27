#ifndef HC_DEVICE_SDK__DEVICE_PLUGIN_HPP_
#define HC_DEVICE_SDK__DEVICE_PLUGIN_HPP_

#include "hc_device_sdk/types.hpp"

namespace hc_device_sdk
{

// Hardware-neutral lifecycle contract. Implementations must not start a
// private ROS executor from these calls; the owning runtime controls threading.
class DevicePlugin
{
public:
  virtual ~DevicePlugin() = default;

  virtual DeviceResult configure(const DeviceConfiguration & configuration) = 0;
  virtual DeviceResult connect() = 0;
  virtual DeviceResult disconnect() = 0;
  virtual DeviceResult activate() = 0;
  virtual DeviceResult deactivate() = 0;

  // The safety path. Repeated calls must succeed and leave the component in an
  // equal or safer state. A stopped position-controlled actuator normally holds
  // measured position; software stop never replaces a hardware emergency stop.
  virtual DeviceResult stop() = 0;

  virtual DeviceHealth health() = 0;
  virtual DeviceCapabilities capabilities() const = 0;
};

// Optional specialization for joint-addressable devices such as arms, waists,
// grippers, dexterous hands, and joint-driven mobile bases.
class JointDevicePlugin : public DevicePlugin
{
public:
  ~JointDevicePlugin() override = default;

  virtual DeviceResult readJointState(JointState & state) = 0;
  virtual DeviceResult writeJointCommand(const JointCommand & command) = 0;
};

}  // namespace hc_device_sdk

#endif  // HC_DEVICE_SDK__DEVICE_PLUGIN_HPP_
