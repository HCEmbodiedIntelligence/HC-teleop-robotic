#include "hc_device_sdk/types.hpp"

namespace hc_device_sdk
{

const char * toString(const ComponentKind kind) noexcept
{
  switch (kind) {
    case ComponentKind::kUnknown:
      return "unknown";
    case ComponentKind::kArm:
      return "arm";
    case ComponentKind::kGripper:
      return "gripper";
    case ComponentKind::kDexterousHand:
      return "dexterous_hand";
    case ComponentKind::kCamera:
      return "camera";
    case ComponentKind::kWaist:
      return "waist";
    case ComponentKind::kBase:
      return "base";
    case ComponentKind::kSensor:
      return "sensor";
  }
  return "unknown";
}

bool componentKindFromString(const std::string & value, ComponentKind & output) noexcept
{
  static const std::map<std::string, ComponentKind> kinds{
    {"unknown", ComponentKind::kUnknown},
    {"arm", ComponentKind::kArm},
    {"gripper", ComponentKind::kGripper},
    {"dexterous_hand", ComponentKind::kDexterousHand},
    {"camera", ComponentKind::kCamera},
    {"waist", ComponentKind::kWaist},
    {"base", ComponentKind::kBase},
    {"sensor", ComponentKind::kSensor},
  };
  const auto found = kinds.find(value);
  if (found == kinds.end()) {
    return false;
  }
  output = found->second;
  return true;
}

}  // namespace hc_device_sdk
