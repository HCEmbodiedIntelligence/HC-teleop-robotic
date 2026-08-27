#ifndef HC_COMPAT_BRIDGE__VR_LEGACY_JSON_HPP_
#define HC_COMPAT_BRIDGE__VR_LEGACY_JSON_HPP_

#include <optional>
#include <string>

#include "hc_teleop_interfaces/msg/vr_frame.hpp"

namespace hc_compat_bridge
{

/// Convert a validated VrFrame into the historical std_msgs/String payload.
/// Returns nullopt if any numeric field is non-finite.
[[nodiscard]] std::optional<std::string> to_legacy_vr_json(
  const hc_teleop_interfaces::msg::VrFrame & frame);

}  // namespace hc_compat_bridge

#endif  // HC_COMPAT_BRIDGE__VR_LEGACY_JSON_HPP_
