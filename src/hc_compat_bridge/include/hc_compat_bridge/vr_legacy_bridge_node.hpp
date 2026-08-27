#ifndef HC_COMPAT_BRIDGE__VR_LEGACY_BRIDGE_NODE_HPP_
#define HC_COMPAT_BRIDGE__VR_LEGACY_BRIDGE_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace hc_compat_bridge
{

class VrLegacyBridgeNode : public rclcpp::Node
{
public:
  explicit VrLegacyBridgeNode(const rclcpp::NodeOptions & options);
  ~VrLegacyBridgeNode() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hc_compat_bridge

#endif  // HC_COMPAT_BRIDGE__VR_LEGACY_BRIDGE_NODE_HPP_
