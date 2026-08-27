#ifndef HC_VR_GATEWAY__VR_GATEWAY_NODE_HPP_
#define HC_VR_GATEWAY__VR_GATEWAY_NODE_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace hc_vr_gateway
{

class VrGatewayNode final : public rclcpp::Node
{
public:
  explicit VrGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VrGatewayNode() override;

  VrGatewayNode(const VrGatewayNode &) = delete;
  VrGatewayNode & operator=(const VrGatewayNode &) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hc_vr_gateway

#endif  // HC_VR_GATEWAY__VR_GATEWAY_NODE_HPP_
