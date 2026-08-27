#ifndef HC_TELEOP_CORE__VR_MAPPER_NODE_HPP_
#define HC_TELEOP_CORE__VR_MAPPER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace hc_teleop_core
{

class VrMapperNode : public rclcpp::Node
{
public:
  explicit VrMapperNode(const rclcpp::NodeOptions & options);
  ~VrMapperNode() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hc_teleop_core

#endif  // HC_TELEOP_CORE__VR_MAPPER_NODE_HPP_
