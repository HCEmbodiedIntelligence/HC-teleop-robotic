#ifndef HC_MOTION__MOTION_ROUTER_NODE_HPP_
#define HC_MOTION__MOTION_ROUTER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace hc_motion
{

class MotionRouterNode : public rclcpp::Node
{
public:
  explicit MotionRouterNode(const rclcpp::NodeOptions & options);
  ~MotionRouterNode() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hc_motion

#endif  // HC_MOTION__MOTION_ROUTER_NODE_HPP_
