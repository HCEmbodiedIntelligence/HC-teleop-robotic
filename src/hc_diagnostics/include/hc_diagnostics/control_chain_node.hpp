#pragma once

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace hc_diagnostics
{

class ControlChainNode final : public rclcpp::Node
{
public:
  explicit ControlChainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControlChainNode() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hc_diagnostics
