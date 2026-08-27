#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hc_teleop_core/command_arbiter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hc_teleop_core::CommandArbiterNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
