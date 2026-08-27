#include <memory>

#include "hc_teleop_core/vr_mapper_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hc_teleop_core::VrMapperNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
