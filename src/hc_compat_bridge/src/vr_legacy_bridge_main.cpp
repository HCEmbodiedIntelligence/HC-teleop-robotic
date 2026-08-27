#include <memory>

#include "hc_compat_bridge/vr_legacy_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hc_compat_bridge::VrLegacyBridgeNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
