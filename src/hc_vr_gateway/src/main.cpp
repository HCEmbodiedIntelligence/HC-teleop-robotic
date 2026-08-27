#include <exception>
#include <memory>

#include "hc_vr_gateway/vr_gateway_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hc_vr_gateway::VrGatewayNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hc_vr_gateway"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
