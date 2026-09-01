#include <memory>

#include "hc_diagnostics/control_chain_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hc_diagnostics::ControlChainNode>());
  rclcpp::shutdown();
  return 0;
}
