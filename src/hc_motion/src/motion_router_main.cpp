#include <memory>

#include "hc_motion/motion_router_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hc_motion::MotionRouterNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
