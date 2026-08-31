#include "hc_motion/motion_router_node.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "hc_motion/route_guard.hpp"
#include "hc_teleop_interfaces/msg/cartesian_target_array.hpp"
#include "hc_teleop_interfaces/msg/joint_command_candidate.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hc_motion
{
namespace
{

SteadyTime deadline_from_ros(
  const builtin_interfaces::msg::Time & deadline, const rclcpp::Time & ros_now,
  SteadyTime steady_now)
{
  const rclcpp::Time target(deadline);
  return steady_now + std::chrono::nanoseconds((target - ros_now).nanoseconds());
}

TargetEnvelope convert_target(
  const hc_teleop_interfaces::msg::CartesianTargetArray & message,
  const rclcpp::Time & ros_now, SteadyTime steady_now)
{
  TargetEnvelope result;
  result.source_id = message.source_id;
  result.session_id = message.session_id;
  result.sequence = message.sequence;
  result.valid_until = deadline_from_ros(message.valid_until, ros_now, steady_now);
  result.targets.reserve(message.targets.size());
  for (const auto & target : message.targets) {
    CartesianTargetDescriptor value;
    value.group_name = target.group_name;
    value.reference_frame = target.reference_frame;
    value.tip_frame = target.tip_frame;
    value.position = {target.pose.position.x, target.pose.position.y, target.pose.position.z};
    value.orientation = {target.pose.orientation.x, target.pose.orientation.y,
      target.pose.orientation.z, target.pose.orientation.w};
    result.targets.push_back(std::move(value));
  }
  return result;
}

CandidateEnvelope convert_candidate(
  const hc_teleop_interfaces::msg::JointCommandCandidate & message,
  const rclcpp::Time & ros_now, SteadyTime steady_now)
{
  CandidateEnvelope result;
  result.source_id = message.source_id;
  result.session_id = message.session_id;
  result.group_name = message.group_name;
  result.sequence = message.sequence;
  result.valid_until = deadline_from_ros(message.valid_until, ros_now, steady_now);
  result.control_mode = message.control_mode;
  result.joint_names = message.command.name;
  result.positions = message.command.position;
  result.velocities = message.command.velocity;
  result.efforts = message.command.effort;
  return result;
}

}  // namespace

class MotionRouterNode::Impl
{
public:
  explicit Impl(MotionRouterNode & node)
  : node_(node), guard_(declare_groups())
  {
    const auto target_input = node_.declare_parameter<std::string>(
      "target_input_topic", "teleop/cartesian_targets");
    const auto backend_target = node_.declare_parameter<std::string>(
      "backend_target_topic", "motion/backend/cartesian_targets");
    const auto backend_candidate = node_.declare_parameter<std::string>(
      "backend_candidate_topic", "motion/backend/joint_candidate");
    const auto candidate_output = node_.declare_parameter<std::string>(
      "candidate_output_topic", "control/joint_candidate");
    for (const auto * topic : {&target_input, &backend_target, &backend_candidate, &candidate_output}) {
      if (topic->empty()) {
        throw std::invalid_argument("motion router topic parameters must not be empty");
      }
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    auto candidate_qos = rclcpp::SensorDataQoS().keep_last(16);
    target_publisher_ =
      node_.create_publisher<hc_teleop_interfaces::msg::CartesianTargetArray>(backend_target, qos);
    candidate_publisher_ =
      node_.create_publisher<hc_teleop_interfaces::msg::JointCommandCandidate>(
      candidate_output, candidate_qos);
    target_subscription_ = node_.create_subscription<
      hc_teleop_interfaces::msg::CartesianTargetArray>(
      target_input, qos,
      [this](hc_teleop_interfaces::msg::CartesianTargetArray::ConstSharedPtr message) {
        const auto ros_now = node_.now();
        const auto steady_now = SteadyClock::now();
        std::string reason;
        if (!guard_.route(convert_target(*message, ros_now, steady_now), steady_now, reason)) {
          ++rejected_targets_;
          warn("Cartesian target", reason);
          return;
        }
        target_publisher_->publish(*message);
        ++routed_targets_;
      });
    candidate_subscription_ = node_.create_subscription<
      hc_teleop_interfaces::msg::JointCommandCandidate>(
      backend_candidate, candidate_qos,
      [this](hc_teleop_interfaces::msg::JointCommandCandidate::ConstSharedPtr message) {
        const auto ros_now = node_.now();
        const auto steady_now = SteadyClock::now();
        std::string reason;
        if (!guard_.authorize(convert_candidate(*message, ros_now, steady_now), steady_now, reason)) {
          ++rejected_candidates_;
          warn("backend candidate", reason);
          return;
        }
        candidate_publisher_->publish(*message);
        ++routed_candidates_;
      });
    RCLCPP_INFO(node_.get_logger(), "Motion router active with strict target/candidate correlation");
  }

  ~Impl()
  {
    RCLCPP_INFO(node_.get_logger(),
      "Motion router stopped: targets=%llu/%llu candidates=%llu/%llu",
      static_cast<unsigned long long>(routed_targets_),
      static_cast<unsigned long long>(rejected_targets_),
      static_cast<unsigned long long>(routed_candidates_),
      static_cast<unsigned long long>(rejected_candidates_));
  }

private:
  std::vector<std::string> declare_groups()
  {
    auto groups = node_.declare_parameter<std::vector<std::string>>(
      "allowed_groups", std::vector<std::string>{});
    if (groups.empty()) {
      throw std::invalid_argument("allowed_groups must be supplied from the robot profile");
    }
    return groups;
  }

  void warn(const char * kind, const std::string & reason)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "Rejected %s: %s", kind, reason.c_str());
  }

  MotionRouterNode & node_;
  RouteGuard guard_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::CartesianTargetArray>::SharedPtr target_publisher_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::JointCommandCandidate>::SharedPtr candidate_publisher_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::CartesianTargetArray>::SharedPtr target_subscription_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::JointCommandCandidate>::SharedPtr
    candidate_subscription_;
  std::uint64_t routed_targets_{0U};
  std::uint64_t rejected_targets_{0U};
  std::uint64_t routed_candidates_{0U};
  std::uint64_t rejected_candidates_{0U};
};

MotionRouterNode::MotionRouterNode(const rclcpp::NodeOptions & options)
: Node("hc_motion_router", options), impl_(std::make_unique<Impl>(*this))
{
}

MotionRouterNode::~MotionRouterNode() = default;

}  // namespace hc_motion

RCLCPP_COMPONENTS_REGISTER_NODE(hc_motion::MotionRouterNode)
