#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <hc_teleop_interfaces/msg/joint_command.hpp>
#include <hc_teleop_interfaces/msg/joint_command_candidate.hpp>
#include <hc_teleop_interfaces/msg/safety_state.hpp>
#include <hc_teleop_interfaces/srv/acquire_control.hpp>
#include <hc_teleop_interfaces/srv/reset_fault.hpp>
#include <hc_teleop_interfaces/srv/set_enabled.hpp>

#include "hc_teleop_core/command_arbiter.hpp"

namespace hc_teleop_core
{

class CommandArbiterNode final : public rclcpp::Node
{
public:
  explicit CommandArbiterNode(const rclcpp::NodeOptions & options);

private:
  using JointCommand = hc_teleop_interfaces::msg::JointCommand;
  using JointCommandCandidate = hc_teleop_interfaces::msg::JointCommandCandidate;
  using SafetyState = hc_teleop_interfaces::msg::SafetyState;
  using AcquireControl = hc_teleop_interfaces::srv::AcquireControl;
  using ResetFault = hc_teleop_interfaces::srv::ResetFault;
  using SetEnabled = hc_teleop_interfaces::srv::SetEnabled;

  void candidateCallback(const JointCommandCandidate::SharedPtr message);
  void acquireCallback(
    const AcquireControl::Request::SharedPtr request,
    AcquireControl::Response::SharedPtr response);
  void enabledCallback(
    const SetEnabled::Request::SharedPtr request,
    SetEnabled::Response::SharedPtr response);
  void resetCallback(
    const ResetFault::Request::SharedPtr request,
    ResetFault::Response::SharedPtr response);
  void timerCallback();
  void publishStatus(bool force = false);
  SafetyState safetyMessage(const ArbiterStatus & status) const;
  std::uint8_t priorityFor(const std::string & source_id) const;
  static builtin_interfaces::msg::Time toRosTime(std::int64_t nanoseconds);
  static builtin_interfaces::msg::Time toRosTime(
    const rclcpp::Time & now_ros,
    SteadyTime now_steady,
    SteadyTime target_steady);
  static std::int64_t timeNanoseconds(const builtin_interfaces::msg::Time & value);

  CommandArbiter arbiter_;
  std::unordered_map<std::string, std::uint8_t> priorities_;
  std::chrono::milliseconds default_lease_duration_;
  std::chrono::milliseconds maximum_lease_duration_;
  rclcpp::Publisher<JointCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<SafetyState>::SharedPtr safety_publisher_;
  rclcpp::Subscription<JointCommandCandidate>::SharedPtr candidate_subscription_;
  rclcpp::Service<AcquireControl>::SharedPtr acquire_service_;
  rclcpp::Service<SetEnabled>::SharedPtr enabled_service_;
  rclcpp::Service<ResetFault>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  ArbiterStatus last_published_status_;
  bool have_published_status_{false};
  std::chrono::steady_clock::time_point last_status_publish_{};
};

}  // namespace hc_teleop_core
