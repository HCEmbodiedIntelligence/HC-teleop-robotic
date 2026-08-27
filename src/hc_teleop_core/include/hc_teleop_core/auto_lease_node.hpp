#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <hc_teleop_interfaces/msg/vr_frame.hpp>
#include <hc_teleop_interfaces/srv/acquire_control.hpp>
#include <hc_teleop_interfaces/srv/set_enabled.hpp>

namespace hc_teleop_core
{

class AutoLeaseNode final : public rclcpp::Node
{
public:
  explicit AutoLeaseNode(const rclcpp::NodeOptions & options);

private:
  using AcquireControl = hc_teleop_interfaces::srv::AcquireControl;
  using SetEnabled = hc_teleop_interfaces::srv::SetEnabled;
  using VrFrame = hc_teleop_interfaces::msg::VrFrame;

  void frameCallback(const VrFrame::SharedPtr message);
  void timerCallback();
  void requestEnable();
  void requestLease();
  static builtin_interfaces::msg::Duration durationMessage(std::chrono::milliseconds duration);

  bool enabled_{false};
  bool have_session_{false};
  bool enable_confirmed_{false};
  bool enable_pending_{false};
  bool lease_pending_{false};
  std::string source_id_;
  std::string session_id_;
  std::string lease_id_;
  rclcpp::Time lease_expires_at_;
  std::chrono::milliseconds lease_duration_;
  std::chrono::milliseconds renew_margin_;
  rclcpp::Client<SetEnabled>::SharedPtr enable_client_;
  rclcpp::Client<AcquireControl>::SharedPtr acquire_client_;
  rclcpp::Subscription<VrFrame>::SharedPtr vr_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hc_teleop_core
