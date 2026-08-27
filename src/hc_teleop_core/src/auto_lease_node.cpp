#include "hc_teleop_core/auto_lease_node.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace hc_teleop_core
{
namespace
{

std::chrono::milliseconds positiveMilliseconds(const double seconds, const char * name)
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive and finite");
  }
  return std::chrono::milliseconds(static_cast<std::int64_t>(std::llround(seconds * 1000.0)));
}

}  // namespace

AutoLeaseNode::AutoLeaseNode(const rclcpp::NodeOptions & options)
: Node("auto_control_lease", options),
  enabled_(declare_parameter<bool>("enabled", false)),
  source_id_(declare_parameter<std::string>("source_id", "vr")),
  lease_duration_(positiveMilliseconds(
      declare_parameter<double>("lease_duration_sec", 1.0), "lease_duration_sec")),
  renew_margin_(positiveMilliseconds(
      declare_parameter<double>("renew_margin_sec", 0.35), "renew_margin_sec"))
{
  if (source_id_.empty()) {
    throw std::invalid_argument("source_id must not be empty");
  }

  const auto vr_topic = declare_parameter<std::string>("vr_topic", "input/vr_frame");
  const auto enable_service = declare_parameter<std::string>(
    "enable_service", "safety/set_enabled");
  const auto acquire_service = declare_parameter<std::string>(
    "acquire_service", "control/acquire");

  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  vr_subscription_ = create_subscription<VrFrame>(
    vr_topic, qos, [this](VrFrame::SharedPtr message) { frameCallback(std::move(message)); });
  enable_client_ = create_client<SetEnabled>(enable_service);
  acquire_client_ = create_client<AcquireControl>(acquire_service);
  timer_ = create_wall_timer(
    positiveMilliseconds(declare_parameter<double>("poll_period_sec", 0.1), "poll_period_sec"),
    [this]() { timerCallback(); });

  RCLCPP_INFO(
    get_logger(), "auto control lease %s for source=%s",
    enabled_ ? "enabled" : "disabled", source_id_.c_str());
}

void AutoLeaseNode::frameCallback(const VrFrame::SharedPtr message)
{
  if (!message || message->source_id.empty()) {
    return;
  }
  if (!have_session_ || session_id_ != message->source_id) {
    session_id_ = message->source_id;
    lease_id_.clear();
    lease_pending_ = false;
    have_session_ = true;
    RCLCPP_INFO(get_logger(), "tracking VR session for auto lease: %s", session_id_.c_str());
  }
}

void AutoLeaseNode::timerCallback()
{
  if (!enabled_ || !have_session_) {
    return;
  }
  if (!enable_confirmed_) {
    requestEnable();
    return;
  }
  const auto now_ros = now();
  const auto renew_margin = rclcpp::Duration::from_nanoseconds(
    std::chrono::duration_cast<std::chrono::nanoseconds>(renew_margin_).count());
  const bool lease_missing = lease_id_.empty();
  const bool lease_needs_renewal =
    !lease_missing && (lease_expires_at_ - now_ros) <= renew_margin;
  if (lease_missing || lease_needs_renewal) {
    requestLease();
  }
}

void AutoLeaseNode::requestEnable()
{
  if (enable_pending_ || !enable_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<SetEnabled::Request>();
  request->enabled = true;
  request->requester_id = source_id_ + "_auto_lease";
  request->reason = "auto enable for simulation lease";
  enable_pending_ = true;
  enable_client_->async_send_request(
    request,
    [this](rclcpp::Client<SetEnabled>::SharedFuture future) {
      enable_pending_ = false;
      const auto response = future.get();
      enable_confirmed_ = response->success;
      if (!response->success) {
        RCLCPP_WARN(get_logger(), "auto enable rejected: %s", response->reason.c_str());
      }
    });
}

void AutoLeaseNode::requestLease()
{
  if (lease_pending_ || !acquire_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<AcquireControl::Request>();
  request->source_id = source_id_;
  request->session_id = session_id_;
  request->requested_duration = durationMessage(lease_duration_);
  lease_pending_ = true;
  acquire_client_->async_send_request(
    request,
    [this](rclcpp::Client<AcquireControl>::SharedFuture future) {
      lease_pending_ = false;
      const auto response = future.get();
      if (!response->granted) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "auto lease rejected: %s",
          response->reason.c_str());
        return;
      }
      lease_id_ = response->lease_id;
      lease_expires_at_ = rclcpp::Time(response->expires_at);
    });
}

builtin_interfaces::msg::Duration AutoLeaseNode::durationMessage(
  const std::chrono::milliseconds duration)
{
  builtin_interfaces::msg::Duration message;
  message.sec = static_cast<std::int32_t>(duration.count() / 1000);
  message.nanosec = static_cast<std::uint32_t>((duration.count() % 1000) * 1000000);
  return message;
}

}  // namespace hc_teleop_core

RCLCPP_COMPONENTS_REGISTER_NODE(hc_teleop_core::AutoLeaseNode)
