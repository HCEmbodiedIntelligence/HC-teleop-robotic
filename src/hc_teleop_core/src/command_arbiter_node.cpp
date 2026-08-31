#include "hc_teleop_core/command_arbiter_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

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

bool sameStatus(const ArbiterStatus & left, const ArbiterStatus & right)
{
  return left.code == right.code && left.enabled == right.enabled &&
         left.fault_latched == right.fault_latched && left.reason == right.reason &&
         left.active_source == right.active_source &&
         left.active_session == right.active_session && left.lease_id == right.lease_id;
}

}  // namespace

CommandArbiterNode::CommandArbiterNode(const rclcpp::NodeOptions & options)
: Node("command_arbiter", options),
  arbiter_(positiveMilliseconds(
      declare_parameter<double>("command_timeout_sec", 0.15), "command_timeout_sec")),
  default_lease_duration_(positiveMilliseconds(
      declare_parameter<double>("default_lease_sec", 1.0), "default_lease_sec")),
  maximum_lease_duration_(positiveMilliseconds(
      declare_parameter<double>("maximum_lease_sec", 5.0), "maximum_lease_sec"))
{
  const auto source_priorities = declare_parameter<std::vector<std::string>>(
    "source_priorities", {"safety:255", "vr:100", "exoskeleton:80", "replay:10"});
  for (const auto & item : source_priorities) {
    const auto separator = item.find(':');
    if (separator == std::string::npos || separator == 0 || separator + 1 >= item.size()) {
      throw std::invalid_argument("source_priorities entries must use source:priority");
    }
    const auto parsed = std::stoul(item.substr(separator + 1));
    if (parsed > std::numeric_limits<std::uint8_t>::max()) {
      throw std::invalid_argument("source priority must be in [0, 255]");
    }
    priorities_[item.substr(0, separator)] = static_cast<std::uint8_t>(parsed);
  }

  // Joint commands are published as one message per component group.  A
  // depth-one history lets back-to-back left/right arm and tool messages
  // overwrite each other before the subscriber can run.
  auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(16));
  stream_qos.best_effort().durability_volatile();
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  state_qos.reliable().transient_local();

  const auto candidate_topic = declare_parameter<std::string>(
    "candidate_topic", "control/joint_candidate");
  const auto command_topic = declare_parameter<std::string>(
    "command_topic", "control/joint_command");
  const auto safety_topic = declare_parameter<std::string>(
    "safety_topic", "safety/state");

  rclcpp::PublisherOptions state_pub_options;
  state_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  command_publisher_ = create_publisher<JointCommand>(command_topic, stream_qos);
  safety_publisher_ = create_publisher<SafetyState>(safety_topic, state_qos, state_pub_options);
  candidate_subscription_ = create_subscription<JointCommandCandidate>(
    candidate_topic, stream_qos,
    std::bind(&CommandArbiterNode::candidateCallback, this, std::placeholders::_1));
  acquire_service_ = create_service<AcquireControl>(
    "control/acquire",
    std::bind(
      &CommandArbiterNode::acquireCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  enabled_service_ = create_service<SetEnabled>(
    "safety/set_enabled",
    std::bind(
      &CommandArbiterNode::enabledCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  reset_service_ = create_service<ResetFault>(
    "safety/reset_fault",
    std::bind(
      &CommandArbiterNode::resetCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  const auto rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be positive and finite");
  }
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz),
    std::bind(&CommandArbiterNode::timerCallback, this));
  arbiter_.setEnabled(declare_parameter<bool>("enabled_on_start", false),
    std::chrono::steady_clock::now());
  publishStatus(true);
  RCLCPP_INFO(
    get_logger(),
    "command arbiter ready: candidate=%s final=%s watchdog=%.0f ms",
    candidate_topic.c_str(), command_topic.c_str(),
    get_parameter("command_timeout_sec").as_double() * 1000.0);
}

void CommandArbiterNode::candidateCallback(const JointCommandCandidate::SharedPtr message)
{
  const auto now_ros = now();
  const auto now_steady = std::chrono::steady_clock::now();
  const auto valid_until_ns = timeNanoseconds(message->valid_until);
  const auto remaining_ns = valid_until_ns - now_ros.nanoseconds();
  if (remaining_ns <= 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "rejected expired command candidate from %s",
      message->source_id.c_str());
    return;
  }

  JointCandidate candidate;
  candidate.source_id = message->source_id;
  candidate.session_id = message->session_id;
  candidate.group_name = message->group_name;
  candidate.sequence = message->sequence;
  candidate.control_mode = message->control_mode;
  candidate.valid_until = now_steady + std::chrono::nanoseconds(remaining_ns);
  candidate.valid_until_ros_nanoseconds = valid_until_ns;
  candidate.names = message->command.name;
  candidate.positions = message->command.position;
  candidate.velocities = message->command.velocity;
  candidate.efforts = message->command.effort;
  std::string reason;
  if (!arbiter_.submit(std::move(candidate), now_steady, reason)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "rejected command candidate from %s: %s",
      message->source_id.c_str(), reason.c_str());
  }
}

void CommandArbiterNode::acquireCallback(
  const AcquireControl::Request::SharedPtr request,
  AcquireControl::Response::SharedPtr response)
{
  const auto requested_ns =
    std::chrono::seconds(request->requested_duration.sec) +
    std::chrono::nanoseconds(request->requested_duration.nanosec);
  auto duration = requested_ns > std::chrono::nanoseconds::zero() ?
    std::chrono::duration_cast<std::chrono::milliseconds>(requested_ns) :
    default_lease_duration_;
  duration = std::min(duration, maximum_lease_duration_);
  const auto now_steady = std::chrono::steady_clock::now();
  const auto result = arbiter_.acquire(
    request->source_id, request->session_id, priorityFor(request->source_id), duration,
    now_steady);
  response->granted = result.granted;
  response->lease_id = result.lease_id;
  response->expires_at = toRosTime(now(), now_steady, result.expires_at);
  response->reason = result.reason;
  publishStatus(true);
}

void CommandArbiterNode::enabledCallback(
  const SetEnabled::Request::SharedPtr request,
  SetEnabled::Response::SharedPtr response)
{
  arbiter_.setEnabled(request->enabled, std::chrono::steady_clock::now());
  const auto current = arbiter_.status(std::chrono::steady_clock::now());
  response->success = true;
  response->state = safetyMessage(current);
  response->reason = current.reason;
  publishStatus(true);
}

void CommandArbiterNode::resetCallback(
  const ResetFault::Request::SharedPtr,
  ResetFault::Response::SharedPtr response)
{
  response->success = arbiter_.resetFault();
  const auto current = arbiter_.status(std::chrono::steady_clock::now());
  response->state = safetyMessage(current);
  response->reason = current.reason;
  publishStatus(true);
}

void CommandArbiterNode::timerCallback()
{
  const auto now_steady = std::chrono::steady_clock::now();
  const auto selected = arbiter_.commands(now_steady);
  for (const auto & command : selected) {
    JointCommand output;
    output.header.stamp = now();
    output.source_id = command.source_id;
    output.session_id = command.session_id;
    output.sequence = command.sequence;
    output.valid_until = toRosTime(command.valid_until_ros_nanoseconds);
    output.control_mode = command.control_mode;
    output.group_name = command.group_name;
    output.command.header = output.header;
    output.command.name = command.names;
    output.command.position = command.positions;
    output.command.velocity = command.velocities;
    output.command.effort = command.efforts;
    command_publisher_->publish(output);
  }
  publishStatus(false);
}

void CommandArbiterNode::publishStatus(const bool force)
{
  const auto now_steady = std::chrono::steady_clock::now();
  const auto current = arbiter_.status(now_steady);
  if (!force && have_published_status_ && sameStatus(current, last_published_status_) &&
    now_steady - last_status_publish_ < std::chrono::seconds(1))
  {
    return;
  }
  safety_publisher_->publish(safetyMessage(current));
  last_published_status_ = current;
  have_published_status_ = true;
  last_status_publish_ = now_steady;
}

CommandArbiterNode::SafetyState CommandArbiterNode::safetyMessage(
  const ArbiterStatus & current) const
{
  SafetyState message;
  message.header.stamp = now();
  switch (current.code) {
    case SafetyCode::kDisabled:
      message.state = SafetyState::DISABLED;
      break;
    case SafetyCode::kActive:
      message.state = SafetyState::ACTIVE;
      break;
    case SafetyCode::kFault:
      message.state = SafetyState::FAULT;
      break;
    case SafetyCode::kReady:
    case SafetyCode::kHolding:
    default:
      message.state = SafetyState::STANDBY;
      break;
  }
  message.enabled = current.enabled;
  message.fault_latched = current.fault_latched;
  message.estop_active = false;
  message.reason = current.reason;
  message.active_source = current.active_source;
  message.active_session = current.active_session;
  message.lease_expires_at = toRosTime(
    now(), std::chrono::steady_clock::now(), current.lease_expires_at);
  return message;
}

std::uint8_t CommandArbiterNode::priorityFor(const std::string & source_id) const
{
  const auto found = priorities_.find(source_id);
  return found == priorities_.end() ? 0 : found->second;
}

builtin_interfaces::msg::Time CommandArbiterNode::toRosTime(const std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Time result;
  if (nanoseconds <= 0) {
    return result;
  }
  result.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return result;
}

builtin_interfaces::msg::Time CommandArbiterNode::toRosTime(
  const rclcpp::Time & now_ros,
  const SteadyTime now_steady,
  const SteadyTime target_steady)
{
  if (target_steady <= now_steady) {
    return builtin_interfaces::msg::Time{};
  }
  return toRosTime(
    now_ros.nanoseconds() +
    std::chrono::duration_cast<std::chrono::nanoseconds>(target_steady - now_steady).count());
}

std::int64_t CommandArbiterNode::timeNanoseconds(
  const builtin_interfaces::msg::Time & value)
{
  return static_cast<std::int64_t>(value.sec) * 1000000000LL + value.nanosec;
}

}  // namespace hc_teleop_core

RCLCPP_COMPONENTS_REGISTER_NODE(hc_teleop_core::CommandArbiterNode)
