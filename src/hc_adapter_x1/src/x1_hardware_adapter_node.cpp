#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hc_teleop_interfaces/msg/joint_command.hpp"
#include "hc_teleop_interfaces/msg/safety_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hc_adapter_x1
{
namespace
{

using JointCommand = hc_teleop_interfaces::msg::JointCommand;
using SafetyState = hc_teleop_interfaces::msg::SafetyState;
using SteadyTime = std::chrono::steady_clock::time_point;

std::int64_t timeNanoseconds(const builtin_interfaces::msg::Time & value)
{
  return static_cast<std::int64_t>(value.sec) * 1000000000LL + value.nanosec;
}

double positiveFinite(const double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive and finite");
  }
  return value;
}

bool validJointState(const sensor_msgs::msg::JointState & message, std::string & reason)
{
  if (message.name.empty() || message.name.size() != message.position.size()) {
    reason = "joint names and positions must be non-empty and equal length";
    return false;
  }
  std::unordered_set<std::string> names;
  names.reserve(message.name.size());
  for (std::size_t index = 0; index < message.name.size(); ++index) {
    if (message.name[index].empty() || !names.insert(message.name[index]).second) {
      reason = "joint names must be non-empty and unique";
      return false;
    }
    if (!std::isfinite(message.position[index])) {
      reason = "joint positions must be finite";
      return false;
    }
  }
  if (!message.velocity.empty() && message.velocity.size() != message.name.size()) {
    reason = "joint velocity length does not match names";
    return false;
  }
  if (!message.effort.empty() && message.effort.size() != message.name.size()) {
    reason = "joint effort length does not match names";
    return false;
  }
  return true;
}

struct FingerShape
{
  std::vector<std::string> names;
  std::vector<double> open;
  std::vector<double> closed;
};

FingerShape leftFingerShape()
{
  return {
    {"L_thumb_roll_joint", "L_thumb_abad_joint", "L_thumb_mcp_joint",
      "L_index_abad_joint", "L_index_pip_joint", "L_middle_pip_joint",
      "L_ring_abad_joint", "L_ring_pip_joint", "L_pinky_abad_joint",
      "L_pinky_pip_joint"},
    {-0.4, -0.045, 0.0, 0.05, 0.0, 0.0, -0.13, 0.0, -0.13, 0.0},
    {-1.121, 1.642, -1.0, 0.13, 2.5, 2.5, -0.05, 2.5, 0.05, 2.5}};
}

FingerShape rightFingerShape()
{
  return {
    {"R_thumb_roll_joint", "R_thumb_abad_joint", "R_thumb_mcp_joint",
      "R_index_abad_joint", "R_index_pip_joint", "R_middle_pip_joint",
      "R_ring_abad_joint", "R_ring_pip_joint", "R_pinky_abad_joint",
      "R_pinky_pip_joint"},
    {0.4, -1.642, 0.0, -0.13, 0.0, 0.0, 0.05, 0.0, 0.05, 0.0},
    {1.121, 0.045, 1.0, -0.05, 2.5, 2.5, 0.13, 2.5, 0.13, 2.5}};
}

}  // namespace

class X1HardwareAdapterNode final : public rclcpp::Node
{
public:
  explicit X1HardwareAdapterNode(const rclcpp::NodeOptions & options)
  : Node("x1_hardware_adapter", options)
  {
    command_output_enabled_ = declare_parameter<bool>("command_output_enabled", true);
    const auto publish_rate_hz = positiveFinite(
      declare_parameter<double>("command_publish_rate_hz", 100.0),
      "command_publish_rate_hz");
    command_progress_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(positiveFinite(
          declare_parameter<double>("command_progress_timeout_sec", 0.15),
          "command_progress_timeout_sec")));
    feedback_warn_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(positiveFinite(
          declare_parameter<double>("feedback_warn_timeout_sec", 0.5),
          "feedback_warn_timeout_sec")));

    motion_group_names_ = declare_parameter<std::vector<std::string>>(
      "motion_group_names", std::vector<std::string>{"left_arm", "right_arm", "waist"});
    if (motion_group_names_.empty()) {
      throw std::invalid_argument("motion_group_names must not be empty");
    }
    for (const auto & group : motion_group_names_) {
      auto names = declare_parameter<std::vector<std::string>>(
        group + ".joint_names", std::vector<std::string>{});
      if (group.empty() || names.empty()) {
        throw std::invalid_argument(group + ".joint_names must not be empty");
      }
      if (!motion_joint_names_.emplace(group, std::move(names)).second) {
        throw std::invalid_argument("motion_group_names must be unique");
      }
    }

    left_finger_group_ = declare_parameter<std::string>(
      "left_finger_group", "left_gripper");
    right_finger_group_ = declare_parameter<std::string>(
      "right_finger_group", "right_gripper");
    left_finger_input_joint_ = declare_parameter<std::string>(
      "left_finger_input_joint", "L_ban");
    right_finger_input_joint_ = declare_parameter<std::string>(
      "right_finger_input_joint", "R_ban");
    left_finger_open_ = declare_parameter<double>("left_finger_open", 0.0);
    left_finger_closed_ = declare_parameter<double>("left_finger_closed", 1.5707);
    right_finger_open_ = declare_parameter<double>("right_finger_open", 0.0);
    right_finger_closed_ = declare_parameter<double>("right_finger_closed", 1.5707);
    for (const auto value : {left_finger_open_, left_finger_closed_, right_finger_open_,
      right_finger_closed_})
    {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("finger input ranges must be finite");
      }
    }
    if (left_finger_open_ == left_finger_closed_ ||
      right_finger_open_ == right_finger_closed_)
    {
      throw std::invalid_argument("finger open and closed values must differ");
    }
    left_finger_shape_ = leftFingerShape();
    right_finger_shape_ = rightFingerShape();

    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(16));
    command_qos.best_effort().durability_volatile();
    auto legacy_command_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    legacy_command_qos.reliable().durability_volatile();
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    state_qos.reliable().transient_local();

    command_subscription_ = create_subscription<JointCommand>(
      declare_parameter<std::string>("command_topic", "control/joint_command"),
      command_qos,
      [this](JointCommand::ConstSharedPtr message) {onCommand(*message);});
    safety_subscription_ = create_subscription<SafetyState>(
      declare_parameter<std::string>("safety_topic", "safety/state"), state_qos,
      [this](SafetyState::ConstSharedPtr message) {onSafety(*message);});
    legacy_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>(
        "legacy_joint_state_topic", "/hc_teleop/joint_states"),
      sensor_qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        onLegacyJointState(*message);
      });
    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "state/joints"),
      rclcpp::QoS(10));
    legacy_command_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>(
        "legacy_joint_command_topic", "/hc_teleop/joint_cmd"),
      legacy_command_qos);
    left_finger_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>(
        "legacy_left_finger_topic", "/hc_teleop/joint_cmd_finger_left"),
      command_qos);
    right_finger_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>(
        "legacy_right_finger_topic", "/hc_teleop/joint_cmd_finger_right"),
      command_qos);

    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() {onCommandTimer();});
    feedback_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {checkFeedback();});

    RCLCPP_INFO(
      get_logger(),
      "X1 HC_X1 compatibility adapter ready: feedback /hc_teleop -> %s, "
      "commands %s -> /hc_teleop (%s)",
      state_publisher_->get_topic_name(), command_subscription_->get_topic_name(),
      command_output_enabled_ ? "enabled" : "shadow/read-only");
  }

private:
  struct BufferedCommand
  {
    std::vector<std::string> names;
    std::vector<double> positions;
    std::string source_id;
    std::string session_id;
    std::uint64_t sequence{0};
    SteadyTime expires_at{};
  };

  void onLegacyJointState(const sensor_msgs::msg::JointState & input)
  {
    std::string reason;
    if (!validJointState(input, reason)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "rejected HC_X1 joint feedback: %s",
        reason.c_str());
      return;
    }
    auto output = input;
    output.header.stamp = now();
    if (!output.velocity.empty() &&
      !std::all_of(output.velocity.begin(), output.velocity.end(),
        [](double value) {return std::isfinite(value);}))
    {
      output.velocity.clear();
    }
    if (!output.effort.empty() &&
      !std::all_of(output.effort.begin(), output.effort.end(),
        [](double value) {return std::isfinite(value);}))
    {
      output.effort.clear();
    }
    bool recovered = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_feedback_ = std::chrono::steady_clock::now();
      feedback_seen_ = true;
      recovered = feedback_stale_warned_;
      feedback_stale_warned_ = false;
    }
    if (recovered) {
      RCLCPP_INFO(get_logger(), "HC_X1 joint feedback recovered");
    }
    state_publisher_->publish(std::move(output));
  }

  void onSafety(const SafetyState & message)
  {
    const bool active = message.enabled && !message.fault_latched &&
      !message.estop_active && message.state == SafetyState::ACTIVE;
    std::lock_guard<std::mutex> lock(mutex_);
    safety_active_ = active;
    if (!active) {
      buffered_commands_.clear();
    }
  }

  bool exactGroupCommand(
    const JointCommand & message, const std::vector<std::string> & expected,
    std::vector<double> & ordered, std::string & reason) const
  {
    if (message.command.name.size() != message.command.position.size() ||
      message.command.name.size() != expected.size())
    {
      reason = "command does not contain the complete configured joint group";
      return false;
    }
    std::unordered_map<std::string, double> values;
    values.reserve(message.command.name.size());
    for (std::size_t index = 0; index < message.command.name.size(); ++index) {
      if (message.command.name[index].empty() ||
        !std::isfinite(message.command.position[index]) ||
        !values.emplace(message.command.name[index], message.command.position[index]).second)
      {
        reason = "command contains an invalid, duplicate or non-finite joint";
        return false;
      }
    }
    ordered.clear();
    ordered.reserve(expected.size());
    for (const auto & name : expected) {
      const auto found = values.find(name);
      if (found == values.end()) {
        reason = "command joint names do not match the configured group";
        return false;
      }
      ordered.push_back(found->second);
    }
    return true;
  }

  bool commandExpiry(const JointCommand & message, SteadyTime & expiry) const
  {
    const auto remaining = timeNanoseconds(message.valid_until) - now().nanoseconds();
    if (remaining <= 0) {
      return false;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    expiry = std::min(
      steady_now + std::chrono::nanoseconds(remaining),
      steady_now + command_progress_timeout_);
    return true;
  }

  void onCommand(const JointCommand & message)
  {
    if (!command_output_enabled_) {
      return;
    }
    if (message.control_mode != JointCommand::CONTROL_MODE_POSITION) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "HC_X1 accepts position commands only");
      return;
    }
    SteadyTime expiry;
    if (!commandExpiry(message, expiry)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "rejected expired HC JointCommand");
      return;
    }

    const auto motion_group = motion_joint_names_.find(message.group_name);
    if (motion_group != motion_joint_names_.end()) {
      std::vector<double> ordered;
      std::string reason;
      if (!exactGroupCommand(message, motion_group->second, ordered, reason)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "rejected %s command: %s",
          message.group_name.c_str(), reason.c_str());
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (!safety_active_) {
        return;
      }
      auto & buffered = buffered_commands_[message.group_name];
      const bool progressed = buffered.source_id != message.source_id ||
        buffered.session_id != message.session_id || buffered.sequence != message.sequence;
      if (!progressed) {
        return;
      }
      buffered.names = motion_group->second;
      buffered.positions = std::move(ordered);
      buffered.source_id = message.source_id;
      buffered.session_id = message.session_id;
      buffered.sequence = message.sequence;
      buffered.expires_at = expiry;
      return;
    }

    if (message.group_name == left_finger_group_) {
      bufferFinger(
        message, left_finger_input_joint_, left_finger_open_, left_finger_closed_, expiry);
      return;
    }
    if (message.group_name == right_finger_group_) {
      bufferFinger(
        message, right_finger_input_joint_, right_finger_open_, right_finger_closed_, expiry);
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "ignored unsupported X1 command group: %s",
      message.group_name.c_str());
  }

  void bufferFinger(
    const JointCommand & message, const std::string & expected_joint,
    const double open, const double closed, const SteadyTime expiry)
  {
    if (message.command.name.size() != 1 || message.command.position.size() != 1 ||
      message.command.name.front() != expected_joint ||
      !std::isfinite(message.command.position.front()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejected %s command: expected one finite %s position",
        message.group_name.c_str(), expected_joint.c_str());
      return;
    }
    const auto fraction = std::clamp(
      (message.command.position.front() - open) / (closed - open), 0.0, 1.0);
    const auto & shape = message.group_name == left_finger_group_ ?
      left_finger_shape_ : right_finger_shape_;
    std::vector<double> positions(shape.names.size());
    for (std::size_t index = 0; index < positions.size(); ++index) {
      positions[index] = shape.open[index] + fraction * (shape.closed[index] - shape.open[index]);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!safety_active_) {
      return;
    }
    auto & buffered = buffered_commands_[message.group_name];
    const bool progressed = buffered.source_id != message.source_id ||
      buffered.session_id != message.session_id || buffered.sequence != message.sequence;
    if (!progressed) {
      return;
    }
    buffered.names = shape.names;
    buffered.positions = std::move(positions);
    buffered.source_id = message.source_id;
    buffered.session_id = message.session_id;
    buffered.sequence = message.sequence;
    buffered.expires_at = expiry;
  }

  void onCommandTimer()
  {
    if (!command_output_enabled_) {
      return;
    }
    sensor_msgs::msg::JointState motion;
    sensor_msgs::msg::JointState left_finger;
    sensor_msgs::msg::JointState right_finger;
    bool publish_motion = false;
    bool publish_left_finger = false;
    bool publish_right_finger = false;
    const auto steady_now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!safety_active_) {
        return;
      }
      for (auto iterator = buffered_commands_.begin();
        iterator != buffered_commands_.end();)
      {
        if (iterator->second.expires_at <= steady_now) {
          iterator = buffered_commands_.erase(iterator);
          continue;
        }
        const auto & group = iterator->first;
        const auto & command = iterator->second;
        if (motion_joint_names_.find(group) != motion_joint_names_.end()) {
          motion.name.insert(motion.name.end(), command.names.begin(), command.names.end());
          motion.position.insert(
            motion.position.end(), command.positions.begin(), command.positions.end());
          publish_motion = true;
        } else if (group == left_finger_group_) {
          left_finger.name = command.names;
          left_finger.position = command.positions;
          publish_left_finger = true;
        } else if (group == right_finger_group_) {
          right_finger.name = command.names;
          right_finger.position = command.positions;
          publish_right_finger = true;
        }
        ++iterator;
      }
    }
    const auto stamp = now();
    if (publish_motion) {
      motion.header.stamp = stamp;
      legacy_command_publisher_->publish(std::move(motion));
    }
    if (publish_left_finger) {
      left_finger.header.stamp = stamp;
      left_finger_publisher_->publish(std::move(left_finger));
    }
    if (publish_right_finger) {
      right_finger.header.stamp = stamp;
      right_finger_publisher_->publish(std::move(right_finger));
    }
  }

  void checkFeedback()
  {
    bool stale = false;
    bool seen = false;
    bool report_stale = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      seen = feedback_seen_;
      stale = seen && std::chrono::steady_clock::now() - last_feedback_ > feedback_warn_timeout_;
      report_stale = stale && !feedback_stale_warned_;
      if (report_stale) {
        feedback_stale_warned_ = true;
      }
    }
    if (!seen) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "waiting for HC_X1 feedback on /hc_teleop/joint_states");
    } else if (report_stale) {
      RCLCPP_WARN(
        get_logger(),
        "HC_X1 joint feedback timed out; motion backend will hold");
    }
  }

  bool command_output_enabled_{true};
  bool safety_active_{false};
  bool feedback_seen_{false};
  bool feedback_stale_warned_{false};
  std::chrono::nanoseconds command_progress_timeout_{};
  std::chrono::nanoseconds feedback_warn_timeout_{};
  SteadyTime last_feedback_{};
  std::mutex mutex_;
  std::vector<std::string> motion_group_names_;
  std::unordered_map<std::string, std::vector<std::string>> motion_joint_names_;
  std::unordered_map<std::string, BufferedCommand> buffered_commands_;
  std::string left_finger_group_;
  std::string right_finger_group_;
  std::string left_finger_input_joint_;
  std::string right_finger_input_joint_;
  double left_finger_open_{0.0};
  double left_finger_closed_{1.0};
  double right_finger_open_{0.0};
  double right_finger_closed_{1.0};
  FingerShape left_finger_shape_;
  FingerShape right_finger_shape_;
  rclcpp::Subscription<JointCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<SafetyState>::SharedPtr safety_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr legacy_state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr legacy_command_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_finger_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_finger_publisher_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

}  // namespace hc_adapter_x1

RCLCPP_COMPONENTS_REGISTER_NODE(hc_adapter_x1::X1HardwareAdapterNode)
