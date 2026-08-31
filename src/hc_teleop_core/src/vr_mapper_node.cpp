#include "hc_teleop_core/vr_mapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "hc_teleop_core/vr_mapper.hpp"
#include "hc_teleop_interfaces/msg/cartesian_state_array.hpp"
#include "hc_teleop_interfaces/msg/cartesian_target_array.hpp"
#include "hc_teleop_interfaces/msg/joint_command_candidate.hpp"
#include "hc_teleop_interfaces/msg/tracked_pose.hpp"
#include "hc_teleop_interfaces/msg/vr_frame.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hc_teleop_core
{
namespace
{

MapperPose pose(const geometry_msgs::msg::Pose & value)
{
  return {{value.position.x, value.position.y, value.position.z},
    {value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w}};
}

geometry_msgs::msg::Pose pose(const MapperPose & value)
{
  geometry_msgs::msg::Pose result;
  result.position.x = value.position[0];
  result.position.y = value.position[1];
  result.position.z = value.position[2];
  result.orientation.x = value.orientation[0];
  result.orientation.y = value.orientation[1];
  result.orientation.z = value.orientation[2];
  result.orientation.w = value.orientation[3];
  return result;
}

builtin_interfaces::msg::Time add_duration(const rclcpp::Time & value, std::chrono::nanoseconds delta)
{
  const std::int64_t nanoseconds = value.nanoseconds() + delta.count();
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return result;
}

std::chrono::milliseconds positive_milliseconds(double seconds, const char * name)
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive and finite");
  }
  return std::chrono::milliseconds(static_cast<std::int64_t>(std::llround(seconds * 1000.0)));
}

ControllerSide parse_controller_side(const std::string & value, const char * name)
{
  if (value == "left") {
    return ControllerSide::kLeft;
  }
  if (value == "right") {
    return ControllerSide::kRight;
  }
  throw std::invalid_argument(std::string(name) + " must be left or right");
}

bool sequence_newer(const std::uint32_t sequence, const std::uint32_t previous)
{
  const auto difference = sequence - previous;
  return difference != 0U && difference < 0x80000000U;
}

struct ToolBinding
{
  std::string group_name;
  ControllerSide controller{ControllerSide::kRight};
  std::vector<std::string> joint_names;
  std::vector<double> open_positions;
  std::vector<double> closed_positions;
};

}  // namespace

class VrMapperNode::Impl
{
public:
  explicit Impl(VrMapperNode & node)
  : node_(node), command_ttl_(positive_milliseconds(
        node_.declare_parameter<double>("command_ttl_sec", 0.1), "command_ttl_sec")),
    source_id_(node_.declare_parameter<std::string>("source_id", "vr")),
    mapper_(configuration())
  {
    if (source_id_.empty()) {
      throw std::invalid_argument("source_id must not be empty");
    }
    const auto vr_topic = node_.declare_parameter<std::string>("vr_topic", "input/vr_frame");
    const auto feedback_topic = node_.declare_parameter<std::string>(
      "cartesian_state_topic", "state/cartesian");
    const auto output_topic = node_.declare_parameter<std::string>(
      "target_topic", "teleop/cartesian_targets");
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = node_.create_publisher<hc_teleop_interfaces::msg::CartesianTargetArray>(
      output_topic, qos);
    const auto candidate_topic = node_.declare_parameter<std::string>(
      "candidate_topic", "control/joint_candidate");
    auto command_qos = rclcpp::SensorDataQoS().keep_last(16);
    tool_publisher_ = node_.create_publisher<hc_teleop_interfaces::msg::JointCommandCandidate>(
      candidate_topic, command_qos);
    feedback_subscription_ = node_.create_subscription<
      hc_teleop_interfaces::msg::CartesianStateArray>(
      feedback_topic, qos,
      [this](hc_teleop_interfaces::msg::CartesianStateArray::ConstSharedPtr message) {
        const auto received = std::chrono::steady_clock::now();
        for (const auto & state : message->states) {
          const auto frames = frames_.find(state.group_name);
          if (!state.valid || frames == frames_.end() ||
            state.reference_frame != frames->second.first || state.tip_frame != frames->second.second)
          {
            continue;
          }
          std::string reason;
          if (!mapper_.updateFeedback(state.group_name, pose(state.pose), received, reason)) {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
              "Rejected Cartesian feedback: %s", reason.c_str());
          }
        }
      });
    vr_subscription_ = node_.create_subscription<hc_teleop_interfaces::msg::VrFrame>(
      vr_topic, qos,
      [this](hc_teleop_interfaces::msg::VrFrame::ConstSharedPtr message) {
        if (!accept_frame(*message)) {
          return;
        }
        MapperFrame frame;
        frame.session_id = message->source_id;
        frame.sequence = message->sequence;
        frame.left = controller(
          message->left_controller, message->left_input.grip, message->left_input.trigger);
        frame.right = controller(
          message->right_controller, message->right_input.grip, message->right_input.trigger);
        const auto targets = mapper_.map(frame, std::chrono::steady_clock::now());
        const auto stamp = node_.now();
        if (!targets.empty()) {
          hc_teleop_interfaces::msg::CartesianTargetArray output;
          output.header.stamp = stamp;
          output.source_id = source_id_;
          output.session_id = message->source_id;
          output.sequence = message->sequence;
          output.valid_until = add_duration(stamp, command_ttl_);
          output.targets.reserve(targets.size());
          for (const auto & target : targets) {
            hc_teleop_interfaces::msg::CartesianTarget converted;
            converted.group_name = target.group_name;
            converted.reference_frame = target.reference_frame;
            converted.tip_frame = target.tip_frame;
            converted.pose_mode = hc_teleop_interfaces::msg::CartesianTarget::FULL_POSE;
            converted.pose = pose(target.pose);
            output.targets.push_back(std::move(converted));
          }
          publisher_->publish(std::move(output));
          ++published_;
        }
        publish_tools(*message, stamp);
      });
    RCLCPP_INFO(node_.get_logger(),
      "VR relative mapper ready; fresh measured FK is required before clutch engagement");
  }

  ~Impl()
  {
    RCLCPP_INFO(node_.get_logger(), "VR mapper stopped: published=%llu",
      static_cast<unsigned long long>(published_));
  }

private:
  MapperConfig configuration()
  {
    const auto groups = node_.declare_parameter<std::vector<std::string>>(
      "group_names", std::vector<std::string>{});
    const auto controllers = node_.declare_parameter<std::vector<std::string>>(
      "controllers", std::vector<std::string>{});
    const auto bases = node_.declare_parameter<std::vector<std::string>>(
      "base_frames", std::vector<std::string>{});
    const auto tips = node_.declare_parameter<std::vector<std::string>>(
      "tip_frames", std::vector<std::string>{});
    if (groups.empty() || groups.size() != controllers.size() || groups.size() != bases.size() ||
      groups.size() != tips.size())
    {
      throw std::invalid_argument(
              "group_names/controllers/base_frames/tip_frames must have equal non-zero lengths");
    }
    MapperConfig config;
    for (std::size_t index = 0; index < groups.size(); ++index) {
      ControllerSide side;
      if (controllers[index] == "left") {
        side = ControllerSide::kLeft;
      } else if (controllers[index] == "right") {
        side = ControllerSide::kRight;
      } else {
        throw std::invalid_argument("controllers entries must be left or right");
      }
      config.bindings.push_back(
          {groups[index], side, bases[index], tips[index], std::nullopt});
      frames_[groups[index]] = {bases[index], tips[index]};
    }
    const auto tool_groups = node_.declare_parameter<std::vector<std::string>>(
      "tool_group_names", std::vector<std::string>{});
    const auto tool_controllers = node_.declare_parameter<std::vector<std::string>>(
      "tool_controllers", std::vector<std::string>{});
    const auto tool_counts = node_.declare_parameter<std::vector<int64_t>>(
      "tool_joint_counts", std::vector<int64_t>{});
    const auto tool_joint_names = node_.declare_parameter<std::vector<std::string>>(
      "tool_joint_names", std::vector<std::string>{});
    const auto tool_open = node_.declare_parameter<std::vector<double>>(
      "tool_open_positions", std::vector<double>{});
    const auto tool_closed = node_.declare_parameter<std::vector<double>>(
      "tool_closed_positions", std::vector<double>{});
    if (tool_groups.size() != tool_controllers.size() || tool_groups.size() != tool_counts.size()) {
      throw std::invalid_argument(
              "tool_group_names/tool_controllers/tool_joint_counts must have equal lengths");
    }
    std::size_t tool_offset = 0U;
    if (tool_joint_names.size() != tool_open.size() || tool_joint_names.size() != tool_closed.size()) {
      throw std::invalid_argument(
              "tool_joint_names/tool_open_positions/tool_closed_positions must have equal lengths");
    }
    for (std::size_t index = 0; index < tool_groups.size(); ++index) {
      if (tool_counts[index] <= 0 || tool_offset + static_cast<std::size_t>(tool_counts[index]) >
        tool_joint_names.size())
      {
        throw std::invalid_argument("tool_joint_counts contains an invalid joint count");
      }
      ToolBinding tool;
      tool.group_name = tool_groups[index];
      tool.controller = parse_controller_side(tool_controllers[index], "tool_controllers");
      const auto count = static_cast<std::size_t>(tool_counts[index]);
      tool.joint_names.assign(tool_joint_names.begin() + tool_offset,
        tool_joint_names.begin() + tool_offset + count);
      tool.open_positions.assign(tool_open.begin() + tool_offset, tool_open.begin() + tool_offset + count);
      tool.closed_positions.assign(
        tool_closed.begin() + tool_offset, tool_closed.begin() + tool_offset + count);
      tools_.push_back(std::move(tool));
      tool_offset += count;
    }
    if (tool_offset != tool_joint_names.size()) {
      throw std::invalid_argument("tool_joint_counts does not cover all tool joints");
    }
    const auto mapping = node_.declare_parameter<std::vector<double>>(
      "axis_mapping", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    if (mapping.size() != config.axis_mapping.size()) {
      throw std::invalid_argument("axis_mapping must contain exactly 9 row-major values");
    }
    std::copy(mapping.begin(), mapping.end(), config.axis_mapping.begin());
    const auto binding_mappings = node_.declare_parameter<std::vector<double>>(
      "binding_axis_mappings", std::vector<double>{});
    if (!binding_mappings.empty() && binding_mappings.size() != groups.size() * 9U) {
      throw std::invalid_argument(
              "binding_axis_mappings must be empty or contain 9 values per arm binding");
    }
    for (std::size_t index = 0; index < groups.size() && !binding_mappings.empty(); ++index) {
      std::array<double, 9> per_binding{};
      std::copy_n(binding_mappings.begin() + index * 9U, 9U, per_binding.begin());
      config.bindings[index].axis_mapping = per_binding;
    }
    config.position_scale = node_.declare_parameter<double>("position_scale", 1.0);
    config.clutch_threshold = node_.declare_parameter<double>("clutch_threshold", 0.5);
    const auto clutch_controller = node_.declare_parameter<std::string>(
      "clutch_controller", "binding");
    if (clutch_controller != "binding") {
      config.clutch_controller = parse_controller_side(clutch_controller, "clutch_controller");
    }
    config.feedback_max_age = positive_milliseconds(
      node_.declare_parameter<double>("feedback_timeout_sec", 0.2), "feedback_timeout_sec");
    return config;
  }

  static ControllerFrame controller(
    const hc_teleop_interfaces::msg::TrackedPose & tracked, double grip, double trigger)
  {
    ControllerFrame result;
    result.tracked =
      tracked.tracking_state == hc_teleop_interfaces::msg::TrackedPose::TRACKING_TRACKED;
    result.grip = grip;
    result.trigger = trigger;
    result.pose = pose(tracked.pose);
    return result;
  }

  void publish_tools(
    const hc_teleop_interfaces::msg::VrFrame & message, const rclcpp::Time & stamp)
  {
    const auto & left = message.left_input;
    const auto & right = message.right_input;
    for (const auto & tool : tools_) {
      const auto & input = tool.controller == ControllerSide::kLeft ? left : right;
      const double value = std::clamp(static_cast<double>(input.trigger), 0.0, 1.0);
      hc_teleop_interfaces::msg::JointCommandCandidate candidate;
      candidate.header.stamp = stamp;
      candidate.source_id = source_id_;
      candidate.session_id = message.source_id;
      candidate.sequence = message.sequence;
      candidate.valid_until = add_duration(stamp, command_ttl_);
      candidate.control_mode = hc_teleop_interfaces::msg::JointCommandCandidate::CONTROL_MODE_POSITION;
      candidate.group_name = tool.group_name;
      candidate.command.name = tool.joint_names;
      candidate.command.position.resize(tool.joint_names.size());
      for (std::size_t index = 0; index < tool.joint_names.size(); ++index) {
        candidate.command.position[index] = tool.open_positions[index] + value *
          (tool.closed_positions[index] - tool.open_positions[index]);
      }
      tool_publisher_->publish(std::move(candidate));
    }
  }

  bool accept_frame(const hc_teleop_interfaces::msg::VrFrame & message)
  {
    if (message.source_id.empty()) {
      return false;
    }
    if (message.source_id != last_session_id_) {
      last_session_id_ = message.source_id;
      last_sequence_.reset();
    }
    if (last_sequence_ && !sequence_newer(message.sequence, *last_sequence_)) {
      return false;
    }
    last_sequence_ = message.sequence;
    return true;
  }

  VrMapperNode & node_;
  std::chrono::milliseconds command_ttl_;
  std::string source_id_;
  std::map<std::string, std::pair<std::string, std::string>> frames_;
  std::vector<ToolBinding> tools_;
  std::string last_session_id_;
  std::optional<std::uint32_t> last_sequence_;
  VrMapper mapper_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::CartesianTargetArray>::SharedPtr publisher_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::JointCommandCandidate>::SharedPtr tool_publisher_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::CartesianStateArray>::SharedPtr
    feedback_subscription_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::VrFrame>::SharedPtr vr_subscription_;
  std::uint64_t published_{0U};
};

VrMapperNode::VrMapperNode(const rclcpp::NodeOptions & options)
: Node("hc_vr_mapper", options), impl_(std::make_unique<Impl>(*this))
{
}

VrMapperNode::~VrMapperNode() = default;

}  // namespace hc_teleop_core

RCLCPP_COMPONENTS_REGISTER_NODE(hc_teleop_core::VrMapperNode)
