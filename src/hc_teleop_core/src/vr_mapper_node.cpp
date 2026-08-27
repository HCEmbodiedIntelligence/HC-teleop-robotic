#include "hc_teleop_core/vr_mapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "hc_teleop_core/vr_mapper.hpp"
#include "hc_teleop_interfaces/msg/cartesian_state_array.hpp"
#include "hc_teleop_interfaces/msg/cartesian_target_array.hpp"
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
        MapperFrame frame;
        frame.session_id = message->source_id;
        frame.sequence = message->sequence;
        frame.left = controller(message->left_controller, message->left_input.grip);
        frame.right = controller(message->right_controller, message->right_input.grip);
        const auto targets = mapper_.map(frame, std::chrono::steady_clock::now());
        if (targets.empty()) {
          return;
        }
        hc_teleop_interfaces::msg::CartesianTargetArray output;
        output.header.stamp = node_.now();
        output.source_id = source_id_;
        output.session_id = message->source_id;
        output.sequence = message->sequence;
        output.valid_until = add_duration(output.header.stamp, command_ttl_);
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
      config.bindings.push_back({groups[index], side, bases[index], tips[index]});
      frames_[groups[index]] = {bases[index], tips[index]};
    }
    const auto mapping = node_.declare_parameter<std::vector<double>>(
      "axis_mapping", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    if (mapping.size() != config.axis_mapping.size()) {
      throw std::invalid_argument("axis_mapping must contain exactly 9 row-major values");
    }
    std::copy(mapping.begin(), mapping.end(), config.axis_mapping.begin());
    config.position_scale = node_.declare_parameter<double>("position_scale", 1.0);
    config.clutch_threshold = node_.declare_parameter<double>("clutch_threshold", 0.5);
    config.feedback_max_age = positive_milliseconds(
      node_.declare_parameter<double>("feedback_timeout_sec", 0.2), "feedback_timeout_sec");
    return config;
  }

  static ControllerFrame controller(
    const hc_teleop_interfaces::msg::TrackedPose & tracked, double grip)
  {
    ControllerFrame result;
    result.tracked =
      tracked.tracking_state == hc_teleop_interfaces::msg::TrackedPose::TRACKING_TRACKED;
    result.grip = grip;
    result.pose = pose(tracked.pose);
    return result;
  }

  VrMapperNode & node_;
  std::chrono::milliseconds command_ttl_;
  std::string source_id_;
  std::map<std::string, std::pair<std::string, std::string>> frames_;
  VrMapper mapper_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::CartesianTargetArray>::SharedPtr publisher_;
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
