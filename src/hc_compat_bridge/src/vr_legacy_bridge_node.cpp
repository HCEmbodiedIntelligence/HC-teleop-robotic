#include "hc_compat_bridge/vr_legacy_bridge_node.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "hc_compat_bridge/vr_legacy_json.hpp"
#include "hc_teleop_interfaces/msg/joint_command.hpp"
#include "hc_teleop_interfaces/msg/vr_frame.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace hc_compat_bridge
{

class VrLegacyBridgeNode::Impl
{
public:
  explicit Impl(VrLegacyBridgeNode & node)
  : node_(node)
  {
    const auto input_topic = node_.declare_parameter<std::string>(
      "input_topic", "input/vr_frame");
    const auto output_topic = node_.declare_parameter<std::string>(
      "output_topic", "/vrdata");
    if (input_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }
    publisher_ = node_.create_publisher<std_msgs::msg::String>(
      output_topic, rclcpp::QoS(10));
    status_pub_ = node_.create_publisher<std_msgs::msg::String>(
      "/teleop/arm/status", rclcpp::QoS(10));
    subscription_ = node_.create_subscription<hc_teleop_interfaces::msg::VrFrame>(
      input_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](hc_teleop_interfaces::msg::VrFrame::ConstSharedPtr frame) {
        const auto payload = to_legacy_vr_json(*frame);
        if (!payload) {
          ++rejected_;
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 5000,
            "Rejected non-finite VrFrame in legacy compatibility bridge");
          return;
        }
        std_msgs::msg::String message;
        message.data = *payload;
        publisher_->publish(std::move(message));
        ++published_;

        // Publish teleop status for legacy dashboard
        const bool left_clutch = frame->left_input.grip >= 0.5;
        const bool right_clutch = frame->right_input.grip >= 0.5;
        std::ostringstream status_ss;
        status_ss << "{\"enabled\":true,\"backend\":\"v23\",\"generic_controller\":{\"solver_fresh\":true,\"command_fresh\":true},\"mode\":\"arms_grippers\",\"left_clutch\":"
                  << (left_clutch ? "true" : "false")
                  << ",\"right_clutch\":"
                  << (right_clutch ? "true" : "false")
                  << ",\"feedback_fresh\":true}";
        std_msgs::msg::String status_msg;
        status_msg.data = status_ss.str();
        status_pub_->publish(std::move(status_msg));
      });

    const auto joint_states_in = node_.declare_parameter<std::string>(
      "joint_states_input_topic", "state/joints");
    const auto joint_states_out = node_.declare_parameter<std::string>(
      "joint_states_output_topic", "/hc_teleop/joint_states");
    if (!joint_states_in.empty() && !joint_states_out.empty()) {
      joint_states_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>(
        joint_states_out, rclcpp::QoS(10));
      joint_states_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
        joint_states_in, rclcpp::QoS(10),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          joint_states_pub_->publish(*msg);
          const auto now_steady = std::chrono::steady_clock::now();
          // If no active command received in the last 300ms, sync command to current feedback
          if (now_steady - last_active_cmd_ > std::chrono::milliseconds(300)) {
            for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
              latest_positions_[msg->name[i]] = msg->position[i];
            }
          }
          publish_merged_command(msg->header);
        });
    }

    const auto joint_cmd_in = node_.declare_parameter<std::string>(
      "joint_cmd_input_topic", "control/joint_command");
    const auto joint_cmd_out = node_.declare_parameter<std::string>(
      "joint_cmd_output_topic", "/hc_teleop/joint_cmd");
    if (!joint_cmd_in.empty() && !joint_cmd_out.empty()) {
      joint_cmd_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>(
        joint_cmd_out, rclcpp::QoS(10));
      joint_cmd_sub_ = node_.create_subscription<hc_teleop_interfaces::msg::JointCommand>(
        joint_cmd_in, rclcpp::SensorDataQoS().keep_last(1),
        [this](hc_teleop_interfaces::msg::JointCommand::ConstSharedPtr msg) {
          last_active_cmd_ = std::chrono::steady_clock::now();
          for (std::size_t i = 0; i < msg->command.name.size() && i < msg->command.position.size(); ++i) {
            latest_positions_[msg->command.name[i]] = msg->command.position[i];
          }
          publish_merged_command(msg->header);
        });
    }
  }

  ~Impl() = default;

private:
  void publish_merged_command(const std_msgs::msg::Header & header)
  {
    if (!joint_cmd_pub_ || latest_positions_.empty()) {
      return;
    }
    sensor_msgs::msg::JointState js;
    js.header = header;
    js.name.reserve(latest_positions_.size());
    js.position.reserve(latest_positions_.size());
    for (const auto & [name, pos] : latest_positions_) {
      js.name.push_back(name);
      js.position.push_back(pos);
    }
    joint_cmd_pub_->publish(std::move(js));
  }

  VrLegacyBridgeNode & node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::VrFrame>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::JointCommand>::SharedPtr joint_cmd_sub_;
  std::map<std::string, double> latest_positions_;
  std::chrono::steady_clock::time_point last_active_cmd_{};
  std::uint64_t published_{0U};
  std::uint64_t rejected_{0U};
};

VrLegacyBridgeNode::VrLegacyBridgeNode(const rclcpp::NodeOptions & options)
: Node("hc_vr_legacy_bridge", options), impl_(std::make_unique<Impl>(*this))
{
}

VrLegacyBridgeNode::~VrLegacyBridgeNode() = default;

}  // namespace hc_compat_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(hc_compat_bridge::VrLegacyBridgeNode)
