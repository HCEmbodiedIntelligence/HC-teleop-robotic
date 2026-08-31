#include "hc_adapter_openarmx/sim_joint_model.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hc_teleop_interfaces/msg/cartesian_state_array.hpp"
#include "hc_teleop_interfaces/msg/joint_command.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"

namespace hc_adapter_openarmx
{
namespace
{

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

struct KinematicsGroup
{
  std::string group_name;
  std::string reference_frame;
  std::string tip_frame;
  std::vector<std::string> joint_names;
  std::shared_ptr<KDL::Chain> chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
};

geometry_msgs::msg::Pose poseFromFrame(const KDL::Frame & frame)
{
  geometry_msgs::msg::Pose result;
  result.position.x = frame.p.x();
  result.position.y = frame.p.y();
  result.position.z = frame.p.z();
  frame.M.GetQuaternion(
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w);
  return result;
}

}  // namespace

class OpenArmXSimNode final : public rclcpp::Node
{
public:
  explicit OpenArmXSimNode(const rclcpp::NodeOptions & options)
  : Node("openarmx_sim_adapter", options)
  {
    const auto urdf_path = declare_parameter<std::string>("urdf_path", "");
    if (urdf_path.empty()) {
      throw std::invalid_argument("urdf_path must be supplied");
    }

    const auto group_names = declare_parameter<std::vector<std::string>>(
      "group_names", std::vector<std::string>{});
    if (group_names.empty()) {
      throw std::invalid_argument("group_names must include at least one arm group");
    }

    const auto velocity_scale = positiveFinite(
      declare_parameter<double>("max_velocity_scale", 0.2), "max_velocity_scale");
    const auto fallback_velocity = positiveFinite(
      declare_parameter<double>("fallback_max_velocity", 1.0), "fallback_max_velocity");

    urdf::Model robot_model;
    if (!robot_model.initFile(urdf_path)) {
      throw std::runtime_error("unable to parse URDF: " + urdf_path);
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
      throw std::runtime_error("unable to build KDL tree from URDF: " + urdf_path);
    }

    std::vector<JointGroupConfig> groups;
    groups.reserve(group_names.size() + 1);
    std::unordered_set<std::string> configured_joints;
    for (const auto & group_name : group_names) {
      groups.push_back(loadGroup(group_name, robot_model, tree, velocity_scale, fallback_velocity));
      for (const auto & jname : groups.back().joint_names) {
        configured_joints.insert(jname);
      }
    }

    JointGroupConfig auxiliary;
    auxiliary.group_name = "auxiliary_joints";
    for (const auto & entry : robot_model.joints_) {
      const auto & joint_name = entry.first;
      const auto & joint_ptr = entry.second;
      if (joint_ptr && (joint_ptr->type == urdf::Joint::REVOLUTE ||
                        joint_ptr->type == urdf::Joint::CONTINUOUS ||
                        joint_ptr->type == urdf::Joint::PRISMATIC))
      {
        if (configured_joints.find(joint_name) == configured_joints.end()) {
          auxiliary.joint_names.push_back(joint_name);
          double lower = -3.14159;
          double upper = 3.14159;
          double vel = fallback_velocity;
          if (joint_ptr->limits) {
            lower = joint_ptr->limits->lower;
            upper = joint_ptr->limits->upper;
            if (joint_ptr->limits->velocity > 0.0) {
              vel = joint_ptr->limits->velocity * velocity_scale;
            }
          }
          auxiliary.lower_limits.push_back(lower);
          auxiliary.upper_limits.push_back(upper);
          auxiliary.max_velocity.push_back(vel);
        }
      }
    }
    if (!auxiliary.joint_names.empty()) {
      groups.push_back(std::move(auxiliary));
    }
    model_ = std::make_unique<SimJointModel>(std::move(groups));

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    auto command_qos = rclcpp::SensorDataQoS().keep_last(16);
    command_subscription_ =
      create_subscription<hc_teleop_interfaces::msg::JointCommand>(
      declare_parameter<std::string>("command_topic", "control/joint_command"), command_qos,
      [this](hc_teleop_interfaces::msg::JointCommand::ConstSharedPtr message) {
        onCommand(*message);
      });
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "state/joints"), rclcpp::QoS(10));
    cartesian_publisher_ =
      create_publisher<hc_teleop_interfaces::msg::CartesianStateArray>(
      declare_parameter<std::string>("cartesian_state_topic", "state/cartesian"), qos);

    const auto publish_rate_hz = positiveFinite(
      declare_parameter<double>("publish_rate_hz", 100.0), "publish_rate_hz");
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() { onTimer(); });
    last_update_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(), "OpenArmX sim adapter ready: groups=%zu joints=%zu urdf=%s",
      kinematics_.size(), model_->jointNames().size(), urdf_path.c_str());
  }

private:
  JointGroupConfig loadGroup(
    const std::string & group_name,
    const urdf::Model & robot_model,
    const KDL::Tree & tree,
    const double velocity_scale,
    const double fallback_velocity)
  {
    JointGroupConfig config;
    config.group_name = group_name;
    config.joint_names = declare_parameter<std::vector<std::string>>(
      group_name + ".joint_names", std::vector<std::string>{});
    config.reference_frame = declare_parameter<std::string>(group_name + ".base_frame", "");
    config.tip_frame = declare_parameter<std::string>(group_name + ".tip_frame", "");
    if (config.reference_frame.empty() || config.tip_frame.empty()) {
      throw std::invalid_argument(group_name + " requires base_frame and tip_frame");
    }

    for (const auto & joint_name : config.joint_names) {
      const auto joint = robot_model.getJoint(joint_name);
      if (!joint || !joint->limits) {
        throw std::invalid_argument("URDF has no limits for joint: " + joint_name);
      }
      config.lower_limits.push_back(joint->limits->lower);
      config.upper_limits.push_back(joint->limits->upper);
      const auto velocity = joint->limits->velocity > 0.0 ?
        joint->limits->velocity * velocity_scale : fallback_velocity;
      config.max_velocity.push_back(velocity);
    }

    auto chain = std::make_shared<KDL::Chain>();
    if (!tree.getChain(config.reference_frame, config.tip_frame, *chain)) {
      throw std::invalid_argument(
              "unable to build KDL chain from " + config.reference_frame + " to " +
              config.tip_frame);
    }
    if (chain->getNrOfJoints() != config.joint_names.size()) {
      throw std::invalid_argument(
              group_name + " KDL joint count does not match configured joint_names");
    }

    KinematicsGroup kinematics;
    kinematics.group_name = group_name;
    kinematics.reference_frame = config.reference_frame;
    kinematics.tip_frame = config.tip_frame;
    kinematics.joint_names = config.joint_names;
    kinematics.chain = std::move(chain);
    kinematics.fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(*kinematics.chain);
    kinematics_.emplace(group_name, std::move(kinematics));
    return config;
  }

  void onCommand(const hc_teleop_interfaces::msg::JointCommand & message)
  {
    if (message.control_mode != hc_teleop_interfaces::msg::JointCommand::CONTROL_MODE_POSITION) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "OpenArmX sim only accepts position commands");
      return;
    }

    const auto now_ros = now();
    const auto remaining_ns = timeNanoseconds(message.valid_until) - now_ros.nanoseconds();
    if (remaining_ns <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ignored expired JointCommand");
      return;
    }

    std::string reason;
    const auto now_steady = std::chrono::steady_clock::now();
    if (!model_->acceptCommand(
        message.group_name, message.command.name, message.command.position, now_steady,
        now_steady + std::chrono::nanoseconds(remaining_ns), reason))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ignored JointCommand for %s: %s",
        message.group_name.c_str(), reason.c_str());
      return;
    }
    ++accepted_commands_;
  }

  void onTimer()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto dt = std::chrono::duration<double>(steady_now - last_update_).count();
    last_update_ = steady_now;
    model_->step(dt, steady_now);
    publishJointState();
    publishCartesianState();
  }

  void publishJointState()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = model_->jointNames();
    message.position = model_->positions();
    message.velocity.assign(message.name.size(), 0.0);
    message.effort.assign(message.name.size(), 0.0);
    joint_state_publisher_->publish(std::move(message));
  }

  void publishCartesianState()
  {
    hc_teleop_interfaces::msg::CartesianStateArray message;
    message.header.stamp = now();
    message.states.reserve(kinematics_.size());
    for (const auto & entry : kinematics_) {
      const auto & kinematics = entry.second;
      hc_teleop_interfaces::msg::CartesianState state;
      state.group_name = kinematics.group_name;
      state.reference_frame = kinematics.reference_frame;
      state.tip_frame = kinematics.tip_frame;

      const auto positions = model_->groupPositions(kinematics.group_name);
      KDL::JntArray q(positions.size());
      for (std::size_t index = 0; index < positions.size(); ++index) {
        q(index) = positions[index];
      }
      KDL::Frame frame;
      state.valid = kinematics.fk->JntToCart(q, frame) >= 0;
      if (state.valid) {
        state.pose = poseFromFrame(frame);
      }
      message.states.push_back(std::move(state));
    }
    cartesian_publisher_->publish(std::move(message));
  }

  std::unique_ptr<SimJointModel> model_;
  std::map<std::string, KinematicsGroup> kinematics_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::JointCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::CartesianStateArray>::SharedPtr cartesian_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_update_{};
  std::uint64_t accepted_commands_{0U};
};

}  // namespace hc_adapter_openarmx

RCLCPP_COMPONENTS_REGISTER_NODE(hc_adapter_openarmx::OpenArmXSimNode)
