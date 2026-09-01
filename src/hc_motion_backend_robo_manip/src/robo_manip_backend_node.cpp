#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hc_teleop_interfaces/msg/cartesian_state_array.hpp"
#include "hc_teleop_interfaces/msg/cartesian_target.hpp"
#include "hc_teleop_interfaces/msg/cartesian_target_array.hpp"
#include "hc_teleop_interfaces/msg/joint_command_candidate.hpp"
#include "humanoid_motion_server/kinematics/kinematics.hpp"
#include "humanoid_motion_server/motion/sdk_motion_backend.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
#include "yaml-cpp/yaml.h"

namespace hc_motion_backend_robo_manip
{
namespace
{

using Backend = humanoid_motion_server::motion::ISdkMotionBackend;
using DynamicTarget = humanoid_motion_server::motion::DynamicTarget;
using ForwardKinematicsRequest = humanoid_motion_server::motion::ForwardKinematicsRequest;
using JointCommand = humanoid_motion_server::motion::JointCommand;
using JointFeedback = humanoid_motion_server::motion::JointFeedback;
using JointGroupModel = humanoid_motion_server::motion::JointGroupModel;
using MotionLimits = humanoid_motion_server::motion::MotionLimits;
using Pose = humanoid_motion_server::motion::Pose;
using ServoPRequest = humanoid_motion_server::motion::ServoPRequest;
namespace sdk_kinematics = humanoid_motion_server::kinematics;
using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

struct SdkIkTuning
{
  double position_tolerance_mm{1.0};
  double orientation_tolerance_rad{0.015};
  bool enable_regularization_task{true};
  double regularization_task_weight{0.0005};
  bool enable_joint_task{true};
  double joint_task_weight{0.001};
};

struct TargetFilterConfig
{
  bool enabled{false};
  std::array<double, 3> workspace_min_m{{0.0, 0.0, 0.0}};
  std::array<double, 3> workspace_max_m{{0.0, 0.0, 0.0}};
  double min_radius_m{0.0};
  double max_radius_m{0.0};
  double max_position_step_m{0.0};
  double max_orientation_step_rad{0.0};
};

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

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const double norm_sq =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && norm_sq > 1e-12;
}

Pose toCorePose(const geometry_msgs::msg::Pose & input)
{
  const double norm = std::sqrt(
    input.orientation.x * input.orientation.x +
    input.orientation.y * input.orientation.y +
    input.orientation.z * input.orientation.z +
    input.orientation.w * input.orientation.w);
  Pose output;
  output.position_m = {{input.position.x, input.position.y, input.position.z}};
  output.orientation_xyzw = {{
    input.orientation.x / norm, input.orientation.y / norm,
    input.orientation.z / norm, input.orientation.w / norm}};
  return output;
}

geometry_msgs::msg::Pose toRosPose(const Pose & input)
{
  geometry_msgs::msg::Pose output;
  output.position.x = input.position_m[0];
  output.position.y = input.position_m[1];
  output.position.z = input.position_m[2];
  output.orientation.x = input.orientation_xyzw[0];
  output.orientation.y = input.orientation_xyzw[1];
  output.orientation.z = input.orientation_xyzw[2];
  output.orientation.w = input.orientation_xyzw[3];
  return output;
}

class RuntimeSdkConfig
{
public:
  RuntimeSdkConfig(
    const std::string & source_path, const std::string & model_path,
    const MotionLimits & final_limits, const SdkIkTuning & ik_tuning)
  {
    if (!std::filesystem::is_regular_file(source_path)) {
      throw std::runtime_error("SDK configuration does not exist: " + source_path);
    }
    const auto absolute_model = std::filesystem::absolute(model_path).lexically_normal();
    if (!std::filesystem::is_regular_file(absolute_model)) {
      throw std::runtime_error("SDK URDF model does not exist: " + absolute_model.string());
    }

    YAML::Node document = YAML::LoadFile(source_path);
    if (!document.IsMap()) {
      throw std::runtime_error("SDK configuration root must be a mapping");
    }
    document["model_path"] = absolute_model.string();

    // Apply the HC profile's ServoP IK tuning to the SDK context itself.  A
    // seven-DOF arm needs a small null-space preference; the SDK's original
    // 1e-3 rad orientation threshold was too strict for noisy VR targets.
    auto placo = document["planning"]["ik"]["placo"];
    placo["position_tolerance"] = ik_tuning.position_tolerance_mm;
    placo["orientation_tolerance"] = ik_tuning.orientation_tolerance_rad;
    placo["enable_regularization_task"] = ik_tuning.enable_regularization_task;
    placo["regularization_task_weight"] = ik_tuning.regularization_task_weight;
    placo["enable_joint_task"] = ik_tuning.enable_joint_task;
    placo["joint_task_weight"] = ik_tuning.joint_task_weight;

    // The SDK creates the mandatory final joint RTC from its YAML rather than
    // from each ServoP request. Keep that final safety envelope identical to
    // the mode-selected HC limits; otherwise the static commissioning YAML
    // silently caps simulation at 0.1 m/s and looks like transport latency.
    auto rtc = document["rtc"];
    rtc["max_velocity"] = final_limits.joint_max_velocity_rad_s.at(0) *
      kRadiansToDegrees;
    rtc["max_acceleration"] = final_limits.joint_max_acceleration_rad_s2.at(0) *
      kRadiansToDegrees;
    rtc["max_jerk"] = final_limits.joint_max_jerk_rad_s3.at(0) * kRadiansToDegrees;
    rtc["cartesian_max_linear_velocity"] =
      final_limits.cartesian_max_linear_velocity_m_s * 1000.0;
    rtc["cartesian_max_linear_acceleration"] =
      final_limits.cartesian_max_linear_acceleration_m_s2 * 1000.0;
    rtc["cartesian_max_linear_jerk"] =
      final_limits.cartesian_max_linear_jerk_m_s3 * 1000.0;
    rtc["cartesian_max_angular_velocity"] =
      final_limits.cartesian_max_angular_velocity_rad_s;
    rtc["cartesian_max_angular_acceleration"] =
      final_limits.cartesian_max_angular_acceleration_rad_s2;
    rtc["cartesian_max_angular_jerk"] =
      final_limits.cartesian_max_angular_jerk_rad_s3;
    YAML::Emitter emitter;
    emitter << document;
    if (!emitter.good()) {
      throw std::runtime_error("failed to serialize runtime SDK configuration");
    }

    auto path_template =
      (std::filesystem::temp_directory_path() / "hc_robo_manip_sdk_XXXXXX").string();
    std::vector<char> path_buffer(path_template.begin(), path_template.end());
    path_buffer.push_back('\0');
    const int descriptor = mkstemp(path_buffer.data());
    if (descriptor < 0) {
      throw std::runtime_error(
              "failed to create runtime SDK configuration: " +
              std::string(std::strerror(errno)));
    }
    path_ = path_buffer.data();

    const std::string payload = emitter.c_str();
    std::size_t written = 0U;
    while (written < payload.size()) {
      const auto count = write(descriptor, payload.data() + written, payload.size() - written);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        const auto error = std::string(std::strerror(errno));
        close(descriptor);
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
        path_.clear();
        throw std::runtime_error("failed to write runtime SDK configuration: " + error);
      }
      written += static_cast<std::size_t>(count);
    }
    if (close(descriptor) != 0) {
      const auto error = std::string(std::strerror(errno));
      std::error_code ignored;
      std::filesystem::remove(path_, ignored);
      path_.clear();
      throw std::runtime_error("failed to close runtime SDK configuration: " + error);
    }
  }

  ~RuntimeSdkConfig()
  {
    if (!path_.empty()) {
      std::error_code ignored;
      std::filesystem::remove(path_, ignored);
    }
  }

  RuntimeSdkConfig(const RuntimeSdkConfig &) = delete;
  RuntimeSdkConfig & operator=(const RuntimeSdkConfig &) = delete;

  const std::string & path() const {return path_;}

private:
  std::string path_;
};

struct Group
{
  std::string name;
  std::string base_frame;
  std::string tip_frame;
  std::vector<std::string> joint_names;
  JointGroupModel model;
  bool active{false};
  std::string source_id;
  std::string input_session_id;
  std::string sdk_session_id;
  std::optional<std::uint64_t> last_published_sequence;
  std::optional<JointCommand> last_safe_candidate;
  std::optional<Pose> last_accepted_target;
  TargetFilterConfig target_filter;
  std::size_t consecutive_tick_failures{0U};
  SteadyTime last_target_at{};
  SteadyTime last_step_at{};
};

}  // namespace

class RoboManipBackendNode final : public rclcpp::Node
{
public:
  RoboManipBackendNode()
  : Node("robo_manip_backend")
  {
    const auto sdk_config_path = declare_parameter<std::string>("sdk_config_path", "");
    const auto urdf_path = declare_parameter<std::string>("urdf_path", "");
    if (sdk_config_path.empty() || urdf_path.empty()) {
      throw std::invalid_argument("sdk_config_path and urdf_path must be supplied");
    }

    nominal_period_sec_ = 1.0 / positiveFinite(
      declare_parameter<double>("nominal_rate_hz", 60.0), "nominal_rate_hz");
    feedback_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(positiveFinite(
        declare_parameter<double>("feedback_timeout_sec", 0.2), "feedback_timeout_sec")));
    session_reset_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(positiveFinite(
        declare_parameter<double>("session_reset_timeout_sec", 0.25),
        "session_reset_timeout_sec")));
    const auto tick_failure_reset_count = declare_parameter<int>(
      "tick_failure_reset_count", 3);
    if (tick_failure_reset_count < 2) {
      throw std::invalid_argument("tick_failure_reset_count must be at least 2");
    }
    tick_failure_reset_count_ = static_cast<std::size_t>(tick_failure_reset_count);
    publish_fk_ = declare_parameter<bool>("publish_fk", true);

    ik_tuning_.position_tolerance_mm = positiveFinite(
      declare_parameter<double>("ik_position_tolerance_m", 0.001) * 1000.0,
      "ik_position_tolerance_m");
    ik_tuning_.orientation_tolerance_rad = positiveFinite(
      declare_parameter<double>("ik_orientation_tolerance_rad", 0.015),
      "ik_orientation_tolerance_rad");
    ik_tuning_.enable_regularization_task =
      declare_parameter<bool>("ik_enable_regularization_task", true);
    ik_tuning_.regularization_task_weight = positiveFinite(
      declare_parameter<double>("ik_regularization_task_weight", 0.0005),
      "ik_regularization_task_weight");
    ik_tuning_.enable_joint_task = declare_parameter<bool>("ik_enable_joint_task", true);
    ik_tuning_.joint_task_weight = positiveFinite(
      declare_parameter<double>("ik_joint_task_weight", 0.001),
      "ik_joint_task_weight");

    joint_max_velocity_ = positiveFinite(
      declare_parameter<double>("joint_max_velocity_rad_s", 1.0),
      "joint_max_velocity_rad_s");
    joint_max_acceleration_ = positiveFinite(
      declare_parameter<double>("joint_max_acceleration_rad_s2", 2.0),
      "joint_max_acceleration_rad_s2");
    joint_max_jerk_ = positiveFinite(
      declare_parameter<double>("joint_max_jerk_rad_s3", 10.0),
      "joint_max_jerk_rad_s3");
    cartesian_max_linear_velocity_ = positiveFinite(
      declare_parameter<double>("cartesian_max_linear_velocity_m_s", 0.3),
      "cartesian_max_linear_velocity_m_s");
    cartesian_max_linear_acceleration_ = positiveFinite(
      declare_parameter<double>("cartesian_max_linear_acceleration_m_s2", 0.6),
      "cartesian_max_linear_acceleration_m_s2");
    cartesian_max_linear_jerk_ = positiveFinite(
      declare_parameter<double>("cartesian_max_linear_jerk_m_s3", 3.0),
      "cartesian_max_linear_jerk_m_s3");
    cartesian_max_angular_velocity_ = positiveFinite(
      declare_parameter<double>("cartesian_max_angular_velocity_rad_s", 1.0),
      "cartesian_max_angular_velocity_rad_s");
    cartesian_max_angular_acceleration_ = positiveFinite(
      declare_parameter<double>("cartesian_max_angular_acceleration_rad_s2", 2.0),
      "cartesian_max_angular_acceleration_rad_s2");
    cartesian_max_angular_jerk_ = positiveFinite(
      declare_parameter<double>("cartesian_max_angular_jerk_rad_s3", 10.0),
      "cartesian_max_angular_jerk_rad_s3");

    urdf::Model robot_model;
    if (!robot_model.initFile(urdf_path)) {
      throw std::runtime_error("unable to parse URDF: " + urdf_path);
    }
    loadGroups(robot_model);

    runtime_sdk_config_ = std::make_unique<RuntimeSdkConfig>(
      sdk_config_path, urdf_path, limits(1U), ik_tuning_);
    humanoid_motion_server::motion::MotionContextFactoryOptions factory_options;
    for (const auto & entry : groups_) {
      factory_options.joint_groups.push_back(entry.second.model);
    }
    diagnostic_kinematics_ = sdk_kinematics::SdkKinematics::create();
    const auto kinematics_status = diagnostic_kinematics_->load(runtime_sdk_config_->path());
    if (!kinematics_status.ok()) {
      throw std::runtime_error(
              "unable to create RoboManip diagnostic kinematics: " +
              kinematics_status.message);
    }
    factory_options.kinematics = diagnostic_kinematics_;
    auto factory = humanoid_motion_server::motion::MotionContextFactory::createFromSdkYaml(
      runtime_sdk_config_->path(), factory_options);
    if (!factory.status.ok() || !factory.backend) {
      throw std::runtime_error("unable to create RoboManip motion context: " +
              factory.status.message);
    }
    backend_ = std::move(factory.backend);

    auto stream_qos = rclcpp::SensorDataQoS().keep_last(1);
    target_subscription_ = create_subscription<
      hc_teleop_interfaces::msg::CartesianTargetArray>(
      declare_parameter<std::string>(
        "target_topic", "motion/backend/cartesian_targets"), stream_qos,
      [this](hc_teleop_interfaces::msg::CartesianTargetArray::ConstSharedPtr message) {
        onTarget(*message);
      });
    feedback_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "state/joints"), stream_qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        onFeedback(*message);
      });
    candidate_publisher_ = create_publisher<
      hc_teleop_interfaces::msg::JointCommandCandidate>(
      declare_parameter<std::string>(
        "candidate_topic", "motion/backend/joint_candidate"),
      rclcpp::SensorDataQoS().keep_last(16));
    if (publish_fk_) {
      cartesian_publisher_ = create_publisher<
        hc_teleop_interfaces::msg::CartesianStateArray>(
        declare_parameter<std::string>("cartesian_state_topic", "state/cartesian"),
        stream_qos);
    }
    watchdog_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {expireSessions();});

    RCLCPP_INFO(
      get_logger(),
      "RoboManip backend ready: groups=%zu nominal_rate=%.1fHz measured_fk=%s "
      "limits[joint=%.2frad/s cart=%.2fm/s] ik[orientation=%.3frad "
      "regularization=%s joint_task=%s] tick_reset=%zu sdk=%s",
      groups_.size(), 1.0 / nominal_period_sec_, publish_fk_ ? "on" : "off",
      joint_max_velocity_, cartesian_max_linear_velocity_,
      ik_tuning_.orientation_tolerance_rad,
      ik_tuning_.enable_regularization_task ? "on" : "off",
      ik_tuning_.enable_joint_task ? "on" : "off", tick_failure_reset_count_,
      sdk_config_path.c_str());
  }

  ~RoboManipBackendNode() override
  {
    for (auto & entry : groups_) {
      stop(entry.second);
    }
  }

private:
  void loadGroups(const urdf::Model & robot_model)
  {
    const auto group_names = declare_parameter<std::vector<std::string>>(
      "group_names", std::vector<std::string>{});
    if (group_names.empty()) {
      throw std::invalid_argument("group_names must include at least one arm group");
    }
    for (const auto & group_name : group_names) {
      if (group_name.empty() || groups_.count(group_name) != 0U) {
        throw std::invalid_argument("group names must be non-empty and unique");
      }
      Group group;
      group.name = group_name;
      group.base_frame = declare_parameter<std::string>(group_name + ".base_frame", "");
      group.tip_frame = declare_parameter<std::string>(group_name + ".tip_frame", "");
      group.joint_names = declare_parameter<std::vector<std::string>>(
        group_name + ".joint_names", std::vector<std::string>{});
      if (group.base_frame.empty() || group.tip_frame.empty() || group.joint_names.empty()) {
        throw std::invalid_argument(
                group_name + " requires joint_names/base_frame/tip_frame");
      }
      group.target_filter.enabled = declare_parameter<bool>(
        group_name + ".target_filter.enabled", false);
      if (group.target_filter.enabled) {
        const auto workspace_min = declare_parameter<std::vector<double>>(
          group_name + ".target_filter.workspace_min_m", std::vector<double>{});
        const auto workspace_max = declare_parameter<std::vector<double>>(
          group_name + ".target_filter.workspace_max_m", std::vector<double>{});
        if (workspace_min.size() != 3U || workspace_max.size() != 3U) {
          throw std::invalid_argument(
                  group_name + " target-filter workspace bounds must contain three values");
        }
        for (std::size_t index = 0; index < 3U; ++index) {
          if (!std::isfinite(workspace_min[index]) ||
            !std::isfinite(workspace_max[index]) ||
            workspace_min[index] >= workspace_max[index])
          {
            throw std::invalid_argument(
                    group_name + " target-filter workspace bounds are invalid");
          }
          group.target_filter.workspace_min_m[index] = workspace_min[index];
          group.target_filter.workspace_max_m[index] = workspace_max[index];
        }
        group.target_filter.min_radius_m = positiveFinite(
          declare_parameter<double>(
            group_name + ".target_filter.min_radius_m", 0.1),
          (group_name + ".target_filter.min_radius_m").c_str());
        group.target_filter.max_radius_m = positiveFinite(
          declare_parameter<double>(
            group_name + ".target_filter.max_radius_m", 1.0),
          (group_name + ".target_filter.max_radius_m").c_str());
        if (group.target_filter.min_radius_m >= group.target_filter.max_radius_m) {
          throw std::invalid_argument(
                  group_name + " target-filter radii must be increasing");
        }
        group.target_filter.max_position_step_m = positiveFinite(
          declare_parameter<double>(
            group_name + ".target_filter.max_position_step_m", 0.03),
          (group_name + ".target_filter.max_position_step_m").c_str());
        group.target_filter.max_orientation_step_rad = positiveFinite(
          declare_parameter<double>(
            group_name + ".target_filter.max_orientation_step_rad", 0.15),
          (group_name + ".target_filter.max_orientation_step_rad").c_str());
        RCLCPP_INFO(
          get_logger(),
          "%s target filter: xyz=[%.2f..%.2f, %.2f..%.2f, %.2f..%.2f]m "
          "radius=[%.2f..%.2f]m step<=%.3fm orientation_step<=%.3frad",
          group_name.c_str(), group.target_filter.workspace_min_m[0],
          group.target_filter.workspace_max_m[0], group.target_filter.workspace_min_m[1],
          group.target_filter.workspace_max_m[1], group.target_filter.workspace_min_m[2],
          group.target_filter.workspace_max_m[2], group.target_filter.min_radius_m,
          group.target_filter.max_radius_m, group.target_filter.max_position_step_m,
          group.target_filter.max_orientation_step_rad);
      }
      group.model.name = group_name;
      group.model.joint_names = group.joint_names;
      for (const auto & joint_name : group.joint_names) {
        const auto joint = robot_model.getJoint(joint_name);
        if (!joint || !joint->limits) {
          throw std::invalid_argument("URDF has no limits for joint: " + joint_name);
        }
        group.model.lower_position_rad.push_back(joint->limits->lower);
        group.model.upper_position_rad.push_back(joint->limits->upper);
      }
      groups_.emplace(group_name, std::move(group));
    }
  }

  void onFeedback(const sensor_msgs::msg::JointState & message)
  {
    const auto count = std::min(message.name.size(), message.position.size());
    for (std::size_t index = 0; index < count; ++index) {
      if (!message.name[index].empty() && std::isfinite(message.position[index])) {
        positions_[message.name[index]] = message.position[index];
      }
    }
    last_feedback_at_ = SteadyClock::now();
    if (publish_fk_) {
      publishFk();
    }
  }

  bool feedbackFor(const Group & group, JointFeedback & output) const
  {
    output.joint_names = group.joint_names;
    output.positions_rad.reserve(group.joint_names.size());
    for (const auto & name : group.joint_names) {
      const auto found = positions_.find(name);
      if (found == positions_.end()) {
        return false;
      }
      output.positions_rad.push_back(found->second);
    }
    output.received_at = last_feedback_at_;
    return true;
  }

  MotionLimits limits(const std::size_t joint_count) const
  {
    MotionLimits output;
    output.joint_max_velocity_rad_s.assign(joint_count, joint_max_velocity_);
    output.joint_max_acceleration_rad_s2.assign(joint_count, joint_max_acceleration_);
    output.joint_max_jerk_rad_s3.assign(joint_count, joint_max_jerk_);
    output.cartesian_max_linear_velocity_m_s = cartesian_max_linear_velocity_;
    output.cartesian_max_linear_acceleration_m_s2 = cartesian_max_linear_acceleration_;
    output.cartesian_max_linear_jerk_m_s3 = cartesian_max_linear_jerk_;
    output.cartesian_max_angular_velocity_rad_s = cartesian_max_angular_velocity_;
    output.cartesian_max_angular_acceleration_rad_s2 = cartesian_max_angular_acceleration_;
    output.cartesian_max_angular_jerk_rad_s3 = cartesian_max_angular_jerk_;
    return output;
  }

  std::optional<Pose> measuredPose(const Group & group, const JointFeedback & feedback)
  {
    ForwardKinematicsRequest request;
    request.group_name = group.name;
    request.base_link = group.base_frame;
    request.link_name = group.tip_frame;
    request.joints.joint_names = feedback.joint_names;
    request.joints.positions_rad = feedback.positions_rad;
    const auto result = backend_->forwardKinematics(request);
    if (!result.status.ok()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot anchor target filter for %s: %s", group.name.c_str(),
        result.status.message.c_str());
      return std::nullopt;
    }
    return result.pose;
  }

  Pose filterTarget(
    const Pose & requested, Group & group, const JointFeedback & feedback,
    bool & limited)
  {
    limited = false;
    if (!group.target_filter.enabled) {
      return requested;
    }

    Pose output = requested;
    const auto & config = group.target_filter;
    for (std::size_t index = 0; index < 3U; ++index) {
      const auto clamped = std::clamp(
        output.position_m[index], config.workspace_min_m[index],
        config.workspace_max_m[index]);
      limited = limited || clamped != output.position_m[index];
      output.position_m[index] = clamped;
    }

    auto radius = std::sqrt(
      output.position_m[0] * output.position_m[0] +
      output.position_m[1] * output.position_m[1] +
      output.position_m[2] * output.position_m[2]);
    if (radius > config.max_radius_m) {
      const auto scale = config.max_radius_m / radius;
      for (auto & value : output.position_m) {
        value *= scale;
      }
      radius = config.max_radius_m;
      limited = true;
    } else if (radius < config.min_radius_m) {
      std::array<double, 3> direction{{1.0, 0.0, 0.0}};
      if (radius > 1e-9) {
        for (std::size_t index = 0; index < 3U; ++index) {
          direction[index] = output.position_m[index] / radius;
        }
      }
      for (std::size_t index = 0; index < 3U; ++index) {
        output.position_m[index] = direction[index] * config.min_radius_m;
      }
      limited = true;
    }

    auto anchor = group.last_accepted_target;
    if (!anchor) {
      anchor = measuredPose(group, feedback);
    }
    if (!anchor) {
      return output;
    }

    std::array<double, 3> delta{};
    double distance_sq = 0.0;
    for (std::size_t index = 0; index < 3U; ++index) {
      delta[index] = output.position_m[index] - anchor->position_m[index];
      distance_sq += delta[index] * delta[index];
    }
    const auto distance = std::sqrt(distance_sq);
    if (distance > config.max_position_step_m) {
      const auto scale = config.max_position_step_m / distance;
      for (std::size_t index = 0; index < 3U; ++index) {
        output.position_m[index] = anchor->position_m[index] + delta[index] * scale;
      }
      limited = true;
    }

    auto from = anchor->orientation_xyzw;
    auto to = output.orientation_xyzw;
    double dot = 0.0;
    for (std::size_t index = 0; index < 4U; ++index) {
      dot += from[index] * to[index];
    }
    if (dot < 0.0) {
      for (auto & value : to) {
        value = -value;
      }
      dot = -dot;
    }
    dot = std::clamp(dot, 0.0, 1.0);
    const auto angle = 2.0 * std::acos(dot);
    if (angle > config.max_orientation_step_rad) {
      const auto fraction = config.max_orientation_step_rad / angle;
      const auto theta = std::acos(dot);
      if (theta > 1e-8) {
        const auto denominator = std::sin(theta);
        const auto from_weight = std::sin((1.0 - fraction) * theta) / denominator;
        const auto to_weight = std::sin(fraction * theta) / denominator;
        for (std::size_t index = 0; index < 4U; ++index) {
          output.orientation_xyzw[index] =
            from_weight * from[index] + to_weight * to[index];
        }
      }
      limited = true;
    }
    return output;
  }

  void onTarget(const hc_teleop_interfaces::msg::CartesianTargetArray & message)
  {
    if (message.source_id.empty() || message.session_id.empty() || message.targets.empty()) {
      return;
    }
    if (timeNanoseconds(message.valid_until) <= now().nanoseconds()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ignored expired RoboManip target envelope");
      return;
    }

    std::map<std::string, const hc_teleop_interfaces::msg::CartesianTarget *> targets;
    for (const auto & target : message.targets) {
      const auto found = groups_.find(target.group_name);
      if (found == groups_.end() || !finitePose(target.pose) ||
        target.reference_frame != found->second.base_frame ||
        target.tip_frame != found->second.tip_frame ||
        !targets.emplace(target.group_name, &target).second)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "rejected atomic RoboManip target envelope: invalid group/frame/pose");
        return;
      }
    }

    const auto steady_now = SteadyClock::now();
    if (steady_now - last_feedback_at_ > feedback_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignored RoboManip target: measured joint feedback is stale");
      return;
    }

    for (const auto & entry : targets) {
      solveAndPublish(message, *entry.second, groups_.at(entry.first), steady_now);
    }
  }

  void solveAndPublish(
    const hc_teleop_interfaces::msg::CartesianTargetArray & envelope,
    const hc_teleop_interfaces::msg::CartesianTarget & target,
    Group & group, const SteadyTime steady_now)
  {
    if (group.last_published_sequence && group.source_id == envelope.source_id &&
      group.input_session_id == envelope.session_id &&
      *group.last_published_sequence == envelope.sequence)
    {
      return;
    }

    JointFeedback feedback;
    if (!feedbackFor(group, feedback)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignored RoboManip target for %s: missing measured joints", group.name.c_str());
      return;
    }

    Pose desired = toCorePose(target.pose);
    if (target.pose_mode != hc_teleop_interfaces::msg::CartesianTarget::FULL_POSE) {
      ForwardKinematicsRequest fk;
      fk.group_name = group.name;
      fk.base_link = group.base_frame;
      fk.link_name = group.tip_frame;
      fk.joints.joint_names = feedback.joint_names;
      fk.joints.positions_rad = feedback.positions_rad;
      const auto measured = backend_->forwardKinematics(fk);
      if (!measured.status.ok()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "cannot resolve partial Cartesian target for %s: %s",
          group.name.c_str(), measured.status.message.c_str());
        return;
      }
      if (target.pose_mode == hc_teleop_interfaces::msg::CartesianTarget::POSITION_ONLY) {
        desired.orientation_xyzw = measured.pose.orientation_xyzw;
      } else if (
        target.pose_mode == hc_teleop_interfaces::msg::CartesianTarget::ORIENTATION_ONLY)
      {
        desired.position_m = measured.pose.position_m;
      } else {
        return;
      }
    }

    const bool identity_changed = group.source_id != envelope.source_id ||
      group.input_session_id != envelope.session_id;
    const bool timed_out = group.active &&
      steady_now - group.last_target_at > session_reset_timeout_;
    if (identity_changed || timed_out) {
      stop(group);
    }

    bool target_limited = false;
    desired = filterTarget(desired, group, feedback, target_limited);
    if (target_limited) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "limited %s Cartesian target to configured workspace/step envelope",
        group.name.c_str());
    }

    if (!group.active || identity_changed || timed_out) {
      group.source_id = envelope.source_id;
      group.input_session_id = envelope.session_id;
      group.sdk_session_id =
        envelope.source_id + ":" + envelope.session_id + ":" + group.name;
      const auto reset = backend_->resetFinalJointTarget(group.name, feedback);
      if (!reset.ok()) {
        reportFailure(group, "RTC reset", reset.message);
        return;
      }
      ServoPRequest request;
      request.request_id = group.sdk_session_id;
      request.group_name = group.name;
      request.base_link = group.base_frame;
      request.link_name = group.tip_frame;
      request.target = desired;
      request.limits = limits(group.joint_names.size());
      const auto started = backend_->startSession(
        group.sdk_session_id, request, feedback, nominal_period_sec_);
      if (!started.ok()) {
        reportFailure(group, "session start", started.message);
        return;
      }
      group.active = true;
      group.last_published_sequence.reset();
      group.last_step_at = steady_now;
    }

    double period_sec = nominal_period_sec_;
    if (group.last_step_at.time_since_epoch().count() != 0) {
      period_sec = std::chrono::duration<double>(steady_now - group.last_step_at).count();
      period_sec = std::clamp(period_sec, nominal_period_sec_ * 0.25, 0.05);
    }
    group.last_target_at = steady_now;
    group.last_step_at = steady_now;

    DynamicTarget dynamic_target{desired};
    auto tick = backend_->tickSession(group.sdk_session_id, &dynamic_target, period_sec);
    if (!tick.status.ok()) {
      ++group.consecutive_tick_failures;
      const bool can_hold = group.last_safe_candidate &&
        group.consecutive_tick_failures < tick_failure_reset_count_;
      if (can_hold) {
        // Publish first: the supplementary diagnostic IK may take tens of
        // milliseconds and must not delay the safe hold command.
        publishCandidate(envelope, group, *group.last_safe_candidate, true);
      }
      if (group.consecutive_tick_failures == 1U ||
        group.consecutive_tick_failures >= tick_failure_reset_count_)
      {
        reportTickFailure(group, tick.status, desired, feedback);
      }
      if (can_hold) {
        return;
      }
      if (group.consecutive_tick_failures >= tick_failure_reset_count_) {
        RCLCPP_ERROR(
          get_logger(),
          "RoboManip ServoP session reset for %s after %zu consecutive Tick failures",
          group.name.c_str(), group.consecutive_tick_failures);
        stop(group);
      }
      return;
    }
    group.consecutive_tick_failures = 0U;
    auto final = backend_->updateFinalJointTarget(group.name, tick.candidate, period_sec);
    if (!final.status.ok() || !final.candidate.passed_final_sdk_rtc) {
      reportFailure(group, "final RTC", final.status.message);
      stop(group);
      return;
    }
    group.last_safe_candidate = final.candidate;
    group.last_accepted_target = desired;
    publishCandidate(envelope, group, final.candidate, false);
  }

  void publishCandidate(
    const hc_teleop_interfaces::msg::CartesianTargetArray & envelope,
    Group & group, const JointCommand & command, const bool holding)
  {
    hc_teleop_interfaces::msg::JointCommandCandidate output;
    output.header.stamp = now();
    output.source_id = envelope.source_id;
    output.session_id = envelope.session_id;
    output.sequence = envelope.sequence;
    output.valid_until = envelope.valid_until;
    output.control_mode =
      hc_teleop_interfaces::msg::JointCommandCandidate::CONTROL_MODE_POSITION;
    output.group_name = group.name;
    output.command.header = output.header;
    output.command.name = command.joint_names;
    output.command.position = command.positions_rad;
    if (holding) {
      // A held command represents a stationary, previously RTC-approved pose;
      // stale non-zero velocities must not leak into the new envelope.
      output.command.velocity.assign(command.joint_names.size(), 0.0);
    } else {
      output.command.velocity = command.velocities_rad_s;
    }
    candidate_publisher_->publish(std::move(output));
    group.last_published_sequence = envelope.sequence;
  }

  std::string diagnoseIkFailure(
    const Group & group, const Pose & desired, const JointFeedback & feedback) const
  {
    sdk_kinematics::InverseKinematicsRequest request;
    request.kinematics.group_name = group.name;
    request.kinematics.base_link = group.base_frame;
    request.kinematics.link_name = group.tip_frame;
    request.target_pose.position_m = {
      desired.position_m[0], desired.position_m[1], desired.position_m[2]};
    request.target_pose.orientation = {
      desired.orientation_xyzw[0], desired.orientation_xyzw[1],
      desired.orientation_xyzw[2], desired.orientation_xyzw[3]};
    sdk_kinematics::JointState seed;
    seed.joint_names = feedback.joint_names;
    seed.positions_rad = feedback.positions_rad;
    request.seed = seed;
    request.parameters.position_tolerance_m = ik_tuning_.position_tolerance_mm / 1000.0;
    request.parameters.orientation_tolerance_rad = ik_tuning_.orientation_tolerance_rad;
    request.parameters.enable_joint_task = ik_tuning_.enable_joint_task;
    request.parameters.placo_joint_task_weight = ik_tuning_.joint_task_weight;
    if (ik_tuning_.enable_joint_task) {
      request.parameters.redundancy_preference = seed;
    }

    const auto result = diagnostic_kinematics_->inverseKinematics(request);
    std::ostringstream detail;
    detail << "target_m=[" << desired.position_m[0] << ',' << desired.position_m[1] << ',' <<
      desired.position_m[2] << "] direct_ik=";
    if (!result.ok()) {
      detail << "failed(code=" << static_cast<int>(result.status.code) << ", reason='" <<
        result.status.message << "')";
    } else {
      detail << "succeeded(position_error_m=" << result.value->position_error_m <<
        ", orientation_error_rad=" << result.value->orientation_error_rad << ')';
    }
    return detail.str();
  }

  void reportTickFailure(
    Group & group, const humanoid_motion_server::motion::MotionStatus & status,
    const Pose & desired, const JointFeedback & feedback)
  {
    const auto diagnostic = diagnoseIkFailure(group, desired, feedback);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "RoboManip ServoP Tick failed for %s (%zu/%zu): api=%s code=%lld reason='%s'; %s; "
      "holding_previous_candidate=%s",
      group.name.c_str(), group.consecutive_tick_failures, tick_failure_reset_count_,
      status.sdk_api.empty() ? "unknown" : status.sdk_api.c_str(),
      static_cast<long long>(status.sdk_code), status.message.c_str(), diagnostic.c_str(),
      group.last_safe_candidate ? "yes" : "no");
  }

  void reportFailure(Group & group, const char * stage, const std::string & reason)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "RoboManip %s failed for %s: %s",
      stage, group.name.c_str(), reason.c_str());
  }

  void stop(Group & group)
  {
    if (group.active && backend_) {
      const auto result = backend_->stopSession(group.sdk_session_id);
      if (!result.ok()) {
        RCLCPP_WARN(
          get_logger(), "failed to stop RoboManip session for %s: %s",
          group.name.c_str(), result.message.c_str());
      }
    }
    group.active = false;
    group.sdk_session_id.clear();
    group.last_published_sequence.reset();
    group.last_safe_candidate.reset();
    group.last_accepted_target.reset();
    group.consecutive_tick_failures = 0U;
    group.last_target_at = {};
    group.last_step_at = {};
  }

  void expireSessions()
  {
    const auto now_steady = SteadyClock::now();
    for (auto & entry : groups_) {
      auto & group = entry.second;
      if (group.active && now_steady - group.last_target_at > session_reset_timeout_) {
        stop(group);
      }
    }
  }

  void publishFk()
  {
    hc_teleop_interfaces::msg::CartesianStateArray output;
    output.header.stamp = now();
    output.states.reserve(groups_.size());
    for (const auto & entry : groups_) {
      const auto & group = entry.second;
      hc_teleop_interfaces::msg::CartesianState state;
      state.group_name = group.name;
      state.reference_frame = group.base_frame;
      state.tip_frame = group.tip_frame;
      JointFeedback feedback;
      if (feedbackFor(group, feedback)) {
        ForwardKinematicsRequest request;
        request.group_name = group.name;
        request.base_link = group.base_frame;
        request.link_name = group.tip_frame;
        request.joints.joint_names = feedback.joint_names;
        request.joints.positions_rad = feedback.positions_rad;
        const auto fk = backend_->forwardKinematics(request);
        state.valid = fk.status.ok();
        if (state.valid) {
          state.pose = toRosPose(fk.pose);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "RoboManip FK failed for %s: %s",
            group.name.c_str(), fk.status.message.c_str());
        }
      }
      output.states.push_back(std::move(state));
    }
    cartesian_publisher_->publish(std::move(output));
  }

  std::map<std::string, Group> groups_;
  std::unordered_map<std::string, double> positions_;
  SteadyTime last_feedback_at_{};
  double nominal_period_sec_{1.0 / 60.0};
  SteadyClock::duration feedback_timeout_{std::chrono::milliseconds(200)};
  SteadyClock::duration session_reset_timeout_{std::chrono::milliseconds(250)};
  std::size_t tick_failure_reset_count_{3U};
  bool publish_fk_{true};
  SdkIkTuning ik_tuning_;
  double joint_max_velocity_{1.0};
  double joint_max_acceleration_{2.0};
  double joint_max_jerk_{10.0};
  double cartesian_max_linear_velocity_{0.3};
  double cartesian_max_linear_acceleration_{0.6};
  double cartesian_max_linear_jerk_{3.0};
  double cartesian_max_angular_velocity_{1.0};
  double cartesian_max_angular_acceleration_{2.0};
  double cartesian_max_angular_jerk_{10.0};
  std::unique_ptr<RuntimeSdkConfig> runtime_sdk_config_;
  std::shared_ptr<sdk_kinematics::SdkKinematics> diagnostic_kinematics_;
  std::shared_ptr<Backend> backend_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::CartesianTargetArray>::SharedPtr
    target_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_subscription_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::JointCommandCandidate>::SharedPtr
    candidate_publisher_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::CartesianStateArray>::SharedPtr
    cartesian_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace hc_motion_backend_robo_manip

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hc_motion_backend_robo_manip::RoboManipBackendNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("robo_manip_backend"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
