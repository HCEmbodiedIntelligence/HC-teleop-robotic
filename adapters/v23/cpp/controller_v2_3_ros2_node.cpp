#include "v23_ik_core.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

using hc::v23::ControllerV23;
using hc::v23::TargetStore;
using hc::v23::TargetTransform;
using hc::v23::quaternion_to_rotation;

double monotonic_seconds() {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

int64_t stamp_to_ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

std::string strip_leading_slashes(const std::string& value) {
  std::size_t pos = 0;
  while (pos < value.size() && value[pos] == '/') {
    ++pos;
  }
  return value.substr(pos);
}

std::string trim_lower(const std::string& input) {
  std::size_t start = 0;
  while (start < input.size() &&
         std::isspace(static_cast<unsigned char>(input[start]))) {
    ++start;
  }
  std::size_t end = input.size();
  while (end > start &&
         std::isspace(static_cast<unsigned char>(input[end - 1]))) {
    --end;
  }
  std::string out = input.substr(start, end - start);
  for (char& ch : out) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return out;
}

std::string generate_uuid_hex() {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  std::ostringstream oss;
  oss << std::hex << std::setfill('0') << std::setw(16) << dist(gen)
      << std::setw(16) << dist(gen);
  return oss.str();
}

std::string json_escape(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream oss;
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(c));
          out += oss.str();
        } else {
          out += c;
        }
    }
  }
  return out;
}

struct ControlToken {
  std::string controller_instance_id;
  std::string session_id;
  int64_t epoch = 0;
  std::string source;
  std::string intervention_id;

  std::string key() const {
    return "hc1:" + controller_instance_id + ":" + session_id + ":" +
           std::to_string(epoch) + ":" + source + ":" + intervention_id;
  }

  bool operator==(const ControlToken& other) const {
    return controller_instance_id == other.controller_instance_id &&
           session_id == other.session_id && epoch == other.epoch &&
           source == other.source && intervention_id == other.intervention_id;
  }
  bool operator!=(const ControlToken& other) const { return !(*this == other); }

  static std::optional<ControlToken> from_key(const std::string& value) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (true) {
      const std::size_t pos = value.find(':', start);
      if (pos == std::string::npos) {
        fields.push_back(value.substr(start));
        break;
      }
      fields.push_back(value.substr(start, pos - start));
      start = pos + 1;
    }
    if (fields.size() != 6 || fields[0] != "hc1" || fields[3].empty()) {
      return std::nullopt;
    }
    for (char c : fields[3]) {
      if (!std::isdigit(static_cast<unsigned char>(c))) {
        return std::nullopt;
      }
    }
    try {
      ControlToken tok;
      tok.controller_instance_id = fields[1];
      tok.session_id = fields[2];
      tok.epoch = std::stoll(fields[3]);
      tok.source = fields[4];
      tok.intervention_id = fields[5];
      return tok;
    } catch (...) {
      return std::nullopt;
    }
  }
};

class ControllerV23Node : public rclcpp::Node {
 public:
  explicit ControllerV23Node(const std::string& config_path)
      : ControllerV23Node(std::make_unique<ControllerV23>(config_path)) {}

 private:
  explicit ControllerV23Node(std::unique_ptr<ControllerV23> controller)
      : rclcpp::Node(controller->ros_config().node_name),
        controller_(std::move(controller)) {
    const auto& ros_cfg = controller_->ros_config();
    if (ros_cfg.rate <= 0.0) {
      throw std::invalid_argument("ros_interface.rate must be positive");
    }

    auto latest_reliable_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto feedback_reliable_qos =
        rclcpp::QoS(rclcpp::KeepLast(4)).reliable();
    auto source_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        ros_cfg.pub_joint_target, latest_reliable_qos);
    if (!ros_cfg.pub_solver_state.empty()) {
      solver_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
          ros_cfg.pub_solver_state, latest_reliable_qos);
    }
    reset_ack_pub_ = this->create_publisher<std_msgs::msg::Header>(
        ros_cfg.pub_reset_ack, feedback_reliable_qos);

    for (const auto& name : controller_->free_joint_names()) {
      free_joint_set_.insert(name);
    }
    for (const auto& task : controller_->pose_tasks()) {
      ee_pose_order_.emplace_back(task.root, task.frame);
    }

    const auto& peer = controller_->peer_teleop_config();
    std::unordered_set<std::string> arm_name_set(peer.arm_joint_names.begin(),
                                                 peer.arm_joint_names.end());
    for (int i = 0; i < static_cast<int>(controller_->free_joint_names().size());
         ++i) {
      if (arm_name_set.count(controller_->free_joint_names()[i]) > 0) {
        clutch_columns_.push_back(i);
        clutch_q_indices_.push_back(controller_->interface().q_indices()[i]);
      }
    }

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        ros_cfg.sub_joint_state, latest_reliable_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          on_joint_state(*msg);
        });

    if (!ros_cfg.sub_reset.empty()) {
      reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          ros_cfg.sub_reset, feedback_reliable_qos,
          [this](const std_msgs::msg::Bool::SharedPtr msg) {
            on_reset(*msg);
          });
    }
    if (!ros_cfg.sub_tf_target.empty()) {
      tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
          ros_cfg.sub_tf_target, rclcpp::QoS(rclcpp::KeepLast(10)),
          [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            on_tf_target(*msg);
          });
    }
    if (!ros_cfg.sub_ee_target.empty()) {
      ee_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          ros_cfg.sub_ee_target, latest_reliable_qos,
          [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            on_ee_target(*msg, false);
          });
    }

    control_source_sub_ = this->create_subscription<std_msgs::msg::String>(
        ros_cfg.sub_control_source, source_qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          on_control_source(*msg);
        });

    clutch_sub_ = this->create_subscription<std_msgs::msg::Header>(
        "/control/solver/clutch", feedback_reliable_qos,
        [this](const std_msgs::msg::Header::SharedPtr msg) {
          on_clutch(*msg);
        });

    if (peer.legacy_intervention) {
      intervention_enabled_ = true;
      intervention_instance_ = generate_uuid_hex();
      gripper_limits_ = peer.gripper_limits;

      ack_pub_ = this->create_publisher<std_msgs::msg::String>(
          "/status/control/ack", rclcpp::QoS(rclcpp::KeepLast(10)));
      record_ee_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
          "/internal/recording/control/solver/ee_targets",
          rclcpp::QoS(rclcpp::KeepLast(10)));
      record_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
          "/internal/recording/control/solver/joint_targets",
          rclcpp::QoS(rclcpp::KeepLast(10)));
      owned_output_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
          "/internal/control/joint_targets", rclcpp::QoS(rclcpp::KeepLast(10)));

      permit_sub_ = this->create_subscription<std_msgs::msg::Header>(
          "/control/motion/permit", rclcpp::QoS(rclcpp::KeepLast(1)),
          [this](const std_msgs::msg::Header::SharedPtr msg) {
            on_permit(*msg);
          });
      owned_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "/internal/vr/owned_poses", rclcpp::QoS(rclcpp::KeepLast(1)),
          [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            on_owned_pose(*msg);
          });
      owned_home_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/internal/vr/owned_home", rclcpp::QoS(rclcpp::KeepLast(1)),
          [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            on_owned_home(*msg);
          });
      owned_aux_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/internal/vr/aux_joint_targets", rclcpp::QoS(rclcpp::KeepLast(1)),
          [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            on_owned_aux(*msg);
          });
      heartbeat_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100), [this]() { publish_heartbeat(); });
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / ros_cfg.rate));
    tick_timer_ = this->create_wall_timer(period, [this]() { on_tick(); });

    RCLCPP_INFO(this->get_logger(),
                "reconstructed controller_v2_3 (C++17) started: %zu free "
                "joints, %d tasks, rate=%.1f Hz, intervention=%s",
                controller_->free_joint_names().size(),
                controller_->total_task_count(), ros_cfg.rate,
                intervention_enabled_ ? "enabled" : "disabled");
  }

  bool intervention_active() const {
    return intervention_enabled_ && iv_active_;
  }

  bool stamp_fresh(int64_t stamp_ns) {
    if (stamp_ns <= 0) return false;
    const double age_s =
        static_cast<double>(this->get_clock()->now().nanoseconds() - stamp_ns) *
        1e-9;
    return age_s >= -0.05 && age_s <= 0.25;
  }

  bool intervention_authorized() const {
    if (!intervention_active() || !iv_ready_ || !iv_error_.empty() ||
        iv_phase_ != "grant" || !iv_token_.has_value() ||
        iv_token_->source != "vr") {
      return false;
    }
    const double now = monotonic_seconds();
    return (now - iv_permit_time_ <= 0.3) &&
           (now - iv_feedback_time_ <= 0.25) &&
           (now - iv_target_time_ <= 0.2);
  }

  bool validate_joint_message_fields(
      const sensor_msgs::msg::JointState& msg) const {
    if (msg.name.size() != msg.position.size()) return false;
    std::unordered_set<std::string> seen;
    seen.reserve(msg.name.size());
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (!std::isfinite(msg.position[i])) return false;
      if (!seen.insert(msg.name[i]).second) return false;
    }
    for (const auto& req : free_joint_set_) {
      if (seen.count(req) == 0) return false;
    }
    return true;
  }

  void reset_references() {
    clutch_held_ = false;
    clutch_reset_pending_ = false;
    reset_pending_ = true;
    reset_cutoff_ns_ = this->get_clock()->now().nanoseconds();
    clutch_event_ns_ = std::max(clutch_event_ns_, reset_cutoff_ns_);
    source_target_pending_ = true;
    controller_->targets().clear();
  }

  void on_control_source(const std_msgs::msg::String& msg) {
    const std::string source = trim_lower(msg.data);
    if ((source != "vr" && source != "motion" && source != "exoskeleton") ||
        source == control_source_) {
      return;
    }
    control_source_ = source;
    clutch_held_ = false;
    clutch_reset_pending_ = false;
    if (intervention_active()) {
      return;
    }
    reset_references();
  }

  void on_clutch(const std_msgs::msg::Header& msg) {
    if (control_source_ != "vr" || intervention_active()) {
      return;
    }
    const int64_t stamp_ns = stamp_to_ns(msg.stamp);
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    if ((msg.frame_id != "hold" && msg.frame_id != "resume" &&
         msg.frame_id != "clear") ||
        (now_ns - stamp_ns < -50000000LL) ||
        (now_ns - stamp_ns > 250000000LL)) {
      return;
    }
    if (stamp_ns <= clutch_event_ns_) {
      return;
    }
    clutch_event_ns_ = stamp_ns;
    if (msg.frame_id == "clear") {
      clutch_held_ = false;
      clutch_reset_pending_ = false;
    } else if (msg.frame_id == "hold") {
      clutch_held_ = true;
      clutch_reset_pending_ = false;
      for (int col : clutch_columns_) {
        controller_->previous_velocity()[col] = 0.0;
      }
    } else {
      clutch_held_ = true;
      clutch_reset_pending_ = true;
      clutch_feedback_cutoff_ns_ = now_ns;
      source_target_pending_ = true;
      source_target_cutoff_ns_ = now_ns;
    }
    RCLCPP_INFO(this->get_logger(), "arm clutch %s: event_ns=%ld, held=%s",
                msg.frame_id.c_str(), static_cast<long>(stamp_ns),
                clutch_held_ ? "true" : "false");
  }

  void on_joint_state(const sensor_msgs::msg::JointState& msg) {
    ++feedback_received_total_;
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const int64_t stamp_ns = stamp_to_ns(msg.header.stamp);
    last_feedback_receive_ns_ = now_ns;
    last_feedback_source_ns_ = stamp_ns;

    const bool resetting = reset_pending_ && !intervention_active();
    std::string reason;
    if (msg.name.size() != msg.position.size()) {
      reason = "length_mismatch";
    } else {
      std::unordered_set<std::string> seen;
      seen.reserve(msg.name.size());
      bool duplicate = false;
      bool nonfinite = false;
      for (std::size_t i = 0; i < msg.name.size(); ++i) {
        if (!seen.insert(msg.name[i]).second) duplicate = true;
        if (!std::isfinite(msg.position[i])) nonfinite = true;
      }
      if (duplicate) {
        reason = "duplicate_names";
      } else {
        bool complete = true;
        for (const auto& req : free_joint_set_) {
          if (seen.count(req) == 0) {
            complete = false;
            break;
          }
        }
        if (!complete) {
          reason = "incomplete_joints";
        } else if (nonfinite) {
          reason = "nonfinite_position";
        } else if ((now_ns - stamp_ns < -50000000LL) ||
                   (now_ns - stamp_ns > 250000000LL)) {
          reason = "stale_or_future_feedback";
        } else if (resetting && stamp_ns <= reset_cutoff_ns_) {
          reason = "before_reset_cutoff";
        } else if (clutch_reset_pending_ &&
                   stamp_ns <= clutch_feedback_cutoff_ns_) {
          reason = "before_clutch_cutoff";
        }
      }
    }

    if (!reason.empty()) {
      ++feedback_rejections_[reason];
      const double now = monotonic_seconds();
      if (now - last_feedback_rejection_log_ >= 2.0) {
        last_feedback_rejection_log_ = now;
        RCLCPP_WARN(
            this->get_logger(),
            "joint feedback rejected before state update: reason=%s, "
            "source_ns=%ld, receive_ns=%ld, age_ms=%.3f, reset_pending=%s, "
            "reset_cutoff_ns=%ld",
            reason.c_str(), static_cast<long>(stamp_ns),
            static_cast<long>(now_ns),
            static_cast<double>(now_ns - stamp_ns) / 1e6,
            reset_pending_ ? "true" : "false",
            static_cast<long>(reset_cutoff_ns_));
      }
      return;
    }

    try {
      controller_->update_joint_state(msg.name, msg.position);
      if (resetting) {
        controller_->reset_to_feedback();
        reset_pending_ = false;
        if (source_target_pending_) {
          source_target_cutoff_ns_ = now_ns;
        }
        std_msgs::msg::Header ack;
        ack.stamp = this->get_clock()->now();
        reset_ack_pub_->publish(ack);
        RCLCPP_WARN(this->get_logger(),
                    "retarget integrator reset to fresh joint feedback");
      }
      if (clutch_reset_pending_ && !resetting) {
        for (std::size_t i = 0; i < clutch_columns_.size(); ++i) {
          controller_->command_q()[clutch_q_indices_[i]] =
              controller_->q()[clutch_q_indices_[i]];
          controller_->previous_velocity()[clutch_columns_[i]] = 0.0;
        }
        clutch_reset_pending_ = false;
        clutch_held_ = false;
        source_target_cutoff_ns_ = now_ns;
        std_msgs::msg::Header ack;
        ack.stamp = this->get_clock()->now();
        ack.frame_id = "clutch";
        reset_ack_pub_->publish(ack);
        RCLCPP_INFO(this->get_logger(),
                    "arm clutch integrators reset to fresh feedback; waiting "
                    "for new targets");
      }
      if (intervention_enabled_) {
        on_intervention_feedback(msg);
      }
    } catch (const std::exception& exc) {
      RCLCPP_WARN(this->get_logger(), "ignoring invalid joint feedback: %s",
                  exc.what());
    }
  }

  void on_reset(const std_msgs::msg::Bool& msg) {
    if (intervention_active()) {
      return;
    }
    if (msg.data) {
      reset_references();
    }
  }

  void on_tf_target(const tf2_msgs::msg::TFMessage& msg) {
    if (intervention_active()) return;
    if (control_source_ != "vr" || reset_pending_) return;
    if (source_target_pending_) {
      if (msg.transforms.empty()) return;
      for (const auto& tf : msg.transforms) {
        if (stamp_to_ns(tf.header.stamp) <= source_target_cutoff_ns_) {
          return;
        }
      }
    }
    std::map<TargetStore::Key, TargetTransform> targets;
    try {
      for (const auto& tf : msg.transforms) {
        const std::string parent = strip_leading_slashes(tf.header.frame_id);
        const std::string child = strip_leading_slashes(tf.child_frame_id);
        const auto& tr = tf.transform.translation;
        const auto& rot = tf.transform.rotation;
        TargetTransform tt;
        tt.translation = Eigen::Vector3d(tr.x, tr.y, tr.z);
        tt.rotation = quaternion_to_rotation(rot.x, rot.y, rot.z, rot.w);
        targets[{parent, child}] = tt;
      }
    } catch (const std::exception& exc) {
      RCLCPP_WARN(this->get_logger(), "ignoring invalid TF target: %s",
                  exc.what());
      return;
    }
    controller_->update_tf_targets(targets);
    bool all_present = true;
    for (const auto& task : controller_->pose_tasks()) {
      if (!controller_->targets().relative_target(task.root, task.frame)) {
        all_present = false;
        break;
      }
    }
    if (all_present) {
      source_target_pending_ = false;
    }
  }

  bool on_ee_target(const geometry_msgs::msg::PoseArray& msg, bool owned) {
    if (intervention_active() && !owned) return false;
    if (!owned) {
      if (control_source_ != "vr" || reset_pending_) return false;
      const int64_t stamp_ns = stamp_to_ns(msg.header.stamp);
      if (source_target_pending_ && stamp_ns <= source_target_cutoff_ns_) {
        return false;
      }
    }
    if (msg.poses.size() != ee_pose_order_.size()) {
      RCLCPP_WARN(this->get_logger(), "ignoring %zu EE targets; expected %zu",
                  msg.poses.size(), ee_pose_order_.size());
      return false;
    }
    std::map<TargetStore::Key, TargetTransform> targets;
    try {
      for (std::size_t i = 0; i < ee_pose_order_.size(); ++i) {
        const auto& pose = msg.poses[i];
        TargetTransform tt;
        tt.translation = Eigen::Vector3d(pose.position.x, pose.position.y,
                                         pose.position.z);
        tt.rotation = quaternion_to_rotation(
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w);
        targets[ee_pose_order_[i]] = tt;
      }
    } catch (const std::exception& exc) {
      RCLCPP_WARN(this->get_logger(), "ignoring invalid EE target: %s",
                  exc.what());
      return false;
    }
    controller_->update_tf_targets(targets);
    source_target_pending_ = false;
    return true;
  }

  // ---- V23Intervention methods ----
  void on_permit(const std_msgs::msg::Header& msg) {
    if (msg.frame_id.rfind("hc1:", 0) != 0) return;
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
      const std::size_t pos = msg.frame_id.find('#', start);
      if (pos == std::string::npos) {
        parts.push_back(msg.frame_id.substr(start));
        break;
      }
      parts.push_back(msg.frame_id.substr(start, pos - start));
      start = pos + 1;
    }
    if (parts.size() != 4) return;
    const auto token_opt = ControlToken::from_key(parts[0]);
    if (!token_opt.has_value()) return;
    const std::string& phase = parts[1];
    const std::string& instance = parts[2];
    const int64_t stamp = stamp_to_ns(msg.stamp);

    if (instance != intervention_instance_ ||
        (phase != "revoke" && phase != "grant" && phase != "off") ||
        stamp <= iv_permit_stamp_ || !stamp_fresh(stamp)) {
      return;
    }
    if (phase == "grant" &&
        (!iv_token_.has_value() || *token_opt != *iv_token_ ||
         (iv_phase_ != "revoke" && iv_phase_ != "grant") || !iv_ready_)) {
      return;
    }
    const bool changed =
        (!iv_token_.has_value() || *token_opt != *iv_token_ ||
         phase != iv_phase_);
    iv_permit_stamp_ = stamp;
    iv_permit_time_ = monotonic_seconds();
    if (changed) {
      iv_token_ = *token_opt;
      iv_phase_ = phase;
      iv_active_ = (phase != "off");
      if (phase == "grant") {
        control_source_ = iv_token_->source;
        source_target_pending_ = false;
      }
      iv_cutoff_ = stamp;
      iv_ready_ = false;
      iv_error_.clear();
      iv_target_stamp_ = 0;
      iv_target_time_ = -std::numeric_limits<double>::infinity();
      iv_homing_ = false;
      iv_home_command_.reset();
      iv_auxiliary_.reset();
      if (phase == "off") {
        control_source_.clear();
        std_msgs::msg::String vr_msg;
        vr_msg.data = "vr";
        on_control_source(vr_msg);
      }
      reset_pending_ = true;
    }
    publish_heartbeat();
  }

  void on_intervention_feedback(const sensor_msgs::msg::JointState& msg) {
    const int64_t stamp = stamp_to_ns(msg.header.stamp);
    if (!stamp_fresh(stamp) || !validate_joint_message_fields(msg)) {
      return;
    }
    iv_feedback_time_ = monotonic_seconds();
    if (iv_token_.has_value() && !iv_ready_ && stamp > iv_cutoff_) {
      try {
        controller_->reset_to_feedback();
        reset_pending_ = false;
        iv_ready_ = true;
      } catch (const std::exception& exc) {
        iv_error_ = exc.what();
      }
      publish_heartbeat();
    }
  }

  void on_owned_home(const sensor_msgs::msg::JointState& msg) {
    const std::size_t sep = msg.header.frame_id.find('#');
    if (sep == std::string::npos) return;
    const std::string key = msg.header.frame_id.substr(0, sep);
    const std::string frame = msg.header.frame_id.substr(sep + 1);
    const int64_t stamp = stamp_to_ns(msg.header.stamp);
    if (!iv_active_ || !iv_ready_ || iv_phase_ != "grant" ||
        !iv_token_.has_value() || iv_token_->source != "vr" ||
        key != iv_token_->key() || frame != "home" ||
        stamp <= std::max(iv_cutoff_, iv_target_stamp_) ||
        !stamp_fresh(stamp) || (monotonic_seconds() - iv_permit_time_ > 0.3) ||
        !validate_joint_message_fields(msg)) {
      return;
    }
    if (!iv_homing_) {
      controller_->reset_to_feedback();
    }
    iv_homing_ = true;
    iv_home_command_ = msg;
    iv_auxiliary_.reset();
    iv_target_stamp_ = stamp;
    iv_target_time_ = monotonic_seconds();
  }

  void on_owned_aux(const sensor_msgs::msg::JointState& msg) {
    const std::size_t sep = msg.header.frame_id.find('#');
    if (sep == std::string::npos) return;
    const std::string key = msg.header.frame_id.substr(0, sep);
    const std::string frame = msg.header.frame_id.substr(sep + 1);
    const int64_t stamp = stamp_to_ns(msg.header.stamp);
    const int64_t prev_stamp =
        iv_auxiliary_.has_value() ? stamp_to_ns(iv_auxiliary_->header.stamp)
                                  : iv_cutoff_;
    if (!iv_active_ || !iv_ready_ || iv_phase_ != "grant" ||
        !iv_token_.has_value() || iv_token_->source != "vr" ||
        key != iv_token_->key() || !frame.empty() || iv_homing_ ||
        stamp <= std::max(iv_cutoff_, prev_stamp) || !stamp_fresh(stamp) ||
        (monotonic_seconds() - iv_permit_time_ > 0.3) || msg.name.empty() ||
        msg.name.size() != msg.position.size()) {
      return;
    }
    std::unordered_set<std::string> seen;
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (!seen.insert(msg.name[i]).second) return;
      auto it = gripper_limits_.find(msg.name[i]);
      if (it == gripper_limits_.end()) return;
      const double v = msg.position[i];
      if (!std::isfinite(v) || v < 0.0 || v > it->second) return;
    }
    iv_auxiliary_ = msg;
  }

  void on_owned_pose(const geometry_msgs::msg::PoseArray& msg) {
    const std::size_t sep = msg.header.frame_id.find('#');
    if (sep == std::string::npos) return;
    const std::string key = msg.header.frame_id.substr(0, sep);
    const std::string frame = msg.header.frame_id.substr(sep + 1);
    const int64_t stamp = stamp_to_ns(msg.header.stamp);
    if (!iv_active_ || !iv_ready_ || iv_phase_ != "grant" ||
        !iv_token_.has_value() || iv_token_->source != "vr" ||
        key != iv_token_->key() || frame != "generic_task_bases" ||
        stamp <= std::max(iv_cutoff_, iv_target_stamp_) ||
        !stamp_fresh(stamp) || (monotonic_seconds() - iv_permit_time_ > 0.3)) {
      return;
    }
    if (on_ee_target(msg, true)) {
      if (iv_homing_) {
        controller_->reset_to_feedback();
      }
      iv_homing_ = false;
      iv_home_command_.reset();
      iv_target_stamp_ = stamp;
      iv_target_time_ = monotonic_seconds();
      if (record_ee_pub_->get_subscription_count() > 0) {
        geometry_msgs::msg::PoseArray rec;
        rec.header.stamp = msg.header.stamp;
        rec.header.frame_id = frame;
        rec.poses = msg.poses;
        record_ee_pub_->publish(rec);
      }
    }
  }

  void publish_owned(const sensor_msgs::msg::JointState& msg) {
    sensor_msgs::msg::JointState target;
    target.header.stamp = msg.header.stamp;
    target.header.frame_id = iv_token_->key() + "#";
    target.name = msg.name;
    target.position = msg.position;
    if (iv_auxiliary_.has_value()) {
      const int64_t aux_stamp = stamp_to_ns(iv_auxiliary_->header.stamp);
      if (stamp_fresh(aux_stamp)) {
        target.name.insert(target.name.end(), iv_auxiliary_->name.begin(),
                           iv_auxiliary_->name.end());
        target.position.insert(target.position.end(),
                               iv_auxiliary_->position.begin(),
                               iv_auxiliary_->position.end());
      }
    }
    owned_output_pub_->publish(target);
  }

  void publish_heartbeat() {
    if (!intervention_enabled_) return;
    if (iv_active_ && iv_phase_ == "grant" &&
        (monotonic_seconds() - iv_permit_time_ > 0.3)) {
      iv_error_ = "v23 control permit expired";
    }
    std::ostringstream oss;
    oss << "{\"participant\":\"solver\",\"instance\":\""
        << json_escape(intervention_instance_) << "\",\"stamp_ns\":"
        << this->get_clock()->now().nanoseconds() << ",\"token\":\""
        << json_escape((iv_token_.has_value() && iv_ready_) ? iv_token_->key()
                                                            : std::string())
        << "\",\"phase\":\"" << json_escape(iv_phase_)
        << "\",\"success\":" << (iv_error_.empty() ? "true" : "false")
        << ",\"reason\":\"" << json_escape(iv_error_) << "\"}";
    std_msgs::msg::String msg;
    msg.data = oss.str();
    ack_pub_->publish(msg);
  }

  void on_tick() {
    const bool owned = intervention_active();
    if (owned && !intervention_authorized()) {
      return;
    }
    if (reset_pending_) {
      const double now = monotonic_seconds();
      if (now - last_reset_wait_log_ >= 2.0) {
        last_reset_wait_log_ = now;
        const int64_t now_ns = this->get_clock()->now().nanoseconds();
        const double rx_age =
            last_feedback_receive_ns_
                ? static_cast<double>(now_ns - last_feedback_receive_ns_) / 1e6
                : -1.0;
        const double src_age =
            last_feedback_source_ns_
                ? static_cast<double>(now_ns - last_feedback_source_ns_) / 1e6
                : -1.0;
        RCLCPP_WARN(
            this->get_logger(),
            "solver reset waiting for fresh feedback: cutoff_ns=%ld, "
            "feedback_received=%ld, receive_age_ms=%.3f, source_age_ms=%.3f; "
            "output remains blocked",
            static_cast<long>(reset_cutoff_ns_),
            static_cast<long>(feedback_received_total_), rx_age, src_age);
      }
      return;
    }
    if (!owned && (control_source_ != "vr" || source_target_pending_)) {
      return;
    }
    if (owned && iv_homing_) {
      if (iv_home_command_.has_value()) {
        owned_output_pub_->publish(*iv_home_command_);
        iv_home_command_.reset();
      }
      return;
    }

    std::vector<double> held_q;
    if (clutch_held_ && !owned) {
      held_q.resize(clutch_q_indices_.size());
      for (std::size_t i = 0; i < clutch_q_indices_.size(); ++i) {
        held_q[i] = controller_->command_q()[clutch_q_indices_[i]];
      }
    }

    std::vector<std::string> names;
    Eigen::VectorXd positions;
    try {
      if (!controller_->have_feedback()) {
        return;
      }
      std::tie(names, positions) = controller_->step();
      if (!held_q.empty()) {
        for (std::size_t i = 0; i < clutch_columns_.size(); ++i) {
          controller_->command_q()[clutch_q_indices_[i]] = held_q[i];
          controller_->previous_velocity()[clutch_columns_[i]] = 0.0;
          positions[clutch_columns_[i]] = held_q[i];
        }
      }
    } catch (const std::exception& exc) {
      ++failure_count_;
      const double now = monotonic_seconds();
      if (now - last_failure_log_ >= 1.0) {
        last_failure_log_ = now;
        RCLCPP_ERROR(this->get_logger(),
                     "controller step failed; holding feedback: %s",
                     exc.what());
      }
      return;
    }

    failure_count_ = 0;
    sensor_msgs::msg::JointState message;
    message.header.stamp = this->get_clock()->now();
    if (clutch_held_ && !owned) {
      message.header.frame_id = "solver_clutch_hold";
    }
    message.name = std::move(names);
    message.position.resize(static_cast<std::size_t>(positions.size()));
    for (Eigen::Index i = 0; i < positions.size(); ++i) {
      message.position[static_cast<std::size_t>(i)] = positions[i];
    }

    if (owned) {
      publish_owned(message);
      if (record_joint_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::JointState rec;
        rec.header.stamp = message.header.stamp;
        rec.name = message.name;
        rec.position = message.position;
        record_joint_pub_->publish(rec);
      }
    } else {
      command_pub_->publish(message);
    }
    if (solver_pub_) {
      solver_pub_->publish(message);
    }
  }

  std::unique_ptr<ControllerV23> controller_;
  std::unordered_set<std::string> free_joint_set_;
  std::vector<TargetStore::Key> ee_pose_order_;
  std::vector<int> clutch_columns_;
  std::vector<int> clutch_q_indices_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr solver_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr reset_ack_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ee_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_source_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr clutch_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // Intervention pubs/subs/state
  bool intervention_enabled_ = false;
  std::string intervention_instance_;
  bool iv_active_ = false;
  std::optional<ControlToken> iv_token_;
  std::string iv_phase_;
  bool iv_ready_ = false;
  std::string iv_error_;
  int64_t iv_permit_stamp_ = 0;
  int64_t iv_cutoff_ = 0;
  double iv_permit_time_ = -std::numeric_limits<double>::infinity();
  double iv_feedback_time_ = -std::numeric_limits<double>::infinity();
  double iv_target_time_ = -std::numeric_limits<double>::infinity();
  int64_t iv_target_stamp_ = 0;
  bool iv_homing_ = false;
  std::optional<sensor_msgs::msg::JointState> iv_home_command_;
  std::optional<sensor_msgs::msg::JointState> iv_auxiliary_;
  std::unordered_map<std::string, double> gripper_limits_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr record_ee_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr record_joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr owned_output_pub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr permit_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr owned_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr owned_home_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr owned_aux_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  int failure_count_ = 0;
  double last_failure_log_ = 0.0;
  bool reset_pending_ = false;
  bool clutch_held_ = false;
  bool clutch_reset_pending_ = false;
  int64_t clutch_event_ns_ = 0;
  int64_t clutch_feedback_cutoff_ns_ = 0;
  int64_t feedback_received_total_ = 0;
  int64_t last_feedback_receive_ns_ = 0;
  int64_t last_feedback_source_ns_ = 0;
  double last_reset_wait_log_ = -std::numeric_limits<double>::infinity();
  std::unordered_map<std::string, int64_t> feedback_rejections_;
  double last_feedback_rejection_log_ = -std::numeric_limits<double>::infinity();
  int64_t reset_cutoff_ns_ = 0;
  std::string control_source_ = "vr";
  bool source_target_pending_ = false;
  int64_t source_target_cutoff_ns_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  std::vector<char*> ros_argv;
  ros_argv.push_back(argv[0]);
  bool in_ros_args = false;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--ros-args") {
      in_ros_args = true;
    }
    if (!in_ros_args && config_path.empty() && !arg.empty() && arg[0] != '-') {
      config_path = arg;
    } else {
      ros_argv.push_back(argv[i]);
    }
  }
  if (config_path.empty()) {
    std::cerr << "Usage: controller_v2_3_ros2_cpp <controller_v23.yml> "
                 "[--ros-args ...]\n";
    return 2;
  }

  const int ros_argc = static_cast<int>(ros_argv.size());
  rclcpp::init(ros_argc, ros_argv.data());
  try {
    auto node = std::make_shared<ControllerV23Node>(config_path);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception& exc) {
    std::cerr << "controller_v2_3_ros2_cpp fatal error: " << exc.what() << "\n";
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
