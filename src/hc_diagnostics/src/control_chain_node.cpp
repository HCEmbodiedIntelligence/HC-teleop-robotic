#include "hc_diagnostics/control_chain_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hc_diagnostics/control_chain_monitor.hpp"
#include "hc_teleop_interfaces/msg/cartesian_state_array.hpp"
#include "hc_teleop_interfaces/msg/cartesian_target_array.hpp"
#include "hc_teleop_interfaces/msg/joint_command.hpp"
#include "hc_teleop_interfaces/msg/joint_command_candidate.hpp"
#include "hc_teleop_interfaces/msg/safety_state.hpp"
#include "hc_teleop_interfaces/msg/vr_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hc_diagnostics
{
namespace
{

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

double positive(const double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive and finite");
  }
  return value;
}

std::string escape_json(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  return output.str();
}

std::string json_array(const std::vector<double> & values)
{
  std::ostringstream output;
  output << '[' << std::setprecision(8);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    output << values[index];
  }
  output << ']';
  return output.str();
}

std::string json_array(const std::vector<std::string> & values)
{
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    output << '"' << escape_json(values[index]) << '"';
  }
  output << ']';
  return output.str();
}

template<typename T>
diagnostic_msgs::msg::KeyValue value(const std::string & key, const T & item)
{
  std::ostringstream text;
  text << item;
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = text.str();
  return result;
}

std::string timestamp_name()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&raw, &local);
  std::ostringstream output;
  output << std::put_time(&local, "%Y%m%d_%H%M%S");
  return output.str();
}

}  // namespace

class ControlChainNode::Impl
{
public:
  explicit Impl(ControlChainNode & node)
  : node_(node), monitor_(configuration())
  {
    log_directory_ = node_.declare_parameter<std::string>("log_directory", "");
    pre_window_ = std::chrono::duration<double>(positive(
        node_.declare_parameter<double>("capture_pre_seconds", 5.0), "capture_pre_seconds"));
    post_window_ = std::chrono::duration<double>(positive(
        node_.declare_parameter<double>("capture_post_seconds", 3.0), "capture_post_seconds"));
    summary_period_ = std::chrono::duration<double>(positive(
        node_.declare_parameter<double>("summary_period_seconds", 5.0), "summary_period_seconds"));
    capture_cooldown_ = std::chrono::duration<double>(positive(
        node_.declare_parameter<double>("capture_cooldown_seconds", 5.0),
        "capture_cooldown_seconds"));
    output_topic_ = node_.declare_parameter<std::string>(
      "diagnostics_topic", "diagnostics/control_chain");
    if (output_topic_.empty()) {
      throw std::invalid_argument("diagnostics_topic must not be empty");
    }
    open_log();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(16);
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(32));
    command_qos.best_effort().durability_volatile();
    auto safety_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    safety_qos.reliable().transient_local();
    publisher_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());
    vr_subscription_ = node_.create_subscription<hc_teleop_interfaces::msg::VrFrame>(
      "input/vr_frame", sensor_qos,
      [this](const hc_teleop_interfaces::msg::VrFrame::ConstSharedPtr message) {
        on_vr(*message);
      });
    target_subscription_ =
      node_.create_subscription<hc_teleop_interfaces::msg::CartesianTargetArray>(
      "teleop/cartesian_targets", sensor_qos,
      [this](const hc_teleop_interfaces::msg::CartesianTargetArray::ConstSharedPtr message) {
        on_target(*message);
      });
    candidate_subscription_ =
      node_.create_subscription<hc_teleop_interfaces::msg::JointCommandCandidate>(
      "motion/backend/joint_candidate", command_qos,
      [this](const hc_teleop_interfaces::msg::JointCommandCandidate::ConstSharedPtr message) {
        on_candidate(*message);
      });
    command_subscription_ = node_.create_subscription<hc_teleop_interfaces::msg::JointCommand>(
      "control/joint_command", command_qos,
      [this](const hc_teleop_interfaces::msg::JointCommand::ConstSharedPtr message) {
        on_command(*message);
      });
    joint_subscription_ = node_.create_subscription<sensor_msgs::msg::JointState>(
      "state/joints", sensor_qos,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        on_joints(*message);
      });
    cartesian_subscription_ =
      node_.create_subscription<hc_teleop_interfaces::msg::CartesianStateArray>(
      "state/cartesian", sensor_qos,
      [this](const hc_teleop_interfaces::msg::CartesianStateArray::ConstSharedPtr message) {
        on_cartesian(*message);
      });
    safety_subscription_ = node_.create_subscription<hc_teleop_interfaces::msg::SafetyState>(
      "safety/state", safety_qos,
      [this](const hc_teleop_interfaces::msg::SafetyState::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        safety_enabled_ = message->enabled;
        safety_state_ = message->state;
        active_session_ = message->active_session;
        record("safety", message->active_session, 0U, "",
          "\"enabled\":" + std::string(message->enabled ? "true" : "false") +
          ",\"state\":" + std::to_string(message->state));
      });
    timer_ = node_.create_wall_timer(1s, [this]() {on_timer();});
    last_summary_ = SteadyTime::clock::now();
    RCLCPP_INFO(
      node_.get_logger(), "control-chain diagnostics ready: topic=%s log=%s",
      output_topic_.c_str(), log_path_.empty() ? "disabled" : log_path_.c_str());
  }

  ~Impl()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    finish_capture(SteadyTime::clock::now(), true);
    write_summary(SteadyTime::clock::now());
    RCLCPP_INFO(
      node_.get_logger(), "control-chain diagnostics stopped: anomalies=%llu log=%s",
      static_cast<unsigned long long>(monitor_.snapshot(SteadyTime::clock::now()).anomaly_count),
      log_path_.empty() ? "disabled" : log_path_.c_str());
  }

private:
  struct Event
  {
    SteadyTime at;
    std::string json;
  };

  MonitorConfig configuration()
  {
    MonitorConfig config;
    config.vr_receive_gap_warn_ms = positive(
      node_.declare_parameter<double>("vr_receive_gap_warn_ms", 80.0),
      "vr_receive_gap_warn_ms");
    config.vr_callback_delay_warn_ms = positive(
      node_.declare_parameter<double>("vr_callback_delay_warn_ms", 20.0),
      "vr_callback_delay_warn_ms");
    config.ik_latency_warn_ms = positive(
      node_.declare_parameter<double>("ik_latency_warn_ms", 20.0), "ik_latency_warn_ms");
    config.arbiter_latency_warn_ms = positive(
      node_.declare_parameter<double>("arbiter_latency_warn_ms", 20.0),
      "arbiter_latency_warn_ms");
    config.joint_step_warn_rad = positive(
      node_.declare_parameter<double>("joint_step_warn_rad", 0.25), "joint_step_warn_rad");
    config.missing_candidate_warn_ms = positive(
      node_.declare_parameter<double>("missing_candidate_warn_ms", 80.0),
      "missing_candidate_warn_ms");
    config.stream_stale_ms = positive(
      node_.declare_parameter<double>("stream_stale_ms", 350.0), "stream_stale_ms");
    config.trace_retention_ms = positive(
      node_.declare_parameter<double>("trace_retention_ms", 2000.0), "trace_retention_ms");
    return config;
  }

  void open_log()
  {
    if (log_directory_.empty()) {
      return;
    }
    const std::filesystem::path directory(log_directory_);
    std::filesystem::create_directories(directory);
    const auto path = directory / ("control_chain_" + timestamp_name() + ".jsonl");
    log_.open(path, std::ios::out | std::ios::app);
    if (!log_) {
      throw std::runtime_error("unable to open diagnostics log: " + path.string());
    }
    log_path_ = path.string();
  }

  void on_vr(const hc_teleop_interfaces::msg::VrFrame & message)
  {
    const auto callback_steady = SteadyTime::clock::now();
    const std::int64_t received_ns =
      static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(message.header.stamp.nanosec);
    const std::int64_t callback_ns = node_.now().nanoseconds();
    VrTiming timing;
    if (received_ns > 0) {
      timing.receive_time_ms = static_cast<double>(received_ns) / 1.0e6;
      if (callback_ns >= received_ns) {
        timing.callback_delay_ms = static_cast<double>(callback_ns - received_ns) / 1.0e6;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream payload;
    payload << "\"loss_total\":" << message.packet_loss_total <<
      ",\"gateway_receive_ms\":" << timing.receive_time_ms <<
      ",\"callback_delay_ms\":" << timing.callback_delay_ms <<
      ",\"left_grip\":" << message.left_input.grip <<
      ",\"right_grip\":" << message.right_input.grip <<
      ",\"left_trigger\":" << message.left_input.trigger <<
      ",\"right_trigger\":" << message.right_input.trigger;
    record("vr", message.source_id, message.sequence, "", payload.str());
    handle(monitor_.observeVr(
        message.source_id, message.sequence, timing, callback_steady));
  }

  void on_target(const hc_teleop_interfaces::msg::CartesianTargetArray & message)
  {
    const auto now = SteadyTime::clock::now();
    std::vector<std::string> groups;
    groups.reserve(message.targets.size());
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & target : message.targets) {
      groups.push_back(target.group_name);
      std::ostringstream payload;
      payload << "\"position\":[" << target.pose.position.x << ',' << target.pose.position.y <<
        ',' << target.pose.position.z << ']';
      record("target", message.session_id, message.sequence, target.group_name, payload.str());
    }
    monitor_.observeTarget(message.session_id, message.sequence, groups, now);
  }

  void on_candidate(const hc_teleop_interfaces::msg::JointCommandCandidate & message)
  {
    const auto now = SteadyTime::clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    record("candidate", message.session_id, message.sequence, message.group_name,
      "\"positions\":" + json_array(message.command.position));
    handle(monitor_.observeCandidate(
        message.session_id, message.sequence, message.group_name,
        message.command.position, now));
  }

  void on_command(const hc_teleop_interfaces::msg::JointCommand & message)
  {
    const auto now = SteadyTime::clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    monitor_.setCommandReference(
      message.group_name, message.command.name, message.command.position);
    record("command", message.session_id, message.sequence, message.group_name,
      "\"names\":" + json_array(message.command.name) +
      ",\"positions\":" + json_array(message.command.position));
    handle(monitor_.observeCommand(
        message.session_id, message.sequence, message.group_name, now));
  }

  void on_joints(const sensor_msgs::msg::JointState & message)
  {
    const auto now = SteadyTime::clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    monitor_.observeJointFeedback(message.name, message.position, now);
    if (!last_feedback_event_ || now - *last_feedback_event_ >= 50ms) {
      last_feedback_event_ = now;
      record("feedback", active_session_, 0U, "",
        "\"names\":" + json_array(message.name) +
        ",\"positions\":" + json_array(message.position));
    }
  }

  void on_cartesian(const hc_teleop_interfaces::msg::CartesianStateArray & message)
  {
    (void)message;
    std::lock_guard<std::mutex> lock(mutex_);
    monitor_.observeCartesianState(SteadyTime::clock::now());
  }

  void on_timer()
  {
    const auto now = SteadyTime::clock::now();
    diagnostic_msgs::msg::DiagnosticArray output;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handle(monitor_.tick(now));
      finish_capture(now, false);
      if (now - last_summary_ >= summary_period_) {
        write_summary(now);
        last_summary_ = now;
      }
      output = diagnostics(monitor_.snapshot(now));
    }
    publisher_->publish(std::move(output));
  }

  void record(
    const std::string & stage, const std::string & session_id, const std::uint64_t sequence,
    const std::string & group_name, const std::string & payload)
  {
    const auto now = SteadyTime::clock::now();
    std::ostringstream json;
    json << "{\"stage\":\"" << escape_json(stage) << "\",\"session\":\"" <<
      escape_json(session_id) << "\",\"sequence\":" << sequence <<
      ",\"group\":\"" << escape_json(group_name) << '"';
    if (!payload.empty()) {
      json << ',' << payload;
    }
    json << '}';
    const Event event{now, json.str()};
    ring_.push_back(event);
    const auto cutoff = now - std::chrono::duration_cast<SteadyTime::duration>(pre_window_);
    while (!ring_.empty() && ring_.front().at < cutoff) {
      ring_.pop_front();
    }
    if (capture_active_) {
      capture_.push_back(event);
    }
  }

  void handle(const std::vector<Anomaly> & anomalies)
  {
    const auto now = SteadyTime::clock::now();
    for (const auto & anomaly : anomalies) {
      std::ostringstream payload;
      payload << "\"code\":\"" << escape_json(anomaly.code) << "\",\"value\":" <<
        anomaly.value << ",\"threshold\":" << anomaly.threshold;
      record("anomaly", anomaly.session_id, anomaly.sequence, anomaly.group_name, payload.str());
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "control-chain anomaly %s group=%s seq=%llu value=%.3f threshold=%.3f",
        anomaly.code.c_str(), anomaly.group_name.c_str(),
        static_cast<unsigned long long>(anomaly.sequence), anomaly.value, anomaly.threshold);
      if (!capture_active_ &&
        (!last_capture_ || now - *last_capture_ >= capture_cooldown_))
      {
        capture_active_ = true;
        capture_end_ = now + std::chrono::duration_cast<SteadyTime::duration>(post_window_);
        capture_.assign(ring_.begin(), ring_.end());
        capture_reason_ = anomaly.code;
      }
    }
  }

  void finish_capture(const SteadyTime now, const bool force)
  {
    if (!capture_active_ || (!force && now < capture_end_)) {
      return;
    }
    if (log_) {
      log_ << "{\"type\":\"anomaly_capture\",\"reason\":\"" <<
        escape_json(capture_reason_) << "\",\"events\":[";
      for (std::size_t index = 0; index < capture_.size(); ++index) {
        if (index > 0U) {
          log_ << ',';
        }
        const double offset = std::chrono::duration<double, std::milli>(
          capture_[index].at - capture_.front().at).count();
        log_ << "{\"offset_ms\":" << offset << ",\"data\":" << capture_[index].json << '}';
      }
      log_ << "]}\n";
      log_.flush();
    }
    capture_.clear();
    capture_active_ = false;
    last_capture_ = now;
  }

  void write_summary(const SteadyTime now)
  {
    if (!log_) {
      return;
    }
    const auto snapshot = monitor_.snapshot(now);
    log_ << "{\"type\":\"summary\",\"anomalies\":" << snapshot.anomaly_count <<
      ",\"safety_enabled\":" << (safety_enabled_ ? "true" : "false") <<
      ",\"streams\":{";
    bool first = true;
    for (const auto & entry : snapshot.streams) {
      if (!first) {
        log_ << ',';
      }
      first = false;
      log_ << '"' << escape_json(entry.first) << "\":{\"hz\":" << entry.second.hz <<
        ",\"age_ms\":" << entry.second.age_ms << ",\"max_gap_ms\":" <<
        entry.second.max_gap_ms << ",\"total\":" << entry.second.total << '}';
    }
    log_ << "},\"latency_ms\":{\"vr_receive_gap_p95\":" <<
      snapshot.vr_receive_gap.p95_ms <<
      ",\"vr_callback_delay_p95\":" << snapshot.vr_callback_delay.p95_ms <<
      ",\"vr_target_p95\":" << snapshot.vr_to_target.p95_ms <<
      ",\"ik_p95\":" << snapshot.target_to_candidate.p95_ms <<
      ",\"arbiter_p95\":" << snapshot.candidate_to_command.p95_ms <<
      ",\"end_to_end_p95\":" << snapshot.vr_to_command.p95_ms << "}}\n";
    log_.flush();
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics(const MonitorSnapshot & snapshot)
  {
    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = node_.now();
    DiagnosticStatus overview;
    overview.name = node_.get_fully_qualified_name() + std::string("/control_chain");
    overview.hardware_id = "hc_teleop";
    const bool recent_anomaly = snapshot.last_anomaly && snapshot.last_anomaly_age_ms < 10000.0;
    overview.level = recent_anomaly ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    overview.message = recent_anomaly ? snapshot.last_anomaly->code : "control chain healthy";
    overview.values.push_back(value("anomaly_count", snapshot.anomaly_count));
    overview.values.push_back(value("safety_enabled", safety_enabled_));
    overview.values.push_back(value("safety_state", static_cast<int>(safety_state_)));
    overview.values.push_back(value("log_path", log_path_));
    if (snapshot.last_anomaly) {
      overview.values.push_back(value("last_anomaly_code", snapshot.last_anomaly->code));
      overview.values.push_back(value("last_anomaly_group", snapshot.last_anomaly->group_name));
      overview.values.push_back(value("last_anomaly_sequence", snapshot.last_anomaly->sequence));
      overview.values.push_back(value("last_anomaly_value", snapshot.last_anomaly->value));
      overview.values.push_back(value("last_anomaly_age_ms", snapshot.last_anomaly_age_ms));
    }
    output.status.push_back(std::move(overview));

    DiagnosticStatus streams;
    streams.name = node_.get_fully_qualified_name() + std::string("/streams");
    streams.hardware_id = "hc_teleop";
    streams.level = DiagnosticStatus::OK;
    streams.message = "stream rates and freshness";
    for (const auto & entry : snapshot.streams) {
      streams.values.push_back(value(entry.first + ".hz", entry.second.hz));
      streams.values.push_back(value(entry.first + ".age_ms", entry.second.age_ms));
      streams.values.push_back(value(entry.first + ".max_gap_ms", entry.second.max_gap_ms));
      streams.values.push_back(value(entry.first + ".total", entry.second.total));
      streams.values.push_back(value(entry.first + ".stale", entry.second.stale));
      if (entry.second.stale && entry.first != "targets" && entry.first != "candidates" &&
        entry.first != "commands")
      {
        streams.level = std::max(streams.level, DiagnosticStatus::WARN);
      }
    }
    output.status.push_back(std::move(streams));

    DiagnosticStatus latency;
    latency.name = node_.get_fully_qualified_name() + std::string("/latency");
    latency.hardware_id = "hc_teleop";
    latency.level = DiagnosticStatus::OK;
    latency.message = "transport and sequence-correlated timing";
    add_latency(latency, "vr_receive_gap", snapshot.vr_receive_gap);
    add_latency(latency, "vr_callback_delay", snapshot.vr_callback_delay);
    add_latency(latency, "vr_to_target", snapshot.vr_to_target);
    add_latency(latency, "target_to_candidate", snapshot.target_to_candidate);
    add_latency(latency, "candidate_to_command", snapshot.candidate_to_command);
    add_latency(latency, "vr_to_command", snapshot.vr_to_command);
    for (const auto & entry : snapshot.candidate_step_rad) {
      latency.values.push_back(value("candidate_step_rad." + entry.first, entry.second));
    }
    for (const auto & entry : snapshot.feedback_error_rad) {
      latency.values.push_back(value("feedback_error_rad." + entry.first, entry.second));
    }
    output.status.push_back(std::move(latency));
    return output;
  }

  static void add_latency(
    DiagnosticStatus & status, const std::string & name, const LatencySnapshot & metric)
  {
    status.values.push_back(value(name + ".samples", metric.samples));
    status.values.push_back(value(name + ".last_ms", metric.last_ms));
    status.values.push_back(value(name + ".mean_ms", metric.mean_ms));
    status.values.push_back(value(name + ".p95_ms", metric.p95_ms));
    status.values.push_back(value(name + ".max_ms", metric.max_ms));
  }

  ControlChainNode & node_;
  ControlChainMonitor monitor_;
  std::mutex mutex_;
  std::string log_directory_;
  std::string log_path_;
  std::string output_topic_;
  std::ofstream log_;
  std::chrono::duration<double> pre_window_{5.0};
  std::chrono::duration<double> post_window_{3.0};
  std::chrono::duration<double> summary_period_{5.0};
  std::chrono::duration<double> capture_cooldown_{5.0};
  SteadyTime last_summary_{};
  std::optional<SteadyTime> last_capture_;
  std::optional<SteadyTime> last_feedback_event_;
  bool capture_active_{false};
  SteadyTime capture_end_{};
  std::string capture_reason_;
  std::deque<Event> ring_;
  std::vector<Event> capture_;
  bool safety_enabled_{false};
  std::uint8_t safety_state_{0U};
  std::string active_session_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::VrFrame>::SharedPtr vr_subscription_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::CartesianTargetArray>::SharedPtr
    target_subscription_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::JointCommandCandidate>::SharedPtr
    candidate_subscription_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::JointCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::CartesianStateArray>::SharedPtr
    cartesian_subscription_;
  rclcpp::Subscription<hc_teleop_interfaces::msg::SafetyState>::SharedPtr safety_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

ControlChainNode::ControlChainNode(const rclcpp::NodeOptions & options)
: Node("control_chain_diagnostics", options), impl_(std::make_unique<Impl>(*this))
{
}

ControlChainNode::~ControlChainNode() = default;

}  // namespace hc_diagnostics

RCLCPP_COMPONENTS_REGISTER_NODE(hc_diagnostics::ControlChainNode)
