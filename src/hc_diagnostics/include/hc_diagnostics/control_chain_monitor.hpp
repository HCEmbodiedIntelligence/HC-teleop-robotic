#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace hc_diagnostics
{

using SteadyTime = std::chrono::steady_clock::time_point;

struct MonitorConfig
{
  double vr_receive_gap_warn_ms{80.0};
  double vr_callback_delay_warn_ms{20.0};
  double ik_latency_warn_ms{20.0};
  double arbiter_latency_warn_ms{20.0};
  double joint_step_warn_rad{0.25};
  double missing_candidate_warn_ms{80.0};
  double stream_stale_ms{350.0};
  double trace_retention_ms{2000.0};
};

struct VrTiming
{
  // ROS-clock milliseconds stamped by hc-vr-gateway immediately after recvfrom.
  // A negative value marks an unavailable/invalid gateway timestamp.
  double receive_time_ms{-1.0};
  // Time from the gateway stamp until this monitor callback starts.
  double callback_delay_ms{-1.0};
};

struct StreamSnapshot
{
  double hz{0.0};
  double age_ms{-1.0};
  double max_gap_ms{0.0};
  std::uint64_t total{0U};
  bool stale{true};
};

struct LatencySnapshot
{
  std::uint64_t samples{0U};
  double last_ms{0.0};
  double mean_ms{0.0};
  double p95_ms{0.0};
  double max_ms{0.0};
};

struct Anomaly
{
  SteadyTime at{};
  std::string code;
  std::string session_id;
  std::uint64_t sequence{0U};
  std::string group_name;
  double value{0.0};
  double threshold{0.0};
};

struct MonitorSnapshot
{
  std::map<std::string, StreamSnapshot> streams;
  LatencySnapshot vr_to_target;
  LatencySnapshot vr_receive_gap;
  LatencySnapshot vr_callback_delay;
  LatencySnapshot target_to_candidate;
  LatencySnapshot candidate_to_command;
  LatencySnapshot vr_to_command;
  std::map<std::string, double> candidate_step_rad;
  std::map<std::string, double> feedback_error_rad;
  std::uint64_t anomaly_count{0U};
  std::optional<Anomaly> last_anomaly;
  double last_anomaly_age_ms{-1.0};
};

class ControlChainMonitor
{
public:
  explicit ControlChainMonitor(MonitorConfig config = {});
  ~ControlChainMonitor();

  std::vector<Anomaly> observeVr(
    const std::string & session_id, std::uint64_t sequence,
    VrTiming timing, SteadyTime now);
  void observeTarget(
    const std::string & session_id, std::uint64_t sequence,
    const std::vector<std::string> & groups, SteadyTime now);
  std::vector<Anomaly> observeCandidate(
    const std::string & session_id, std::uint64_t sequence,
    const std::string & group_name, const std::vector<double> & positions, SteadyTime now);
  std::vector<Anomaly> observeCommand(
    const std::string & session_id, std::uint64_t sequence,
    const std::string & group_name, SteadyTime now);
  void observeJointFeedback(
    const std::vector<std::string> & names, const std::vector<double> & positions,
    SteadyTime now);
  void setCommandReference(
    const std::string & group_name, const std::vector<std::string> & names,
    const std::vector<double> & positions);
  void observeCartesianState(SteadyTime now);
  std::vector<Anomaly> tick(SteadyTime now);
  MonitorSnapshot snapshot(SteadyTime now) const;

private:
  class StreamMetric;
  class LatencyWindow;
  struct Trace
  {
    std::string session_id;
    std::uint64_t sequence{0U};
    std::optional<SteadyTime> vr_at;
    std::optional<SteadyTime> target_at;
    std::map<std::string, SteadyTime> candidate_at;
    std::map<std::string, SteadyTime> command_at;
    std::vector<std::string> target_groups;
    bool missing_reported{false};
    SteadyTime updated_at{};
  };
  struct JointVector
  {
    std::string session_id;
    std::vector<std::string> names;
    std::vector<double> positions;
  };

  std::string key(const std::string & session_id, std::uint64_t sequence) const;
  std::vector<Anomaly> acceptAnomaly(Anomaly anomaly);
  void prune(SteadyTime now);

  MonitorConfig config_;
  std::map<std::string, std::unique_ptr<StreamMetric>> streams_;
  std::unordered_map<std::string, Trace> traces_;
  std::map<std::string, JointVector> last_candidates_;
  std::map<std::string, JointVector> command_references_;
  std::map<std::string, double> candidate_steps_;
  std::map<std::string, double> feedback_errors_;
  std::optional<double> last_vr_receive_time_ms_;
  std::string last_vr_receive_session_;
  std::optional<Anomaly> last_anomaly_;
  std::uint64_t anomaly_count_{0U};
  std::unique_ptr<LatencyWindow> vr_to_target_;
  std::unique_ptr<LatencyWindow> vr_receive_gap_;
  std::unique_ptr<LatencyWindow> vr_callback_delay_;
  std::unique_ptr<LatencyWindow> target_to_candidate_;
  std::unique_ptr<LatencyWindow> candidate_to_command_;
  std::unique_ptr<LatencyWindow> vr_to_command_;
};

}  // namespace hc_diagnostics
