#include "hc_diagnostics/control_chain_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace hc_diagnostics
{
namespace
{

double milliseconds(const SteadyTime newer, const SteadyTime older)
{
  return std::chrono::duration<double, std::milli>(newer - older).count();
}

bool positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

class ControlChainMonitor::StreamMetric
{
public:
  void observe(const SteadyTime now)
  {
    if (last_) {
      max_gap_ms_ = std::max(max_gap_ms_, milliseconds(now, *last_));
    }
    last_ = now;
    timestamps_.push_back(now);
    ++total_;
    trim(now);
  }

  StreamSnapshot snapshot(const SteadyTime now, const double stale_ms)
  {
    trim(now);
    StreamSnapshot result;
    result.total = total_;
    result.max_gap_ms = max_gap_ms_;
    if (last_) {
      result.age_ms = std::max(0.0, milliseconds(now, *last_));
      result.stale = result.age_ms > stale_ms;
    }
    if (timestamps_.size() >= 2U) {
      const double span = std::chrono::duration<double>(
        timestamps_.back() - timestamps_.front()).count();
      if (span > 0.0) {
        result.hz = static_cast<double>(timestamps_.size() - 1U) / span;
      }
    }
    return result;
  }

private:
  void trim(const SteadyTime now)
  {
    const auto cutoff = now - std::chrono::seconds(2);
    while (timestamps_.size() > 1U && timestamps_.front() < cutoff) {
      timestamps_.pop_front();
    }
  }

  std::deque<SteadyTime> timestamps_;
  std::optional<SteadyTime> last_;
  double max_gap_ms_{0.0};
  std::uint64_t total_{0U};
};

class ControlChainMonitor::LatencyWindow
{
public:
  void observe(const double value)
  {
    if (!std::isfinite(value) || value < 0.0) {
      return;
    }
    values_.push_back(value);
    if (values_.size() > 1000U) {
      values_.pop_front();
    }
    ++total_;
    sum_ += value;
    last_ = value;
    maximum_ = std::max(maximum_, value);
  }

  LatencySnapshot snapshot() const
  {
    LatencySnapshot result;
    result.samples = total_;
    result.last_ms = last_;
    result.mean_ms = total_ == 0U ? 0.0 : sum_ / static_cast<double>(total_);
    result.max_ms = maximum_;
    if (!values_.empty()) {
      std::vector<double> sorted(values_.begin(), values_.end());
      const auto index = static_cast<std::size_t>(
        std::floor(0.95 * static_cast<double>(sorted.size() - 1U)));
      std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
      result.p95_ms = sorted[index];
    }
    return result;
  }

private:
  std::deque<double> values_;
  std::uint64_t total_{0U};
  double sum_{0.0};
  double last_{0.0};
  double maximum_{0.0};
};

ControlChainMonitor::ControlChainMonitor(MonitorConfig config)
: config_(std::move(config)),
  vr_to_target_(std::make_unique<LatencyWindow>()),
  vr_receive_gap_(std::make_unique<LatencyWindow>()),
  vr_callback_delay_(std::make_unique<LatencyWindow>()),
  target_to_candidate_(std::make_unique<LatencyWindow>()),
  candidate_to_command_(std::make_unique<LatencyWindow>()),
  vr_to_command_(std::make_unique<LatencyWindow>())
{
  for (const double value : {config_.vr_receive_gap_warn_ms,
      config_.vr_callback_delay_warn_ms, config_.ik_latency_warn_ms,
      config_.arbiter_latency_warn_ms, config_.joint_step_warn_rad,
      config_.missing_candidate_warn_ms, config_.stream_stale_ms,
      config_.trace_retention_ms})
  {
    if (!positive_finite(value)) {
      throw std::invalid_argument("control chain monitor thresholds must be positive and finite");
    }
  }
  if (config_.trace_retention_ms <= config_.missing_candidate_warn_ms) {
    throw std::invalid_argument("trace retention must exceed the missing-candidate threshold");
  }
  for (const auto * name : {"vr", "targets", "candidates", "commands", "joints", "cartesian"}) {
    streams_.emplace(name, std::make_unique<StreamMetric>());
  }
}

ControlChainMonitor::~ControlChainMonitor() = default;

std::vector<Anomaly> ControlChainMonitor::observeVr(
  const std::string & session_id, const std::uint64_t sequence,
  const VrTiming timing, const SteadyTime now)
{
  std::vector<Anomaly> anomalies;
  if (std::isfinite(timing.receive_time_ms) && timing.receive_time_ms >= 0.0) {
    if (last_vr_receive_time_ms_ && last_vr_receive_session_ == session_id &&
      timing.receive_time_ms > *last_vr_receive_time_ms_)
    {
      const double gap = timing.receive_time_ms - *last_vr_receive_time_ms_;
      vr_receive_gap_->observe(gap);
      if (gap > config_.vr_receive_gap_warn_ms) {
        auto accepted = acceptAnomaly(
          {now, "vr_receive_gap", session_id, sequence, "", gap,
            config_.vr_receive_gap_warn_ms});
        anomalies.insert(anomalies.end(), accepted.begin(), accepted.end());
      }
    }
    last_vr_receive_time_ms_ = timing.receive_time_ms;
    last_vr_receive_session_ = session_id;
  }
  if (std::isfinite(timing.callback_delay_ms) && timing.callback_delay_ms >= 0.0) {
    vr_callback_delay_->observe(timing.callback_delay_ms);
    if (timing.callback_delay_ms > config_.vr_callback_delay_warn_ms) {
      auto accepted = acceptAnomaly(
        {now, "vr_callback_delay", session_id, sequence, "", timing.callback_delay_ms,
          config_.vr_callback_delay_warn_ms});
      anomalies.insert(anomalies.end(), accepted.begin(), accepted.end());
    }
  }
  streams_.at("vr")->observe(now);
  auto & trace = traces_[key(session_id, sequence)];
  trace.session_id = session_id;
  trace.sequence = sequence;
  trace.vr_at = now;
  trace.updated_at = now;
  prune(now);
  return anomalies;
}

void ControlChainMonitor::observeTarget(
  const std::string & session_id, const std::uint64_t sequence,
  const std::vector<std::string> & groups, const SteadyTime now)
{
  streams_.at("targets")->observe(now);
  auto & trace = traces_[key(session_id, sequence)];
  trace.session_id = session_id;
  trace.sequence = sequence;
  if (!trace.target_at) {
    trace.target_at = now;
    if (trace.vr_at) {
      vr_to_target_->observe(milliseconds(now, *trace.vr_at));
    }
  }
  trace.target_groups = groups;
  trace.updated_at = now;
  prune(now);
}

std::vector<Anomaly> ControlChainMonitor::observeCandidate(
  const std::string & session_id, const std::uint64_t sequence,
  const std::string & group_name, const std::vector<double> & positions, const SteadyTime now)
{
  std::vector<Anomaly> anomalies;
  streams_.at("candidates")->observe(now);
  auto & trace = traces_[key(session_id, sequence)];
  trace.session_id = session_id;
  trace.sequence = sequence;
  if (trace.candidate_at.emplace(group_name, now).second && trace.target_at) {
    const double latency = milliseconds(now, *trace.target_at);
    target_to_candidate_->observe(latency);
    if (latency > config_.ik_latency_warn_ms) {
      auto accepted = acceptAnomaly(
        {now, "ik_latency", session_id, sequence, group_name, latency,
          config_.ik_latency_warn_ms});
      anomalies.insert(anomalies.end(), accepted.begin(), accepted.end());
    }
  }
  trace.updated_at = now;

  const auto previous = last_candidates_.find(group_name);
  if (previous != last_candidates_.end() && previous->second.session_id == session_id &&
    previous->second.positions.size() == positions.size())
  {
    double squared = 0.0;
    for (std::size_t index = 0; index < positions.size(); ++index) {
      const double delta = positions[index] - previous->second.positions[index];
      squared += delta * delta;
    }
    const double step = std::sqrt(squared);
    candidate_steps_[group_name] = step;
    if (std::isfinite(step) && step > config_.joint_step_warn_rad) {
      auto accepted = acceptAnomaly(
        {now, "ik_joint_step", session_id, sequence, group_name, step,
          config_.joint_step_warn_rad});
      anomalies.insert(anomalies.end(), accepted.begin(), accepted.end());
    }
  }
  last_candidates_[group_name] = {session_id, {}, positions};
  prune(now);
  return anomalies;
}

std::vector<Anomaly> ControlChainMonitor::observeCommand(
  const std::string & session_id, const std::uint64_t sequence,
  const std::string & group_name, const SteadyTime now)
{
  std::vector<Anomaly> anomalies;
  streams_.at("commands")->observe(now);
  auto & trace = traces_[key(session_id, sequence)];
  trace.session_id = session_id;
  trace.sequence = sequence;
  if (trace.command_at.emplace(group_name, now).second) {
    const auto candidate = trace.candidate_at.find(group_name);
    if (candidate != trace.candidate_at.end()) {
      const double latency = milliseconds(now, candidate->second);
      candidate_to_command_->observe(latency);
      if (latency > config_.arbiter_latency_warn_ms) {
        auto accepted = acceptAnomaly(
          {now, "arbiter_latency", session_id, sequence, group_name, latency,
            config_.arbiter_latency_warn_ms});
        anomalies.insert(anomalies.end(), accepted.begin(), accepted.end());
      }
    }
    if (trace.vr_at) {
      vr_to_command_->observe(milliseconds(now, *trace.vr_at));
    }
  }
  trace.updated_at = now;
  prune(now);
  return anomalies;
}

void ControlChainMonitor::observeJointFeedback(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  const SteadyTime now)
{
  streams_.at("joints")->observe(now);
  const std::size_t count = std::min(names.size(), positions.size());
  std::unordered_map<std::string, double> measured;
  measured.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    if (std::isfinite(positions[index])) {
      measured[names[index]] = positions[index];
    }
  }
  for (const auto & entry : command_references_) {
    const auto & reference = entry.second;
    if (reference.names.size() != reference.positions.size()) {
      continue;
    }
    double squared = 0.0;
    bool complete = true;
    for (std::size_t index = 0; index < reference.names.size(); ++index) {
      const auto found = measured.find(reference.names[index]);
      if (found == measured.end()) {
        complete = false;
        break;
      }
      const double delta = reference.positions[index] - found->second;
      squared += delta * delta;
    }
    if (complete) {
      feedback_errors_[entry.first] = std::sqrt(squared);
    }
  }
}

void ControlChainMonitor::setCommandReference(
  const std::string & group_name, const std::vector<std::string> & names,
  const std::vector<double> & positions)
{
  command_references_[group_name] = {"", names, positions};
}

void ControlChainMonitor::observeCartesianState(const SteadyTime now)
{
  streams_.at("cartesian")->observe(now);
}

std::vector<Anomaly> ControlChainMonitor::tick(const SteadyTime now)
{
  std::vector<Anomaly> anomalies;
  for (auto & entry : traces_) {
    auto & trace = entry.second;
    if (!trace.target_at || trace.missing_reported || trace.target_groups.empty() ||
      milliseconds(now, *trace.target_at) <= config_.missing_candidate_warn_ms)
    {
      continue;
    }
    std::vector<std::string> missing;
    for (const auto & group : trace.target_groups) {
      if (trace.candidate_at.find(group) == trace.candidate_at.end()) {
        missing.push_back(group);
      }
    }
    if (!missing.empty()) {
      trace.missing_reported = true;
      for (const auto & group : missing) {
        auto accepted = acceptAnomaly(
          {now, "missing_ik_candidate", trace.session_id, trace.sequence, group,
            milliseconds(now, *trace.target_at), config_.missing_candidate_warn_ms});
        anomalies.insert(anomalies.end(), accepted.begin(), accepted.end());
      }
    }
  }
  prune(now);
  return anomalies;
}

MonitorSnapshot ControlChainMonitor::snapshot(const SteadyTime now) const
{
  MonitorSnapshot result;
  for (const auto & entry : streams_) {
    result.streams[entry.first] = entry.second->snapshot(now, config_.stream_stale_ms);
  }
  result.vr_to_target = vr_to_target_->snapshot();
  result.vr_receive_gap = vr_receive_gap_->snapshot();
  result.vr_callback_delay = vr_callback_delay_->snapshot();
  result.target_to_candidate = target_to_candidate_->snapshot();
  result.candidate_to_command = candidate_to_command_->snapshot();
  result.vr_to_command = vr_to_command_->snapshot();
  result.candidate_step_rad = candidate_steps_;
  result.feedback_error_rad = feedback_errors_;
  result.anomaly_count = anomaly_count_;
  result.last_anomaly = last_anomaly_;
  if (last_anomaly_) {
    result.last_anomaly_age_ms = std::max(0.0, milliseconds(now, last_anomaly_->at));
  }
  return result;
}

std::string ControlChainMonitor::key(
  const std::string & session_id, const std::uint64_t sequence) const
{
  return session_id + '\x1f' + std::to_string(sequence);
}

std::vector<Anomaly> ControlChainMonitor::acceptAnomaly(Anomaly anomaly)
{
  ++anomaly_count_;
  last_anomaly_ = anomaly;
  return {std::move(anomaly)};
}

void ControlChainMonitor::prune(const SteadyTime now)
{
  for (auto iterator = traces_.begin(); iterator != traces_.end();) {
    if (milliseconds(now, iterator->second.updated_at) > config_.trace_retention_ms) {
      iterator = traces_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

}  // namespace hc_diagnostics
