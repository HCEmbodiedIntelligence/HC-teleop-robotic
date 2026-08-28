#include "hc_teleop_core/command_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace hc_teleop_core
{

CommandArbiter::CommandArbiter(const Duration command_timeout)
: command_timeout_(command_timeout)
{
  if (command_timeout_ <= Duration::zero()) {
    throw std::invalid_argument("command timeout must be positive");
  }
}

void CommandArbiter::setEnabled(const bool enabled, const SteadyTime now)
{
  enabled_ = enabled;
  clearRuntimeState();
  hold_reason_ = enabled ? "waiting for a valid control lease" : "disabled";
  if (!enabled) {
    lease_.reset();
  } else {
    expireIfNeeded(now);
  }
}

void CommandArbiter::latchFault(std::string reason)
{
  fault_latched_ = true;
  fault_reason_ = reason.empty() ? "unspecified safety fault" : std::move(reason);
  clearRuntimeState();
  lease_.reset();
  hold_reason_ = fault_reason_;
}

bool CommandArbiter::resetFault()
{
  if (!fault_latched_) {
    return true;
  }
  fault_latched_ = false;
  fault_reason_.clear();
  clearRuntimeState();
  lease_.reset();
  enabled_ = false;
  hold_reason_ = "fault reset; explicit enable required";
  return true;
}

AcquireResult CommandArbiter::acquire(
  std::string source_id,
  std::string session_id,
  const std::uint8_t priority,
  const Duration duration,
  const SteadyTime now)
{
  expireIfNeeded(now);
  AcquireResult result;
  if (source_id.empty() || session_id.empty()) {
    result.reason = "source_id and session_id are required";
    return result;
  }
  if (duration <= Duration::zero()) {
    result.reason = "lease duration must be positive";
    return result;
  }
  if (fault_latched_) {
    result.reason = "safety fault is latched";
    return result;
  }
  if (leaseActive(now)) {
    const bool same_owner = lease_->source_id == source_id && lease_->session_id == session_id;
    if (!same_owner && priority <= lease_->priority) {
      result.reason = "an equal or higher-priority source owns the control lease";
      return result;
    }
  }

  clearRuntimeState();
  const auto lease_id = makeLeaseId(source_id, session_id, ++lease_nonce_);
  lease_ = ControlLease{
    lease_id, std::move(source_id), std::move(session_id), priority, now + duration};
  hold_reason_ = enabled_ ? "waiting for the lease owner's first command" : "disabled";
  result.granted = true;
  result.lease_id = lease_id;
  result.expires_at = lease_->expires_at;
  result.reason = "granted";
  return result;
}

bool CommandArbiter::release(const std::string & lease_id, const SteadyTime now)
{
  expireIfNeeded(now);
  if (!lease_ || lease_->lease_id != lease_id) {
    return false;
  }
  lease_.reset();
  clearRuntimeState();
  hold_reason_ = "control lease released";
  return true;
}

bool CommandArbiter::submit(
  JointCandidate candidate,
  const SteadyTime now,
  std::string & rejection_reason)
{
  expireIfNeeded(now);
  if (!enabled_) {
    rejection_reason = "control is disabled";
    return false;
  }
  if (fault_latched_) {
    rejection_reason = "safety fault is latched";
    return false;
  }
  if (!leaseActive(now)) {
    rejection_reason = "no active control lease";
    return false;
  }
  if (candidate.source_id != lease_->source_id || candidate.session_id != lease_->session_id) {
    rejection_reason = "candidate does not own the active lease";
    return false;
  }
  if (candidate.valid_until <= now) {
    rejection_reason = "candidate has expired";
    return false;
  }
  if (!validCandidate(candidate, rejection_reason)) {
    return false;
  }
  const auto stream = candidate.source_id + '\x1f' + candidate.session_id + '\x1f' +
    candidate.group_name;
  const auto sequence_it = last_sequence_by_stream_.find(stream);
  if (sequence_it != last_sequence_by_stream_.end() &&
    !sequenceNewer(candidate.sequence, sequence_it->second))
  {
    rejection_reason = "candidate sequence is stale or duplicated";
    return false;
  }
  last_sequence_by_stream_[stream] = candidate.sequence;
  const auto group_name = candidate.group_name;
  latest_by_group_[group_name] = TimedCandidate{std::move(candidate), now};
  hold_reason_.clear();
  rejection_reason.clear();
  return true;
}

std::vector<JointCandidate> CommandArbiter::commands(const SteadyTime now)
{
  expireIfNeeded(now);
  if (!enabled_ || fault_latched_ || !leaseActive(now)) {
    return {};
  }

  bool validity_expired = false;
  bool watchdog_expired = false;
  for (auto it = latest_by_group_.begin(); it != latest_by_group_.end();) {
    const auto & timed = it->second;
    if (timed.candidate.valid_until <= now) {
      validity_expired = true;
      it = latest_by_group_.erase(it);
    } else if (now - timed.received_at > command_timeout_) {
      watchdog_expired = true;
      it = latest_by_group_.erase(it);
    } else {
      ++it;
    }
  }

  if (latest_by_group_.empty()) {
    if (watchdog_expired) {
      hold_reason_ = "command watchdog expired";
    } else if (validity_expired) {
      hold_reason_ = "candidate validity expired";
    } else if (hold_reason_.empty()) {
      hold_reason_ = "waiting for a fresh command";
    }
    return {};
  }

  std::vector<JointCandidate> selected;
  selected.reserve(latest_by_group_.size());
  for (const auto & entry : latest_by_group_) {
    selected.push_back(entry.second.candidate);
  }
  std::sort(
    selected.begin(), selected.end(),
    [](const JointCandidate & left, const JointCandidate & right) {
      return left.group_name < right.group_name;
    });
  hold_reason_.clear();
  return selected;
}

std::optional<JointCandidate> CommandArbiter::command(const SteadyTime now)
{
  auto selected = commands(now);
  if (selected.empty()) {
    return std::nullopt;
  }
  return selected.front();
}

ArbiterStatus CommandArbiter::status(const SteadyTime now) const
{
  ArbiterStatus result;
  result.enabled = enabled_;
  result.fault_latched = fault_latched_;
  if (lease_ && lease_->expires_at > now) {
    result.active_source = lease_->source_id;
    result.active_session = lease_->session_id;
    result.lease_id = lease_->lease_id;
    result.lease_expires_at = lease_->expires_at;
  }
  if (fault_latched_) {
    result.code = SafetyCode::kFault;
    result.reason = fault_reason_;
  } else if (!enabled_) {
    result.code = SafetyCode::kDisabled;
    result.reason = hold_reason_.empty() ? "disabled" : hold_reason_;
  } else if (!result.lease_id.empty() && std::any_of(
    latest_by_group_.begin(), latest_by_group_.end(),
    [this, now](const auto & entry) {
      return entry.second.candidate.valid_until > now &&
             now - entry.second.received_at <= command_timeout_;
    }))
  {
    result.code = SafetyCode::kActive;
    result.reason = "active";
  } else if (!result.lease_id.empty()) {
    result.code = SafetyCode::kHolding;
    result.reason = hold_reason_.empty() ? "waiting for a fresh command" : hold_reason_;
  } else {
    result.code = SafetyCode::kReady;
    result.reason = hold_reason_.empty() ? "waiting for a control lease" : hold_reason_;
  }
  return result;
}

bool CommandArbiter::leaseActive(const SteadyTime now) const
{
  return lease_.has_value() && lease_->expires_at > now;
}

void CommandArbiter::expireIfNeeded(const SteadyTime now)
{
  if (lease_ && lease_->expires_at <= now) {
    lease_.reset();
    clearRuntimeState();
    hold_reason_ = "control lease expired";
  }
}

void CommandArbiter::clearRuntimeState()
{
  latest_by_group_.clear();
  last_sequence_by_stream_.clear();
}

bool CommandArbiter::validCandidate(const JointCandidate & candidate, std::string & reason)
{
  if (candidate.source_id.empty() || candidate.session_id.empty()) {
    reason = "source_id and session_id are required";
    return false;
  }
  if (candidate.group_name.empty()) {
    reason = "group_name is required";
    return false;
  }
  if (candidate.names.empty()) {
    reason = "joint names must have a non-zero length";
    return false;
  }
  if (candidate.control_mode < 1 || candidate.control_mode > 3) {
    reason = "unsupported control mode";
    return false;
  }
  if ((!candidate.positions.empty() && candidate.positions.size() != candidate.names.size()) ||
    (!candidate.velocities.empty() && candidate.velocities.size() != candidate.names.size()) ||
    (!candidate.efforts.empty() && candidate.efforts.size() != candidate.names.size()))
  {
    reason = "optional velocity and effort arrays must match joint names";
    return false;
  }
  const bool required_mode_array_present =
    (candidate.control_mode == 1U && candidate.positions.size() == candidate.names.size()) ||
    (candidate.control_mode == 2U && candidate.velocities.size() == candidate.names.size()) ||
    (candidate.control_mode == 3U && candidate.efforts.size() == candidate.names.size());
  if (!required_mode_array_present) {
    reason = "the selected control mode requires its matching value array";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & name : candidate.names) {
    if (name.empty() || !names.insert(name).second) {
      reason = "joint names must be non-empty and unique";
      return false;
    }
  }
  const auto finite = [](const std::vector<double> & values) {
      return std::all_of(values.begin(), values.end(), [](const double value) {
          return std::isfinite(value);
        });
    };
  if (!finite(candidate.positions) || !finite(candidate.velocities) || !finite(candidate.efforts)) {
    reason = "joint command contains a non-finite value";
    return false;
  }
  return true;
}

bool CommandArbiter::sequenceNewer(const std::uint64_t sequence, const std::uint64_t previous)
{
  const auto difference = sequence - previous;
  return difference != 0 && difference < (std::numeric_limits<std::uint64_t>::max() / 2U + 1U);
}

std::string CommandArbiter::makeLeaseId(
  const std::string & source_id,
  const std::string & session_id,
  const std::uint64_t nonce)
{
  std::ostringstream stream;
  stream << source_id << '-' << session_id << '-' << nonce;
  return stream.str();
}

}  // namespace hc_teleop_core
