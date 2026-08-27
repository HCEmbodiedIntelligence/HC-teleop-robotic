#include "hc_motion/route_guard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace hc_motion
{
namespace
{

bool finite_values(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

}  // namespace

RouteGuard::RouteGuard(std::vector<std::string> allowed_groups)
: allowed_groups_(allowed_groups.begin(), allowed_groups.end())
{
  if (allowed_groups_.empty() || allowed_groups_.count("") != 0U ||
    allowed_groups_.size() != allowed_groups.size())
  {
    throw std::invalid_argument("allowed motion groups must be non-empty and unique");
  }
}

bool RouteGuard::route(
  const TargetEnvelope & envelope, SteadyTime now, std::string & reason)
{
  prune(now);
  if (envelope.source_id.empty() || envelope.session_id.empty() || envelope.targets.empty()) {
    reason = "source, session and at least one target are required";
    return false;
  }
  if (envelope.valid_until <= now) {
    reason = "Cartesian target has expired";
    return false;
  }
  const std::string stream = envelope.source_id + '\x1f' + envelope.session_id;
  const auto previous = last_sequence_.find(stream);
  if (previous != last_sequence_.end() && !sequenceNewer(envelope.sequence, previous->second)) {
    reason = "Cartesian target sequence is stale or duplicated";
    return false;
  }

  std::set<std::string> groups;
  for (const auto & target : envelope.targets) {
    if (allowed_groups_.count(target.group_name) == 0U) {
      reason = "target refers to a group not allowed by the robot profile";
      return false;
    }
    if (!groups.insert(target.group_name).second || target.reference_frame.empty() ||
      target.tip_frame.empty())
    {
      reason = "target groups must be unique and frames must be non-empty";
      return false;
    }
    const bool finite = std::all_of(target.position.begin(), target.position.end(),
        [](double value) {return std::isfinite(value);}) &&
      std::all_of(target.orientation.begin(), target.orientation.end(),
        [](double value) {return std::isfinite(value);});
    double quaternion_norm = 0.0;
    for (double value : target.orientation) {
      quaternion_norm += value * value;
    }
    if (!finite || quaternion_norm <= 1e-12) {
      reason = "target pose is non-finite or has a zero quaternion";
      return false;
    }
  }

  last_sequence_[stream] = envelope.sequence;
  for (const auto & target : envelope.targets) {
    authorizations_[key(envelope.source_id, envelope.session_id, target.group_name)] =
      Authorization{envelope.sequence, envelope.valid_until};
  }
  reason.clear();
  return true;
}

bool RouteGuard::authorize(
  const CandidateEnvelope & envelope, SteadyTime now, std::string & reason)
{
  prune(now);
  const auto found = authorizations_.find(
    key(envelope.source_id, envelope.session_id, envelope.group_name));
  if (found == authorizations_.end()) {
    reason = "backend candidate has no live routed target";
    return false;
  }
  if (envelope.sequence != found->second.sequence) {
    reason = "backend candidate does not answer the latest target sequence";
    return false;
  }
  constexpr auto kJitterTolerance = std::chrono::milliseconds(20);
  if (envelope.valid_until <= now || envelope.valid_until > found->second.valid_until + kJitterTolerance) {
    reason = "backend candidate validity is expired or exceeds its input target";
    return false;
  }
  if (envelope.control_mode < 1U || envelope.control_mode > 3U ||
    envelope.joint_names.empty())
  {
    reason = "backend candidate mode or joint list is invalid";
    return false;
  }
  const std::set<std::string> names(envelope.joint_names.begin(), envelope.joint_names.end());
  if (names.size() != envelope.joint_names.size() || names.count("") != 0U) {
    reason = "backend candidate joint names must be non-empty and unique";
    return false;
  }
  const auto size = envelope.joint_names.size();
  const bool array_sizes_valid =
    (envelope.positions.empty() || envelope.positions.size() == size) &&
    (envelope.velocities.empty() || envelope.velocities.size() == size) &&
    (envelope.efforts.empty() || envelope.efforts.size() == size) &&
    (envelope.control_mode != 1U || envelope.positions.size() == size) &&
    (envelope.control_mode != 2U || envelope.velocities.size() == size) &&
    (envelope.control_mode != 3U || envelope.efforts.size() == size);
  if (!array_sizes_valid || !finite_values(envelope.positions) ||
    !finite_values(envelope.velocities) || !finite_values(envelope.efforts))
  {
    reason = "backend candidate arrays are malformed or non-finite";
    return false;
  }
  reason.clear();
  return true;
}

void RouteGuard::prune(SteadyTime now)
{
  for (auto it = authorizations_.begin(); it != authorizations_.end();) {
    if (it->second.valid_until <= now) {
      it = authorizations_.erase(it);
    } else {
      ++it;
    }
  }
}

bool RouteGuard::sequenceNewer(std::uint64_t sequence, std::uint64_t previous)
{
  const std::uint64_t difference = sequence - previous;
  return difference != 0U && difference < (std::numeric_limits<std::uint64_t>::max() / 2U + 1U);
}

std::string RouteGuard::key(
  const std::string & source, const std::string & session, const std::string & group)
{
  return source + '\x1f' + session + '\x1f' + group;
}

}  // namespace hc_motion
