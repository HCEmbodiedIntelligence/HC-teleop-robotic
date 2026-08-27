#include "hc_motion/control_arbiter.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace hc_motion
{
namespace
{

bool overlaps(const std::vector<std::string> & left, const std::vector<std::string> & right)
{
  const std::set<std::string> names(left.begin(), left.end());
  return std::any_of(right.begin(), right.end(), [&names](const std::string & name) {
      return names.count(name) != 0U;
    });
}

bool contains_id(const std::vector<std::string> & values, const std::string & id)
{
  return std::find(values.begin(), values.end(), id) != values.end();
}

}  // namespace

MotionStatus ControlArbiter::registerEndpoint(const EndpointPolicy & policy)
{
  if (policy.endpoint_name.empty() || policy.group_name.empty()) {
    return {StatusCode::kInvalidArgument, "endpoint and group names must be non-empty"};
  }
  if (is_servo(policy.kind) && policy.servo_lease <= std::chrono::milliseconds::zero()) {
    return {StatusCode::kInvalidArgument, "servo lease must be positive"};
  }
  if (policies_.count(policy.endpoint_name) != 0U) {
    return {StatusCode::kInvalidArgument, "duplicate endpoint policy: " + policy.endpoint_name};
  }
  policies_.emplace(policy.endpoint_name, policy);
  return MotionStatus::Ok();
}

MotionStatus ControlArbiter::validateClaim(const CommandClaim & claim, bool servo) const
{
  if (claim.session_id.empty() || claim.endpoint_name.empty() || claim.group_name.empty()) {
    return {StatusCode::kInvalidArgument, "claim identity and group must be non-empty"};
  }
  const std::set<std::string> unique(claim.joint_names.begin(), claim.joint_names.end());
  if (claim.joint_names.empty() || unique.size() != claim.joint_names.size() ||
    unique.count("") != 0U)
  {
    return {StatusCode::kInvalidArgument, "claim joints must be non-empty and unique"};
  }
  const auto policy = policies_.find(claim.endpoint_name);
  if (policy == policies_.end()) {
    return {StatusCode::kNotConfigured, "no policy for endpoint: " + claim.endpoint_name};
  }
  if (policy->second.kind != claim.kind || policy->second.group_name != claim.group_name) {
    return {StatusCode::kInvalidArgument, "claim kind/group does not match endpoint policy"};
  }
  if (servo != is_servo(claim.kind)) {
    return {StatusCode::kInvalidArgument, servo ? "expected Servo claim" : "expected Move claim"};
  }
  return MotionStatus::Ok();
}

std::vector<std::string> ControlArbiter::computeWinners(SteadyTime now) const
{
  std::vector<const ClaimState *> candidates;
  for (const auto & [unused, state] : claims_) {
    (void)unused;
    if (is_servo(state.claim.kind) && now - state.received_at >= state.lease) {
      continue;
    }
    candidates.push_back(&state);
  }
  std::sort(candidates.begin(), candidates.end(), [](const ClaimState * left, const ClaimState * right) {
      return left->priority != right->priority ? left->priority > right->priority :
             left->receive_sequence > right->receive_sequence;
    });
  std::vector<std::string> result;
  std::vector<const ClaimState *> selected;
  for (const auto * candidate : candidates) {
    const bool conflict = std::any_of(selected.begin(), selected.end(), [candidate](const auto * winner) {
        return overlaps(candidate->claim.joint_names, winner->claim.joint_names);
      });
    if (!conflict) {
      selected.push_back(candidate);
      result.push_back(candidate->claim.session_id);
    }
  }
  return result;
}

std::vector<std::string> ControlArbiter::removePreemptedMoves(
  const std::vector<std::string> & old_winners,
  const std::vector<std::string> & new_winners)
{
  std::vector<std::string> result;
  for (const auto & id : old_winners) {
    const auto found = claims_.find(id);
    if (!contains_id(new_winners, id) && found != claims_.end() &&
      is_move(found->second.claim.kind))
    {
      result.push_back(id);
    }
  }
  for (const auto & id : result) {
    claims_.erase(id);
  }
  return result;
}

std::vector<std::string> ControlArbiter::removeExpiredServos(SteadyTime now)
{
  std::vector<std::string> expired;
  for (auto it = claims_.begin(); it != claims_.end();) {
    if (is_servo(it->second.claim.kind) && now - it->second.received_at >= it->second.lease) {
      expired.push_back(it->first);
      it = claims_.erase(it);
    } else {
      ++it;
    }
  }
  return expired;
}

ArbitrationResult ControlArbiter::snapshot(
  MotionStatus status, bool admitted, std::vector<std::string> preempted,
  std::vector<std::string> expired, SteadyTime now) const
{
  return {std::move(status), admitted, computeWinners(now), std::move(preempted),
    std::move(expired)};
}

ArbitrationResult ControlArbiter::submitMove(const CommandClaim & claim, SteadyTime now)
{
  const auto validation = validateClaim(claim, false);
  if (!validation.ok()) {
    return snapshot(validation, false, {}, {}, now);
  }
  if (claims_.count(claim.session_id) != 0U) {
    return snapshot({StatusCode::kInvalidArgument, "duplicate session id"}, false, {}, {}, now);
  }
  auto expired = removeExpiredServos(now);
  const auto old_winners = computeWinners(now);
  const auto & policy = policies_.at(claim.endpoint_name);
  claims_.emplace(claim.session_id,
    ClaimState{claim, policy.priority, next_sequence_++, now, policy.servo_lease});
  auto preempted = removePreemptedMoves(old_winners, computeWinners(now));
  if (!contains_id(computeWinners(now), claim.session_id)) {
    claims_.erase(claim.session_id);
    return snapshot({StatusCode::kRejected, "Move conflicts with a higher-priority command"},
      false, std::move(preempted), std::move(expired), now);
  }
  return snapshot(MotionStatus::Ok(), true, std::move(preempted), std::move(expired), now);
}

ArbitrationResult ControlArbiter::updateServo(const CommandClaim & claim, SteadyTime now)
{
  const auto validation = validateClaim(claim, true);
  if (!validation.ok()) {
    return snapshot(validation, false, {}, {}, now);
  }
  auto expired = removeExpiredServos(now);
  const auto old_winners = computeWinners(now);
  const auto & policy = policies_.at(claim.endpoint_name);
  claims_[claim.session_id] =
    ClaimState{claim, policy.priority, next_sequence_++, now, policy.servo_lease};
  auto preempted = removePreemptedMoves(old_winners, computeWinners(now));
  const bool admitted = contains_id(computeWinners(now), claim.session_id);
  return snapshot(MotionStatus::Ok(), admitted, std::move(preempted), std::move(expired), now);
}

ArbitrationResult ControlArbiter::evaluate(SteadyTime now)
{
  return snapshot(MotionStatus::Ok(), false, {}, removeExpiredServos(now), now);
}

void ControlArbiter::release(const std::string & session_id) {claims_.erase(session_id);}

bool ControlArbiter::contains(const std::string & session_id) const
{
  return claims_.count(session_id) != 0U;
}

std::vector<std::string> ControlArbiter::winners(SteadyTime now) const
{
  return computeWinners(now);
}

}  // namespace hc_motion
