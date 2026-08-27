#include "hc_motion/goal_monitor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace hc_motion
{
namespace
{

bool finite_pose(const Pose & pose)
{
  const bool finite = std::all_of(pose.position_m.begin(), pose.position_m.end(),
      [](double value) {return std::isfinite(value);}) &&
    std::all_of(pose.orientation_xyzw.begin(), pose.orientation_xyzw.end(),
      [](double value) {return std::isfinite(value);});
  double norm_squared = 0.0;
  for (double value : pose.orientation_xyzw) {
    norm_squared += value * value;
  }
  return finite && norm_squared > 1e-12;
}

double orientation_distance(const Pose & left, const Pose & right)
{
  double left_norm = 0.0;
  double right_norm = 0.0;
  double dot = 0.0;
  for (std::size_t index = 0; index < 4U; ++index) {
    left_norm += left.orientation_xyzw[index] * left.orientation_xyzw[index];
    right_norm += right.orientation_xyzw[index] * right.orientation_xyzw[index];
    dot += left.orientation_xyzw[index] * right.orientation_xyzw[index];
  }
  if (left_norm <= 0.0 || right_norm <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  dot = std::clamp(std::abs(dot / std::sqrt(left_norm * right_norm)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

}  // namespace

GoalMonitor::GoalMonitor(GoalMonitorConfig config)
: config_(std::move(config))
{
}

MotionStatus GoalMonitor::begin(
  const std::string & session_id, const JointTarget & joint_goal,
  const std::optional<Pose> & cartesian_goal, SteadyTime now)
{
  const std::array<double, 4> tolerances{{config_.joint_position_tolerance_rad,
      config_.joint_velocity_tolerance_rad_s, config_.cartesian_position_tolerance_m,
      config_.cartesian_orientation_tolerance_rad}};
  if (!std::all_of(tolerances.begin(), tolerances.end(), [](double value) {
      return std::isfinite(value) && value >= 0.0;
    }) || config_.stable_duration < std::chrono::milliseconds::zero() ||
    config_.move_timeout <= std::chrono::milliseconds::zero())
  {
    return {StatusCode::kInvalidArgument, "goal monitor configuration is invalid"};
  }
  const std::set<std::string> unique(joint_goal.joint_names.begin(), joint_goal.joint_names.end());
  if (session_id.empty() || joint_goal.joint_names.empty() ||
    joint_goal.joint_names.size() != joint_goal.positions_rad.size() ||
    unique.size() != joint_goal.joint_names.size() || unique.count("") != 0U ||
    !std::all_of(joint_goal.positions_rad.begin(), joint_goal.positions_rad.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return {StatusCode::kInvalidArgument, "goal identity/joints/positions are invalid"};
  }
  if (cartesian_goal && !finite_pose(*cartesian_goal)) {
    return {StatusCode::kInvalidArgument, "Cartesian goal is invalid"};
  }
  goals_[session_id] = GoalState{joint_goal, cartesian_goal, now, std::nullopt};
  return MotionStatus::Ok();
}

GoalObservationResult GoalMonitor::observe(
  const std::string & session_id, const JointFeedback & feedback,
  const std::optional<Pose> & fk_pose, SteadyTime now)
{
  const auto found = goals_.find(session_id);
  if (found == goals_.end()) {
    return {GoalObservation::kInvalidFeedback,
      {StatusCode::kInternalError, "goal monitor has no such session"}};
  }
  auto & goal = found->second;
  if (now - goal.started_at > config_.move_timeout) {
    return {GoalObservation::kTimedOut, {StatusCode::kTimeout, "Move timed out"}};
  }
  if (feedback.joint_names.size() != feedback.positions_rad.size() ||
    feedback.joint_names.size() != feedback.velocities_rad_s.size())
  {
    goal.stable_since.reset();
    return {GoalObservation::kInvalidFeedback,
      {StatusCode::kInvalidArgument, "feedback array length mismatch"}};
  }
  std::map<std::string, std::size_t> indices;
  for (std::size_t index = 0; index < feedback.joint_names.size(); ++index) {
    if (feedback.joint_names[index].empty() ||
      !indices.emplace(feedback.joint_names[index], index).second ||
      !std::isfinite(feedback.positions_rad[index]) ||
      !std::isfinite(feedback.velocities_rad_s[index]))
    {
      goal.stable_since.reset();
      return {GoalObservation::kInvalidFeedback,
        {StatusCode::kInvalidArgument, "feedback contains duplicate or invalid data"}};
    }
  }

  bool within = true;
  for (std::size_t index = 0; index < goal.joint_goal.joint_names.size(); ++index) {
    const auto actual = indices.find(goal.joint_goal.joint_names[index]);
    if (actual == indices.end()) {
      goal.stable_since.reset();
      return {GoalObservation::kInvalidFeedback,
        {StatusCode::kInvalidArgument, "feedback is missing a goal joint"}};
    }
    within = within &&
      std::abs(feedback.positions_rad[actual->second] - goal.joint_goal.positions_rad[index]) <=
      config_.joint_position_tolerance_rad &&
      std::abs(feedback.velocities_rad_s[actual->second]) <=
      config_.joint_velocity_tolerance_rad_s;
  }
  if (goal.cartesian_goal) {
    if (!fk_pose || !finite_pose(*fk_pose)) {
      goal.stable_since.reset();
      return {GoalObservation::kInvalidFeedback,
        {StatusCode::kBackendError, "Cartesian Move requires backend FK from real feedback"}};
    }
    double squared_distance = 0.0;
    for (std::size_t index = 0; index < 3U; ++index) {
      const double delta = fk_pose->position_m[index] - goal.cartesian_goal->position_m[index];
      squared_distance += delta * delta;
    }
    within = within && std::sqrt(squared_distance) <= config_.cartesian_position_tolerance_m &&
      orientation_distance(*fk_pose, *goal.cartesian_goal) <=
      config_.cartesian_orientation_tolerance_rad;
  }
  if (!within) {
    goal.stable_since.reset();
    return {GoalObservation::kRunning, MotionStatus::Ok()};
  }
  if (!goal.stable_since) {
    goal.stable_since = now;
  }
  return now - *goal.stable_since >= config_.stable_duration ?
    GoalObservationResult{GoalObservation::kReached, MotionStatus::Ok()} :
    GoalObservationResult{GoalObservation::kRunning, MotionStatus::Ok()};
}

void GoalMonitor::erase(const std::string & session_id) {goals_.erase(session_id);}

bool GoalMonitor::contains(const std::string & session_id) const
{
  return goals_.count(session_id) != 0U;
}

}  // namespace hc_motion
