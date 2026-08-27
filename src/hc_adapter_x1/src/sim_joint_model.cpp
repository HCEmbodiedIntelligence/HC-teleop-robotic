#include "hc_adapter_x1/sim_joint_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace hc_adapter_x1
{

SimJointModel::SimJointModel(std::vector<JointGroupConfig> groups)
{
  if (groups.empty()) {
    throw std::invalid_argument("at least one joint group is required");
  }

  for (auto & group : groups) {
    validateGroupConfig(group);
    if (groups_.find(group.group_name) != groups_.end()) {
      throw std::invalid_argument("duplicate joint group: " + group.group_name);
    }

    GroupState state;
    state.config = std::move(group);
    state.indices.reserve(state.config.joint_names.size());
    for (std::size_t index = 0; index < state.config.joint_names.size(); ++index) {
      const auto & joint_name = state.config.joint_names[index];
      if (joint_index_.find(joint_name) != joint_index_.end()) {
        throw std::invalid_argument("joint appears in more than one group: " + joint_name);
      }
      const auto global_index = joint_names_.size();
      joint_index_[joint_name] = global_index;
      joint_names_.push_back(joint_name);
      lower_limits_.push_back(state.config.lower_limits[index]);
      upper_limits_.push_back(state.config.upper_limits[index]);
      max_velocity_.push_back(state.config.max_velocity[index]);
      const auto initial = clamp(0.0, lower_limits_.back(), upper_limits_.back());
      positions_.push_back(initial);
      targets_.push_back(initial);
      state.indices.push_back(global_index);
    }
    groups_.emplace(state.config.group_name, std::move(state));
  }
}

bool SimJointModel::acceptCommand(
  const std::string & group_name,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions,
  const SteadyTime now,
  const SteadyTime valid_until,
  std::string & reason)
{
  const auto group = groups_.find(group_name);
  if (group == groups_.end()) {
    reason = "unknown joint group";
    return false;
  }
  if (valid_until <= now) {
    reason = "command is expired";
    return false;
  }
  const auto & expected = group->second.config.joint_names;
  if (joint_names != expected) {
    reason = "joint command names do not match the configured group order";
    return false;
  }
  if (positions.size() != expected.size()) {
    reason = "position command length does not match joint names";
    return false;
  }
  if (!std::all_of(positions.begin(), positions.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    reason = "position command contains a non-finite value";
    return false;
  }

  for (std::size_t index = 0; index < positions.size(); ++index) {
    const auto global_index = group->second.indices[index];
    targets_[global_index] = clamp(
      positions[index], lower_limits_[global_index], upper_limits_[global_index]);
  }
  group->second.valid_until = valid_until;
  reason.clear();
  return true;
}

void SimJointModel::step(const double dt_seconds, const SteadyTime now)
{
  if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
    return;
  }

  for (auto & entry : groups_) {
    auto & group = entry.second;
    if (group.valid_until && *group.valid_until <= now) {
      for (const auto global_index : group.indices) {
        targets_[global_index] = positions_[global_index];
      }
      group.valid_until.reset();
    }
  }

  for (std::size_t index = 0; index < positions_.size(); ++index) {
    const auto delta = targets_[index] - positions_[index];
    const auto max_step = max_velocity_[index] * dt_seconds;
    if (std::abs(delta) <= max_step) {
      positions_[index] = targets_[index];
    } else {
      positions_[index] += std::copysign(max_step, delta);
    }
  }
}

const std::vector<std::string> & SimJointModel::jointNames() const
{
  return joint_names_;
}

const std::vector<double> & SimJointModel::positions() const
{
  return positions_;
}

std::vector<double> SimJointModel::groupPositions(const std::string & group_name) const
{
  const auto group = groups_.find(group_name);
  if (group == groups_.end()) {
    return {};
  }
  std::vector<double> result;
  result.reserve(group->second.indices.size());
  for (const auto global_index : group->second.indices) {
    result.push_back(positions_[global_index]);
  }
  return result;
}

bool SimJointModel::hasGroup(const std::string & group_name) const
{
  return groups_.find(group_name) != groups_.end();
}

void SimJointModel::validateGroupConfig(const JointGroupConfig & group)
{
  if (group.group_name.empty()) {
    throw std::invalid_argument("joint group name must not be empty");
  }
  if (group.joint_names.empty()) {
    throw std::invalid_argument("joint group must contain at least one joint");
  }
  if (group.lower_limits.size() != group.joint_names.size() ||
    group.upper_limits.size() != group.joint_names.size())
  {
    throw std::invalid_argument("joint limits must match joint name count");
  }
  if (group.max_velocity.empty()) {
    throw std::invalid_argument("joint max velocity must be provided");
  }
  if (group.max_velocity.size() != group.joint_names.size()) {
    throw std::invalid_argument("joint max velocity must match joint name count");
  }
  std::unordered_set<std::string> names;
  for (std::size_t index = 0; index < group.joint_names.size(); ++index) {
    if (group.joint_names[index].empty() || !names.insert(group.joint_names[index]).second) {
      throw std::invalid_argument("joint names must be non-empty and unique");
    }
    const auto lower = group.lower_limits[index];
    const auto upper = group.upper_limits[index];
    const auto velocity = group.max_velocity[index];
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
      throw std::invalid_argument("joint limits must be finite and ordered");
    }
    if (!std::isfinite(velocity) || velocity <= 0.0) {
      throw std::invalid_argument("joint max velocity must be positive and finite");
    }
  }
}

double SimJointModel::clamp(const double value, const double lower, const double upper)
{
  return std::min(std::max(value, lower), upper);
}

}  // namespace hc_adapter_x1
