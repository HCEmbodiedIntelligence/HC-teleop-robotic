#include "hc_device_sdk/mapping.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace hc_device_sdk
{
namespace
{

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

bool optionalLengthValid(const std::vector<double> & values, const std::size_t count)
{
  return values.empty() || values.size() == count;
}

DeviceResult invalid(std::string message)
{
  return DeviceResult::failure(DeviceError::kInvalidState, std::move(message));
}

}  // namespace

DeviceResult validateJointMappings(const std::vector<JointMapping> & mappings)
{
  if (mappings.empty()) {
    return DeviceResult::failure(
      DeviceError::kInvalidConfiguration, "at least one joint mapping is required");
  }
  std::set<std::string> logical_names;
  std::set<std::pair<std::string, std::string>> vendor_keys;
  for (const auto & mapping : mappings) {
    if (mapping.logical_name.empty() || mapping.vendor_name.empty()) {
      return DeviceResult::failure(
        DeviceError::kInvalidConfiguration,
        "joint mappings require non-empty logical and vendor names");
    }
    if (!std::isfinite(mapping.vendor_to_logical_scale) ||
      std::abs(mapping.vendor_to_logical_scale) <= std::numeric_limits<double>::epsilon() ||
      !std::isfinite(mapping.vendor_to_logical_offset_rad))
    {
      return DeviceResult::failure(
        DeviceError::kInvalidConfiguration,
        "joint mapping scale must be finite and non-zero and offset must be finite");
    }
    if (!logical_names.insert(mapping.logical_name).second) {
      return DeviceResult::failure(
        DeviceError::kInvalidConfiguration,
        "duplicate logical joint name '" + mapping.logical_name + "'");
    }
    if (!vendor_keys.emplace(mapping.vendor_group, mapping.vendor_name).second) {
      return DeviceResult::failure(
        DeviceError::kInvalidConfiguration,
        "duplicate vendor joint key '" + mapping.vendor_group + "/" +
        mapping.vendor_name + "'");
    }
  }
  return DeviceResult::success();
}

double vendorPositionToLogical(const JointMapping & mapping, const double value)
{
  return mapping.vendor_to_logical_scale * value + mapping.vendor_to_logical_offset_rad;
}

double logicalPositionToVendor(const JointMapping & mapping, const double value)
{
  return (value - mapping.vendor_to_logical_offset_rad) /
         mapping.vendor_to_logical_scale;
}

double vendorVelocityToLogical(const JointMapping & mapping, const double value)
{
  return mapping.vendor_to_logical_scale * value;
}

double logicalVelocityToVendor(const JointMapping & mapping, const double value)
{
  return value / mapping.vendor_to_logical_scale;
}

double vendorAccelerationToLogical(const JointMapping & mapping, const double value)
{
  return mapping.vendor_to_logical_scale * value;
}

double logicalAccelerationToVendor(const JointMapping & mapping, const double value)
{
  return value / mapping.vendor_to_logical_scale;
}

double vendorEffortToLogical(const JointMapping & mapping, const double value)
{
  return value / mapping.vendor_to_logical_scale;
}

double logicalEffortToVendor(const JointMapping & mapping, const double value)
{
  return mapping.vendor_to_logical_scale * value;
}

JointMappingTable::JointMappingTable(std::vector<JointMapping> mappings)
: mappings_(std::move(mappings))
{
  const auto validation = validateJointMappings(mappings_);
  if (!validation) {
    throw std::invalid_argument(validation.message);
  }
  for (std::size_t index = 0; index < mappings_.size(); ++index) {
    const auto & mapping = mappings_[index];
    logical_index_.emplace(mapping.logical_name, index);
    vendor_index_.emplace(
      std::make_pair(mapping.vendor_group, mapping.vendor_name), index);
    vendor_name_indices_[mapping.vendor_name].push_back(index);
  }
}

DeviceResult JointMappingTable::vendorToLogical(
  const VendorJointState & input, JointState & output) const
{
  const auto count = input.vendor_joint_names.size();
  if (count == 0U || input.positions.size() != count ||
    (!input.vendor_groups.empty() && input.vendor_groups.size() != count) ||
    !optionalLengthValid(input.velocities, count) ||
    !optionalLengthValid(input.efforts, count))
  {
    return invalid("vendor state field lengths are inconsistent");
  }
  if (!allFinite(input.positions) || !allFinite(input.velocities) ||
    !allFinite(input.efforts))
  {
    return invalid("vendor state contains NaN or Inf");
  }

  std::map<std::pair<std::string, std::string>, std::size_t> exact_inputs;
  std::map<std::string, std::vector<std::size_t>> inputs_by_name;
  for (std::size_t index = 0; index < count; ++index) {
    const auto & name = input.vendor_joint_names[index];
    const auto group = input.vendor_groups.empty() ? std::string{} : input.vendor_groups[index];
    if (name.empty() || !exact_inputs.emplace(std::make_pair(group, name), index).second) {
      return invalid("vendor state contains an empty or duplicate joint key");
    }
    inputs_by_name[name].push_back(index);
  }

  JointState converted;
  converted.sample_time = input.sample_time;
  converted.joint_names.reserve(mappings_.size());
  converted.positions_rad.reserve(mappings_.size());
  if (!input.velocities.empty()) {
    converted.velocities_rad_s.reserve(mappings_.size());
  }
  if (!input.efforts.empty()) {
    converted.efforts_nm.reserve(mappings_.size());
  }

  for (const auto & mapping : mappings_) {
    std::size_t input_index = 0U;
    bool found = false;
    if (!input.vendor_groups.empty()) {
      const auto exact = exact_inputs.find(
        std::make_pair(mapping.vendor_group, mapping.vendor_name));
      if (exact != exact_inputs.end()) {
        input_index = exact->second;
        found = true;
      }
    } else {
      const auto by_name = inputs_by_name.find(mapping.vendor_name);
      const auto configured_by_name = vendor_name_indices_.find(mapping.vendor_name);
      if (by_name != inputs_by_name.end() && by_name->second.size() == 1U &&
        configured_by_name != vendor_name_indices_.end() &&
        configured_by_name->second.size() == 1U)
      {
        input_index = by_name->second.front();
        found = true;
      }
    }
    if (!found) {
      return invalid(
        "vendor state is missing or ambiguously identifies joint '" +
        mapping.vendor_group + "/" + mapping.vendor_name + "'");
    }

    converted.joint_names.push_back(mapping.logical_name);
    converted.positions_rad.push_back(
      vendorPositionToLogical(mapping, input.positions[input_index]));
    if (!input.velocities.empty()) {
      converted.velocities_rad_s.push_back(
        vendorVelocityToLogical(mapping, input.velocities[input_index]));
    }
    if (!input.efforts.empty()) {
      converted.efforts_nm.push_back(
        vendorEffortToLogical(mapping, input.efforts[input_index]));
    }
  }

  output = std::move(converted);
  return DeviceResult::success();
}

DeviceResult JointMappingTable::logicalToVendor(
  const JointCommand & input, VendorJointCommand & output) const
{
  const auto count = input.joint_names.size();
  if (count == 0U || !optionalLengthValid(input.positions_rad, count) ||
    !optionalLengthValid(input.velocities_rad_s, count) ||
    !optionalLengthValid(input.accelerations_rad_s2, count) ||
    !optionalLengthValid(input.efforts_nm, count))
  {
    return invalid("logical command field lengths are inconsistent");
  }
  if ((input.mode == JointControlMode::kPosition && input.positions_rad.empty()) ||
    (input.mode == JointControlMode::kVelocity && input.velocities_rad_s.empty()) ||
    (input.mode == JointControlMode::kEffort && input.efforts_nm.empty()))
  {
    return invalid("logical command is missing the field required by its control mode");
  }
  if (!allFinite(input.positions_rad) || !allFinite(input.velocities_rad_s) ||
    !allFinite(input.accelerations_rad_s2) || !allFinite(input.efforts_nm))
  {
    return invalid("logical command contains NaN or Inf");
  }

  VendorJointCommand converted;
  converted.mode = input.mode;
  converted.logical_group_name = input.group_name;
  converted.vendor_joint_names.reserve(count);
  converted.vendor_groups.reserve(count);
  if (!input.positions_rad.empty()) {
    converted.positions.reserve(count);
  }
  if (!input.velocities_rad_s.empty()) {
    converted.velocities.reserve(count);
  }
  if (!input.accelerations_rad_s2.empty()) {
    converted.accelerations.reserve(count);
  }
  if (!input.efforts_nm.empty()) {
    converted.efforts.reserve(count);
  }

  std::set<std::string> names;
  for (std::size_t index = 0; index < count; ++index) {
    const auto & name = input.joint_names[index];
    if (name.empty() || !names.insert(name).second) {
      return invalid("logical command contains an empty or duplicate joint name");
    }
    const auto found = logical_index_.find(name);
    if (found == logical_index_.end()) {
      return invalid("logical command contains unknown joint '" + name + "'");
    }
    const auto & mapping = mappings_[found->second];
    converted.vendor_joint_names.push_back(mapping.vendor_name);
    converted.vendor_groups.push_back(mapping.vendor_group);
    if (!input.positions_rad.empty()) {
      converted.positions.push_back(logicalPositionToVendor(mapping, input.positions_rad[index]));
    }
    if (!input.velocities_rad_s.empty()) {
      converted.velocities.push_back(logicalVelocityToVendor(mapping, input.velocities_rad_s[index]));
    }
    if (!input.accelerations_rad_s2.empty()) {
      converted.accelerations.push_back(
        logicalAccelerationToVendor(mapping, input.accelerations_rad_s2[index]));
    }
    if (!input.efforts_nm.empty()) {
      converted.efforts.push_back(logicalEffortToVendor(mapping, input.efforts_nm[index]));
    }
  }

  output = std::move(converted);
  return DeviceResult::success();
}

}  // namespace hc_device_sdk
