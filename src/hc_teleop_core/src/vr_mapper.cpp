#include "hc_teleop_core/vr_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace hc_teleop_core
{
namespace
{

using Matrix = std::array<double, 9>;

Matrix transpose(const Matrix & value)
{
  return {value[0], value[3], value[6], value[1], value[4], value[7],
    value[2], value[5], value[8]};
}

Matrix multiply(const Matrix & left, const Matrix & right)
{
  Matrix result{};
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 3U; ++column) {
      for (std::size_t inner = 0; inner < 3U; ++inner) {
        result[row * 3U + column] +=
          left[row * 3U + inner] * right[inner * 3U + column];
      }
    }
  }
  return result;
}

std::array<double, 3> multiply(const Matrix & matrix, const std::array<double, 3> & vector)
{
  return {
    matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
    matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
    matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]};
}

Matrix quaternion_matrix(const std::array<double, 4> & value)
{
  double norm = 0.0;
  for (double item : value) {
    norm += item * item;
  }
  norm = std::sqrt(norm);
  const double x = value[0] / norm;
  const double y = value[1] / norm;
  const double z = value[2] / norm;
  const double w = value[3] / norm;
  return {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
    2.0 * (x * z + y * w), 2.0 * (x * y + z * w),
    1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
    1.0 - 2.0 * (x * x + y * y)};
}

std::array<double, 4> matrix_quaternion(const Matrix & matrix)
{
  std::array<double, 4> result{};
  const double trace = matrix[0] + matrix[4] + matrix[8];
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    result = {(matrix[7] - matrix[5]) / scale, (matrix[2] - matrix[6]) / scale,
      (matrix[3] - matrix[1]) / scale, 0.25 * scale};
  } else if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
    const double scale = 2.0 * std::sqrt(1.0 + matrix[0] - matrix[4] - matrix[8]);
    result = {0.25 * scale, (matrix[1] + matrix[3]) / scale,
      (matrix[2] + matrix[6]) / scale, (matrix[7] - matrix[5]) / scale};
  } else if (matrix[4] > matrix[8]) {
    const double scale = 2.0 * std::sqrt(1.0 + matrix[4] - matrix[0] - matrix[8]);
    result = {(matrix[1] + matrix[3]) / scale, 0.25 * scale,
      (matrix[5] + matrix[7]) / scale, (matrix[2] - matrix[6]) / scale};
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + matrix[8] - matrix[0] - matrix[4]);
    result = {(matrix[2] + matrix[6]) / scale, (matrix[5] + matrix[7]) / scale,
      0.25 * scale, (matrix[3] - matrix[1]) / scale};
  }
  double norm = 0.0;
  for (double item : result) {
    norm += item * item;
  }
  norm = std::sqrt(norm);
  for (double & item : result) {
    item /= norm;
  }
  if (result[3] < 0.0) {
    for (double & item : result) {
      item = -item;
    }
  }
  return result;
}

double determinant(const Matrix & value)
{
  return value[0] * (value[4] * value[8] - value[5] * value[7]) -
         value[1] * (value[3] * value[8] - value[5] * value[6]) +
         value[2] * (value[3] * value[7] - value[4] * value[6]);
}

bool sequence_newer(std::uint32_t sequence, std::uint32_t previous)
{
  const std::uint32_t difference = sequence - previous;
  return difference != 0U && difference < 0x80000000U;
}

}  // namespace

VrMapper::VrMapper(MapperConfig config)
: config_(std::move(config))
{
  if (config_.bindings.empty() || !std::isfinite(config_.position_scale) ||
    config_.position_scale <= 0.0 || !std::isfinite(config_.clutch_threshold) ||
    config_.clutch_threshold < 0.0 || config_.clutch_threshold > 1.0 ||
    config_.feedback_max_age <= std::chrono::milliseconds::zero())
  {
    throw std::invalid_argument("VR mapper configuration is invalid");
  }
  std::set<std::string> groups;
  std::set<ControllerSide> sides;
  for (const auto & binding : config_.bindings) {
    if (binding.group_name.empty() || binding.reference_frame.empty() || binding.tip_frame.empty() ||
      !groups.insert(binding.group_name).second || !sides.insert(binding.controller).second)
    {
      throw std::invalid_argument("VR bindings and frames must be non-empty and unique");
    }
    clutches_.emplace(binding.group_name, ClutchState{});
  }
  const Matrix gram = multiply(config_.axis_mapping, transpose(config_.axis_mapping));
  for (std::size_t index = 0; index < gram.size(); ++index) {
    const double expected = index == 0U || index == 4U || index == 8U ? 1.0 : 0.0;
    if (!std::isfinite(gram[index]) || std::abs(gram[index] - expected) > 1e-6) {
      throw std::invalid_argument("axis_mapping must be orthonormal");
    }
  }
  if (std::abs(std::abs(determinant(config_.axis_mapping)) - 1.0) > 1e-6) {
    throw std::invalid_argument("axis_mapping determinant must have magnitude one");
  }
}

bool VrMapper::updateFeedback(
  const std::string & group_name, const MapperPose & pose,
  MapperTime received_at, std::string & reason)
{
  if (clutches_.count(group_name) == 0U) {
    reason = "feedback group is not configured";
    return false;
  }
  if (!validPose(pose)) {
    reason = "feedback pose is non-finite or has a zero quaternion";
    return false;
  }
  feedback_[group_name] = Feedback{pose, received_at};
  reason.clear();
  return true;
}

std::vector<MappedTarget> VrMapper::map(const MapperFrame & frame, MapperTime now)
{
  std::vector<MappedTarget> result;
  if (frame.session_id.empty()) {
    reset();
    return result;
  }
  if (frame.session_id != session_id_) {
    for (auto & [unused, clutch] : clutches_) {
      (void)unused;
      clutch = ClutchState{};
    }
    session_id_ = frame.session_id;
    last_sequence_.reset();
  }
  if (last_sequence_ && !sequence_newer(frame.sequence, *last_sequence_)) {
    return result;
  }
  last_sequence_ = frame.sequence;

  for (const auto & binding : config_.bindings) {
    auto & clutch = clutches_.at(binding.group_name);
    const auto & controller = binding.controller == ControllerSide::kLeft ? frame.left : frame.right;
    const auto feedback = feedback_.find(binding.group_name);
    const bool feedback_fresh = feedback != feedback_.end() && feedback->second.received_at <= now &&
      now - feedback->second.received_at <= config_.feedback_max_age;
    const bool active = controller.tracked && validPose(controller.pose) &&
      std::isfinite(controller.grip) && controller.grip >= config_.clutch_threshold &&
      feedback_fresh;
    if (!active) {
      clutch = ClutchState{};
      continue;
    }
    if (!clutch.engaged) {
      clutch.engaged = true;
      clutch.controller_anchor = controller.pose;
      clutch.robot_anchor = feedback->second.pose;
    }
    result.push_back({binding.group_name, binding.reference_frame, binding.tip_frame,
      apply(controller.pose, clutch)});
  }
  return result;
}

void VrMapper::reset()
{
  for (auto & [unused, clutch] : clutches_) {
    (void)unused;
    clutch = ClutchState{};
  }
  session_id_.clear();
  last_sequence_.reset();
}

bool VrMapper::validPose(const MapperPose & pose) const
{
  const bool finite = std::all_of(pose.position.begin(), pose.position.end(),
      [](double value) {return std::isfinite(value);}) &&
    std::all_of(pose.orientation.begin(), pose.orientation.end(),
      [](double value) {return std::isfinite(value);});
  double norm = 0.0;
  for (double value : pose.orientation) {
    norm += value * value;
  }
  return finite && norm > 1e-12;
}

MapperPose VrMapper::apply(const MapperPose & controller, const ClutchState & clutch) const
{
  MapperPose result = clutch.robot_anchor;
  std::array<double, 3> delta{};
  for (std::size_t index = 0; index < 3U; ++index) {
    delta[index] = controller.position[index] - clutch.controller_anchor.position[index];
  }
  const auto mapped_delta = multiply(config_.axis_mapping, delta);
  for (std::size_t index = 0; index < 3U; ++index) {
    result.position[index] += config_.position_scale * mapped_delta[index];
  }

  const Matrix current = quaternion_matrix(controller.orientation);
  const Matrix anchor = quaternion_matrix(clutch.controller_anchor.orientation);
  const Matrix robot_anchor = quaternion_matrix(clutch.robot_anchor.orientation);
  const Matrix vr_delta = multiply(current, transpose(anchor));
  const Matrix mapped_rotation = multiply(
    multiply(config_.axis_mapping, vr_delta), transpose(config_.axis_mapping));
  result.orientation = matrix_quaternion(multiply(mapped_rotation, robot_anchor));
  return result;
}

}  // namespace hc_teleop_core
