#include "hc_compat_bridge/vr_legacy_json.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string_view>

#include "hc_teleop_interfaces/msg/controller_input.hpp"
#include "hc_teleop_interfaces/msg/tracked_pose.hpp"

namespace hc_compat_bridge
{
namespace
{

using ControllerInput = hc_teleop_interfaces::msg::ControllerInput;
using TrackedPose = hc_teleop_interfaces::msg::TrackedPose;

constexpr std::array<std::pair<std::uint32_t, std::string_view>, 11> kButtons{{
  {ControllerInput::BUTTON_PRIMARY, "primary"},
  {ControllerInput::BUTTON_SECONDARY, "secondary"},
  {ControllerInput::BUTTON_GRIP, "grip_button"},
  {ControllerInput::BUTTON_TRIGGER, "trigger_button"},
  {ControllerInput::BUTTON_MENU, "menu"},
  {ControllerInput::BUTTON_PRIMARY_AXIS_CLICK, "primary_axis_click"},
  {ControllerInput::BUTTON_PRIMARY_AXIS_TOUCH, "primary_axis_touch"},
  {ControllerInput::BUTTON_SECONDARY_AXIS_CLICK, "secondary_axis_click"},
  {ControllerInput::BUTTON_SECONDARY_AXIS_TOUCH, "secondary_axis_touch"},
  {ControllerInput::BUTTON_PRIMARY_TOUCH, "primary_touch"},
  {ControllerInput::BUTTON_SECONDARY_TOUCH, "secondary_touch"},
}};

bool finite(const TrackedPose & tracked) noexcept
{
  const auto & p = tracked.pose.position;
  const auto & q = tracked.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}

bool finite(const ControllerInput & input) noexcept
{
  return std::isfinite(input.trigger) && std::isfinite(input.grip) &&
         std::isfinite(input.primary_axis[0]) &&
         std::isfinite(input.primary_axis[1]) &&
         std::isfinite(input.secondary_axis[0]) &&
         std::isfinite(input.secondary_axis[1]);
}

void write_pose(std::ostream & stream, const TrackedPose & tracked)
{
  const auto & p = tracked.pose.position;
  const auto & q = tracked.pose.orientation;
  stream << "{\"position\":[" << p.x << ',' << p.y << ',' << p.z
         << "],\"quaternion\":[" << q.x << ',' << q.y << ',' << q.z << ',' << q.w
         << "]}";
}

void write_button_names(std::ostream & stream, std::uint32_t mask)
{
  stream << '[';
  bool first = true;
  for (const auto & [bit, name] : kButtons) {
    if ((mask & bit) == 0U) {
      continue;
    }
    if (!first) {
      stream << ',';
    }
    first = false;
    stream << '\"' << name << '\"';
  }
  stream << ']';
}

void write_input(std::ostream & stream, const ControllerInput & input)
{
  stream << "{\"held_mask\":" << input.held_mask
         << ",\"pressed_mask\":" << input.pressed_mask
         << ",\"released_mask\":" << input.released_mask
         << ",\"held\":";
  write_button_names(stream, input.held_mask);
  stream << ",\"pressed\":";
  write_button_names(stream, input.pressed_mask);
  stream << ",\"released\":";
  write_button_names(stream, input.released_mask);
  stream << ",\"trigger\":" << input.trigger
         << ",\"grip\":" << input.grip
         << ",\"primary_axis\":[" << input.primary_axis[0] << ','
         << input.primary_axis[1] << "]"
         << ",\"secondary_axis\":[" << input.secondary_axis[0] << ','
         << input.secondary_axis[1] << "]}";
}

}  // namespace

std::optional<std::string> to_legacy_vr_json(
  const hc_teleop_interfaces::msg::VrFrame & frame)
{
  if (!finite(frame.head) || !finite(frame.left_controller) ||
    !finite(frame.right_controller) || !finite(frame.left_input) ||
    !finite(frame.right_input))
  {
    return std::nullopt;
  }

  const double source_time = static_cast<double>(frame.source_stamp.sec) +
    static_cast<double>(frame.source_stamp.nanosec) * 1.0e-9;
  std::ostringstream stream;
  stream << std::boolalpha << std::setprecision(std::numeric_limits<double>::max_digits10);
  stream << "{\"protocol_version\":" << frame.protocol_version
         << ",\"sequence\":" << frame.sequence
         << ",\"vr_timestamp\":" << source_time
         << ",\"tracking\":{"
         << "\"head\":" << (frame.head.tracking_state != TrackedPose::TRACKING_UNAVAILABLE)
         << ",\"left\":" <<
    (frame.left_controller.tracking_state != TrackedPose::TRACKING_UNAVAILABLE)
         << ",\"right\":" <<
    (frame.right_controller.tracking_state != TrackedPose::TRACKING_UNAVAILABLE)
         << "},\"poses\":{\"head\":";
  write_pose(stream, frame.head);
  stream << ",\"left\":";
  write_pose(stream, frame.left_controller);
  stream << ",\"right\":";
  write_pose(stream, frame.right_controller);
  stream << "},\"inputs\":{\"left\":";
  write_input(stream, frame.left_input);
  stream << ",\"right\":";
  write_input(stream, frame.right_input);
  stream << "}}";
  return stream.str();
}

}  // namespace hc_compat_bridge
