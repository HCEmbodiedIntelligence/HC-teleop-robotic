#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "hc_compat_bridge/vr_legacy_json.hpp"
#include "hc_teleop_interfaces/msg/tracked_pose.hpp"
#include "hc_teleop_interfaces/msg/vr_frame.hpp"

namespace
{

hc_teleop_interfaces::msg::VrFrame frame()
{
  hc_teleop_interfaces::msg::VrFrame value;
  value.protocol_version = 2U;
  value.sequence = 42U;
  value.source_stamp.sec = 12;
  value.source_stamp.nanosec = 500000000U;
  value.head.tracking_state = hc_teleop_interfaces::msg::TrackedPose::TRACKING_TRACKED;
  value.left_controller.tracking_state =
    hc_teleop_interfaces::msg::TrackedPose::TRACKING_TRACKED;
  value.right_controller.tracking_state =
    hc_teleop_interfaces::msg::TrackedPose::TRACKING_UNAVAILABLE;
  value.head.pose.orientation.w = 1.0;
  value.left_controller.pose.position.x = 1.0;
  value.left_controller.pose.position.y = 2.0;
  value.left_controller.pose.position.z = 3.0;
  value.left_controller.pose.orientation.w = 1.0;
  value.right_controller.pose.orientation.w = 1.0;
  value.left_input.held_mask = 9U;
  value.left_input.pressed_mask = 1U;
  value.left_input.trigger = 0.75F;
  value.right_input.grip = 0.8F;
  return value;
}

TEST(VrLegacyJson, PreservesHistoricalSchema)
{
  const auto json = hc_compat_bridge::to_legacy_vr_json(frame());
  ASSERT_TRUE(json.has_value());
  EXPECT_NE(json->find("\"protocol_version\":2"), std::string::npos);
  EXPECT_NE(json->find("\"sequence\":42"), std::string::npos);
  EXPECT_NE(json->find("\"vr_timestamp\":12.5"), std::string::npos);
  EXPECT_NE(json->find("\"tracking\":{\"head\":true,\"left\":true,\"right\":false}"),
    std::string::npos);
  EXPECT_NE(json->find("\"held\":[\"primary\",\"trigger_button\"]"),
    std::string::npos);
  EXPECT_NE(json->find("\"position\":[1,2,3]"), std::string::npos);
}

TEST(VrLegacyJson, RejectsNonFiniteValues)
{
  auto value = frame();
  value.head.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(hc_compat_bridge::to_legacy_vr_json(value).has_value());
}

}  // namespace
