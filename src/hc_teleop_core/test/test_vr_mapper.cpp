#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "hc_teleop_core/vr_mapper.hpp"

namespace
{

using namespace std::chrono_literals;

hc_teleop_core::MapperConfig openarm_config()
{
  hc_teleop_core::MapperConfig config;
  config.bindings = {
    {"left_arm", hc_teleop_core::ControllerSide::kLeft, "left_base", "left_tcp"},
    {"right_arm", hc_teleop_core::ControllerSide::kRight, "right_base", "right_tcp"},
  };
  config.axis_mapping = {0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  config.position_scale = 0.8;
  config.clutch_threshold = 0.5;
  config.feedback_max_age = 150ms;
  return config;
}

hc_teleop_core::MapperPose pose(double x, double y, double z)
{
  return {{x, y, z}, {0.0, 0.0, 0.0, 1.0}};
}

TEST(VrMapper, AppliesOpenArmProfileAxisMappingFromMeasuredAnchor)
{
  hc_teleop_core::VrMapper mapper(openarm_config());
  const hc_teleop_core::MapperTime now{};
  std::string reason;
  ASSERT_TRUE(mapper.updateFeedback("left_arm", pose(0.5, 0.1, 0.2), now, reason));
  ASSERT_TRUE(mapper.updateFeedback("right_arm", pose(-0.5, 0.1, 0.2), now, reason));
  hc_teleop_core::MapperFrame frame;
  frame.session_id = "pico/1";
  frame.sequence = 1U;
  frame.left = {true, 0.8, pose(1.0, 2.0, 3.0)};
  frame.right = {false, 0.0, pose(0.0, 0.0, 0.0)};
  const auto anchor = mapper.map(frame, now);
  ASSERT_EQ(anchor.size(), 1U);
  EXPECT_NEAR(anchor[0].pose.position[0], 0.5, 1e-12);
  EXPECT_NEAR(anchor[0].pose.position[1], 0.1, 1e-12);
  EXPECT_NEAR(anchor[0].pose.position[2], 0.2, 1e-12);

  frame.sequence = 2U;
  frame.left.pose.position = {1.1, 2.1, 3.1};
  const auto mapped = mapper.map(frame, now + 10ms);
  ASSERT_EQ(mapped.size(), 1U);
  EXPECT_NEAR(mapped[0].pose.position[0], 0.42, 1e-12);  // VR +Z -> robot -X
  EXPECT_NEAR(mapped[0].pose.position[1], 0.02, 1e-12);  // VR +X -> robot -Y
  EXPECT_NEAR(mapped[0].pose.position[2], 0.28, 1e-12);  // VR +Y -> robot +Z
}

TEST(VrMapper, MapsRelativeOrientationThroughConfiguredBasis)
{
  hc_teleop_core::VrMapper mapper(openarm_config());
  const hc_teleop_core::MapperTime now{};
  std::string reason;
  ASSERT_TRUE(mapper.updateFeedback("left_arm", pose(0.0, 0.0, 0.0), now, reason));
  hc_teleop_core::MapperFrame frame;
  frame.session_id = "pico/1";
  frame.sequence = 1U;
  frame.left = {true, 0.8, pose(0.0, 0.0, 0.0)};
  ASSERT_EQ(mapper.map(frame, now).size(), 1U);
  frame.sequence = 2U;
  frame.left.pose.orientation = {std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)};
  const auto mapped = mapper.map(frame, now + 1ms);
  ASSERT_EQ(mapped.size(), 1U);
  EXPECT_NEAR(mapped[0].pose.orientation[0], 0.0, 1e-9);
  EXPECT_NEAR(mapped[0].pose.orientation[1], -std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(mapped[0].pose.orientation[2], 0.0, 1e-9);
  EXPECT_NEAR(mapped[0].pose.orientation[3], std::sqrt(0.5), 1e-9);
}

TEST(VrMapper, UsesConfiguredRightGripAsSharedArmClutch)
{
  auto config = openarm_config();
  config.clutch_controller = hc_teleop_core::ControllerSide::kRight;
  hc_teleop_core::VrMapper mapper(config);
  const hc_teleop_core::MapperTime now{};
  std::string reason;
  ASSERT_TRUE(mapper.updateFeedback("left_arm", pose(0.1, 0.0, 0.0), now, reason));
  ASSERT_TRUE(mapper.updateFeedback("right_arm", pose(-0.1, 0.0, 0.0), now, reason));
  hc_teleop_core::MapperFrame frame;
  frame.session_id = "pico/shared-clutch";
  frame.sequence = 1U;
  frame.left = {true, 0.0, pose(0.0, 0.0, 0.0)};
  frame.right = {true, 0.8, pose(0.0, 0.0, 0.0)};
  EXPECT_EQ(mapper.map(frame, now).size(), 2U);

  frame.sequence = 2U;
  frame.right.grip = 0.0;
  EXPECT_TRUE(mapper.map(frame, now + 1ms).empty());
}

TEST(VrMapper, AppliesPerBindingBasesForMirroredArmReferenceFrames)
{
  auto config = openarm_config();
  config.clutch_controller = hc_teleop_core::ControllerSide::kRight;
  config.bindings[0].axis_mapping = std::array<double, 9>{
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0};
  config.bindings[1].axis_mapping = std::array<double, 9>{
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0};
  hc_teleop_core::VrMapper mapper(config);
  const hc_teleop_core::MapperTime now{};
  std::string reason;
  ASSERT_TRUE(mapper.updateFeedback("left_arm", pose(0.0, 0.0, 0.0), now, reason));
  ASSERT_TRUE(mapper.updateFeedback("right_arm", pose(0.0, 0.0, 0.0), now, reason));

  hc_teleop_core::MapperFrame frame;
  frame.session_id = "pico/mirrored";
  frame.sequence = 1U;
  frame.left = {true, 0.0, pose(0.0, 0.0, 0.0)};
  frame.right = {true, 0.8, pose(0.0, 0.0, 0.0)};
  ASSERT_EQ(mapper.map(frame, now).size(), 2U);

  frame.sequence = 2U;
  frame.left.pose.position = {0.1, 0.2, 0.3};
  frame.right.pose.position = {0.1, 0.2, 0.3};
  const auto mapped = mapper.map(frame, now + 1ms);
  ASSERT_EQ(mapped.size(), 2U);
  // Local positions differ because the two bases are +/-90 degrees about X.
  EXPECT_NEAR(mapped[0].pose.position[0], 0.24, 1e-12);
  EXPECT_NEAR(mapped[0].pose.position[1], -0.08, 1e-12);
  EXPECT_NEAR(mapped[0].pose.position[2], 0.16, 1e-12);
  EXPECT_NEAR(mapped[1].pose.position[0], 0.24, 1e-12);
  EXPECT_NEAR(mapped[1].pose.position[1], 0.08, 1e-12);
  EXPECT_NEAR(mapped[1].pose.position[2], -0.16, 1e-12);
  // Applying each URDF base rotation gives the same torso-frame displacement:
  // +X=0.24, +Y=0.16, +Z=0.08 for both arms.
}

TEST(VrMapper, StopsOnStaleFeedbackTrackingLossAndDuplicateSequence)
{
  hc_teleop_core::VrMapper mapper(openarm_config());
  const hc_teleop_core::MapperTime now{};
  std::string reason;
  ASSERT_TRUE(mapper.updateFeedback("left_arm", pose(0.0, 0.0, 0.0), now, reason));
  hc_teleop_core::MapperFrame frame;
  frame.session_id = "pico/1";
  frame.sequence = 1U;
  frame.left = {true, 0.8, pose(0.0, 0.0, 0.0)};
  EXPECT_EQ(mapper.map(frame, now).size(), 1U);
  EXPECT_TRUE(mapper.map(frame, now + 1ms).empty());
  frame.sequence = 2U;
  EXPECT_TRUE(mapper.map(frame, now + 151ms).empty());
  ASSERT_TRUE(mapper.updateFeedback("left_arm", pose(0.2, 0.0, 0.0), now + 152ms, reason));
  frame.sequence = 3U;
  frame.left.tracked = false;
  EXPECT_TRUE(mapper.map(frame, now + 152ms).empty());
}

TEST(VrMapper, RejectsInvalidMappingAndFeedback)
{
  auto config = openarm_config();
  config.axis_mapping[0] = 2.0;
  EXPECT_THROW(
    {
      const hc_teleop_core::VrMapper invalid_mapper(config);
      (void)invalid_mapper;
    },
    std::invalid_argument);

  config = openarm_config();
  config.bindings[0].axis_mapping = std::array<double, 9>{
    2.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  EXPECT_THROW(
    {
      const hc_teleop_core::VrMapper invalid_mapper(config);
      (void)invalid_mapper;
    },
    std::invalid_argument);

  hc_teleop_core::VrMapper mapper(openarm_config());
  auto invalid = pose(0.0, 0.0, 0.0);
  invalid.position[0] = std::numeric_limits<double>::quiet_NaN();
  std::string reason;
  EXPECT_FALSE(mapper.updateFeedback(
      "left_arm", invalid, hc_teleop_core::MapperTime{}, reason));
}

}  // namespace
