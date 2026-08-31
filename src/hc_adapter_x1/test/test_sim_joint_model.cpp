#include "hc_adapter_x1/sim_joint_model.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <gtest/gtest.h>

namespace
{

using namespace std::chrono_literals;
using hc_adapter_x1::JointGroupConfig;
using hc_adapter_x1::SimJointModel;
using hc_adapter_x1::SteadyTime;

JointGroupConfig group(const std::string & name)
{
  JointGroupConfig config;
  config.group_name = name;
  config.joint_names = {name + "_joint_1", name + "_joint_2"};
  config.lower_limits = {-1.0, -2.0};
  config.upper_limits = {1.0, 2.0};
  config.max_velocity = {2.0, 4.0};
  config.reference_frame = name + "_base";
  config.tip_frame = name + "_tip";
  return config;
}

TEST(SimJointModel, AppliesRateLimitAndClampsToConfiguredLimits)
{
  const SteadyTime now{};
  SimJointModel model({group("left_arm")});
  std::string reason;
  ASSERT_TRUE(model.acceptCommand(
    "left_arm", {"left_arm_joint_1", "left_arm_joint_2"}, {5.0, -5.0}, now, now + 2s,
    reason)) << reason;

  model.step(0.25, now + 250ms);
  const auto first_step = model.groupPositions("left_arm");
  ASSERT_EQ(first_step.size(), 2U);
  EXPECT_DOUBLE_EQ(first_step[0], 0.5);
  EXPECT_DOUBLE_EQ(first_step[1], -1.0);

  model.step(1.0, now + 1250ms);
  const auto clamped = model.groupPositions("left_arm");
  EXPECT_DOUBLE_EQ(clamped[0], 1.0);
  EXPECT_DOUBLE_EQ(clamped[1], -2.0);
}

TEST(SimJointModel, RejectsWrongJointOrder)
{
  const SteadyTime now{};
  SimJointModel model({group("left_arm")});
  std::string reason;
  EXPECT_FALSE(model.acceptCommand(
    "left_arm", {"left_arm_joint_2", "left_arm_joint_1"}, {0.1, 0.2}, now, now + 1s,
    reason));
  EXPECT_EQ(reason, "joint command names do not match the configured group order");
}

TEST(SimJointModel, UsesNamedInitialPositionsAndClampsThem)
{
  const std::unordered_map<std::string, double> initial{
    {"left_arm_joint_1", 0.75},
    {"left_arm_joint_2", -5.0},
  };
  SimJointModel model({group("left_arm")}, initial);
  const auto positions = model.groupPositions("left_arm");
  ASSERT_EQ(positions.size(), 2U);
  EXPECT_DOUBLE_EQ(positions[0], 0.75);
  EXPECT_DOUBLE_EQ(positions[1], -2.0);
}

TEST(SimJointModel, ExpiredCommandStopsFurtherMotion)
{
  const SteadyTime now{};
  SimJointModel model({group("left_arm")});
  std::string reason;
  ASSERT_TRUE(model.acceptCommand(
    "left_arm", {"left_arm_joint_1", "left_arm_joint_2"}, {1.0, 2.0}, now, now + 10ms,
    reason)) << reason;

  model.step(0.002, now + 2ms);
  const auto before_expiry = model.groupPositions("left_arm");
  model.step(1.0, now + 20ms);
  const auto after_expiry = model.groupPositions("left_arm");
  EXPECT_DOUBLE_EQ(after_expiry[0], before_expiry[0]);
  EXPECT_DOUBLE_EQ(after_expiry[1], before_expiry[1]);
}

TEST(SimJointModel, ValidatesConfiguration)
{
  auto invalid = group("left_arm");
  invalid.max_velocity = {1.0};
  EXPECT_THROW(SimJointModel({invalid}), std::invalid_argument);
}

}  // namespace
