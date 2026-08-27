#include <chrono>
#include <string>

#include <gtest/gtest.h>

#include "hc_motion/control_arbiter.hpp"

namespace
{

using namespace std::chrono_literals;

TEST(ControlArbiter, AllowsDisjointGroupsAndPreemptsConflictingMove)
{
  hc_motion::ControlArbiter arbiter;
  EXPECT_TRUE(arbiter.registerEndpoint(
      {"move_left", hc_motion::MotionKind::kMoveJ, "left_arm", 10, 100ms}).ok());
  EXPECT_TRUE(arbiter.registerEndpoint(
      {"move_right", hc_motion::MotionKind::kMoveJ, "right_arm", 10, 100ms}).ok());
  EXPECT_TRUE(arbiter.registerEndpoint(
      {"safety_servo", hc_motion::MotionKind::kServoJ, "left_arm", 100, 100ms}).ok());
  const auto now = hc_motion::SteadyTime{};
  auto left = arbiter.submitMove(
    {"move-l", "move_left", "left_arm", hc_motion::MotionKind::kMoveJ, {"l1", "l2"}}, now);
  auto right = arbiter.submitMove(
    {"move-r", "move_right", "right_arm", hc_motion::MotionKind::kMoveJ, {"r1", "r2"}}, now);
  EXPECT_TRUE(left.admitted);
  EXPECT_TRUE(right.admitted);
  auto safety = arbiter.updateServo(
    {"safe", "safety_servo", "left_arm", hc_motion::MotionKind::kServoJ, {"l1", "l2"}},
    now + 1ms);
  EXPECT_TRUE(safety.admitted);
  ASSERT_EQ(safety.preempted_move_ids.size(), 1U);
  EXPECT_EQ(safety.preempted_move_ids.front(), "move-l");
  EXPECT_TRUE(arbiter.contains("move-r"));
}

TEST(ControlArbiter, ExpiresServoWithoutResumingPreemptedMove)
{
  hc_motion::ControlArbiter arbiter;
  ASSERT_TRUE(arbiter.registerEndpoint(
      {"move", hc_motion::MotionKind::kMoveJ, "arm", 1, 50ms}).ok());
  ASSERT_TRUE(arbiter.registerEndpoint(
      {"servo", hc_motion::MotionKind::kServoJ, "arm", 2, 50ms}).ok());
  const auto now = hc_motion::SteadyTime{};
  ASSERT_TRUE(arbiter.submitMove(
      {"move-1", "move", "arm", hc_motion::MotionKind::kMoveJ, {"j1"}}, now).admitted);
  ASSERT_TRUE(arbiter.updateServo(
      {"servo-1", "servo", "arm", hc_motion::MotionKind::kServoJ, {"j1"}}, now).admitted);
  const auto result = arbiter.evaluate(now + 51ms);
  EXPECT_EQ(result.expired_servo_ids, std::vector<std::string>{"servo-1"});
  EXPECT_TRUE(result.winner_session_ids.empty());
  EXPECT_FALSE(arbiter.contains("move-1"));
}

TEST(ControlArbiter, RejectsUnconfiguredOrSpoofedEndpoint)
{
  hc_motion::ControlArbiter arbiter;
  ASSERT_TRUE(arbiter.registerEndpoint(
      {"servo", hc_motion::MotionKind::kServoJ, "left", 4, 100ms}).ok());
  const auto result = arbiter.updateServo(
    {"x", "servo", "right", hc_motion::MotionKind::kServoJ, {"j1"}},
    hc_motion::SteadyTime{});
  EXPECT_FALSE(result.admitted);
  EXPECT_EQ(result.status.code, hc_motion::StatusCode::kInvalidArgument);
}

}  // namespace
