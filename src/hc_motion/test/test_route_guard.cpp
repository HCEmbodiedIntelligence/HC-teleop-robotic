#include <chrono>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "hc_motion/route_guard.hpp"

namespace
{

using namespace std::chrono_literals;

hc_motion::CartesianTargetDescriptor target(std::string group)
{
  hc_motion::CartesianTargetDescriptor value;
  value.group_name = std::move(group);
  value.reference_frame = "base";
  value.tip_frame = "tcp";
  value.orientation = {0.0, 0.0, 0.0, 1.0};
  return value;
}

hc_motion::CandidateEnvelope candidate(const hc_motion::SteadyTime now)
{
  hc_motion::CandidateEnvelope value;
  value.source_id = "vr";
  value.session_id = "session";
  value.group_name = "left_arm";
  value.sequence = 4U;
  value.valid_until = now + 80ms;
  value.control_mode = 1U;
  value.joint_names = {"j1", "j2"};
  value.positions = {0.1, 0.2};
  return value;
}

TEST(RouteGuard, CorrelatesBackendCandidateWithLatestLiveTarget)
{
  hc_motion::RouteGuard guard({"left_arm", "right_arm"});
  const auto now = hc_motion::SteadyTime{};
  hc_motion::TargetEnvelope input{
    "vr", "session", 4U, now + 100ms, {target("left_arm"), target("right_arm")}};
  std::string reason;
  ASSERT_TRUE(guard.route(input, now, reason)) << reason;
  EXPECT_TRUE(guard.authorize(candidate(now), now, reason)) << reason;

  auto wrong = candidate(now);
  wrong.sequence = 3U;
  EXPECT_FALSE(guard.authorize(wrong, now, reason));
  wrong = candidate(now);
  wrong.group_name = "waist";
  EXPECT_FALSE(guard.authorize(wrong, now, reason));
}

TEST(RouteGuard, RejectsStaleInvalidOrEscalatedInput)
{
  hc_motion::RouteGuard guard({"left_arm"});
  const auto now = hc_motion::SteadyTime{};
  std::string reason;
  hc_motion::TargetEnvelope input{"vr", "session", 1U, now + 100ms, {target("left_arm")}};
  ASSERT_TRUE(guard.route(input, now, reason));
  EXPECT_FALSE(guard.route(input, now, reason));

  auto output = candidate(now);
  output.sequence = 1U;
  output.valid_until = now + 130ms;
  EXPECT_FALSE(guard.authorize(output, now, reason));
  output.valid_until = now + 80ms;
  output.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(guard.authorize(output, now, reason));
}

TEST(RouteGuard, ExpiresAuthorization)
{
  hc_motion::RouteGuard guard({"left_arm"});
  const auto now = hc_motion::SteadyTime{};
  std::string reason;
  ASSERT_TRUE(guard.route(
      {"vr", "session", 4U, now + 100ms, {target("left_arm")}}, now, reason));
  EXPECT_FALSE(guard.authorize(candidate(now), now + 101ms, reason));
}

TEST(RouteGuard, CorrelatesPendingCandidatesIndependentlyPerArm)
{
  hc_motion::RouteGuard guard({"left_arm", "right_arm"});
  const auto now = hc_motion::SteadyTime{};
  std::string reason;
  ASSERT_TRUE(guard.route(
      {"vr", "session", 4U, now + 100ms, {target("left_arm"), target("right_arm")}},
      now, reason));

  auto left4 = candidate(now);
  ASSERT_TRUE(guard.authorize(left4, now, reason)) << reason;

  ASSERT_TRUE(guard.route(
      {"vr", "session", 5U, now + 120ms, {target("left_arm"), target("right_arm")}},
      now + 1ms, reason));
  auto left5 = candidate(now);
  left5.sequence = 5U;
  ASSERT_TRUE(guard.authorize(left5, now + 2ms, reason)) << reason;

  // The right-arm result for sequence 4 remains useful and monotonic even
  // though sequence 5 has already been routed and the left arm advanced.
  auto right4 = candidate(now);
  right4.group_name = "right_arm";
  EXPECT_TRUE(guard.authorize(right4, now + 3ms, reason)) << reason;

  auto right5 = right4;
  right5.sequence = 5U;
  EXPECT_TRUE(guard.authorize(right5, now + 4ms, reason)) << reason;

  // A group may never move backwards after a newer candidate was accepted.
  EXPECT_FALSE(guard.authorize(left4, now + 5ms, reason));
  EXPECT_EQ(reason, "backend candidate sequence is stale or duplicated for its group");
}

}  // namespace
