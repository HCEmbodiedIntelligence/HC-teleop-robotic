#include "hc_teleop_core/command_arbiter.hpp"

#include <chrono>
#include <limits>
#include <string>

#include <gtest/gtest.h>

namespace
{

using namespace std::chrono_literals;
using hc_teleop_core::CommandArbiter;
using hc_teleop_core::JointCandidate;
using hc_teleop_core::SafetyCode;
using hc_teleop_core::SteadyTime;

JointCandidate candidate(
  const SteadyTime now,
  const std::uint64_t sequence = 1,
  const std::string & group_name = "left_arm")
{
  JointCandidate result;
  result.source_id = "vr";
  result.session_id = "session-a";
  result.group_name = group_name;
  result.sequence = sequence;
  result.control_mode = 1;
  result.valid_until = now + 100ms;
  result.names = {"joint_1", "joint_2"};
  result.positions = {0.1, -0.2};
  return result;
}

TEST(CommandArbiter, RequiresExplicitEnableAndLease)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  std::string reason;
  EXPECT_FALSE(arbiter.submit(candidate(now), now, reason));
  EXPECT_EQ(reason, "control is disabled");

  arbiter.setEnabled(true, now);
  EXPECT_FALSE(arbiter.submit(candidate(now), now, reason));
  EXPECT_EQ(reason, "no active control lease");

  const auto lease = arbiter.acquire("vr", "session-a", 10, 1s, now);
  ASSERT_TRUE(lease.granted);
  EXPECT_TRUE(arbiter.submit(candidate(now), now, reason));
  ASSERT_EQ(arbiter.commands(now + 10ms).size(), 1U);
  EXPECT_EQ(arbiter.status(now + 10ms).code, SafetyCode::kActive);
}

TEST(CommandArbiter, RejectsWrongOwnerDuplicateAndInvalidValues)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  arbiter.setEnabled(true, now);
  ASSERT_TRUE(arbiter.acquire("vr", "session-a", 10, 1s, now).granted);
  std::string reason;

  auto wrong = candidate(now);
  wrong.source_id = "replay";
  EXPECT_FALSE(arbiter.submit(wrong, now, reason));

  EXPECT_TRUE(arbiter.submit(candidate(now), now, reason));
  EXPECT_FALSE(arbiter.submit(candidate(now), now + 1ms, reason));
  EXPECT_EQ(reason, "candidate sequence is stale or duplicated");

  auto invalid = candidate(now, 2);
  invalid.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(arbiter.submit(invalid, now, reason));
  EXPECT_EQ(reason, "joint command contains a non-finite value");
}

TEST(CommandArbiter, ExpiresCommandsAndLeasesWithoutReplayingStaleData)
{
  const SteadyTime now{};
  CommandArbiter arbiter(50ms);
  arbiter.setEnabled(true, now);
  ASSERT_TRUE(arbiter.acquire("vr", "session-a", 10, 200ms, now).granted);
  std::string reason;
  EXPECT_TRUE(arbiter.submit(candidate(now), now, reason));
  EXPECT_TRUE(arbiter.commands(now + 51ms).empty());
  EXPECT_EQ(arbiter.status(now + 51ms).code, SafetyCode::kHolding);
  EXPECT_EQ(arbiter.status(now + 51ms).reason, "command watchdog expired");
  EXPECT_EQ(arbiter.status(now + 201ms).code, SafetyCode::kReady);
}

TEST(CommandArbiter, KeepsIndependentLatestCommandPerGroup)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  arbiter.setEnabled(true, now);
  ASSERT_TRUE(arbiter.acquire("vr", "session-a", 10, 1s, now).granted);
  std::string reason;

  auto left = candidate(now, 1, "left_arm");
  auto right = candidate(now, 1, "right_arm");
  right.positions = {-0.3, 0.4};
  ASSERT_TRUE(arbiter.submit(left, now, reason)) << reason;
  ASSERT_TRUE(arbiter.submit(right, now, reason)) << reason;

  const auto commands = arbiter.commands(now + 10ms);
  ASSERT_EQ(commands.size(), 2U);
  EXPECT_EQ(commands[0].group_name, "left_arm");
  EXPECT_EQ(commands[1].group_name, "right_arm");
  EXPECT_EQ(commands[1].positions[0], -0.3);
}

TEST(CommandArbiter, SameOwnerRenewalPreservesActiveCommandsAndSequenceHistory)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  arbiter.setEnabled(true, now);
  const auto initial_lease = arbiter.acquire("vr", "session-a", 10, 1s, now);
  ASSERT_TRUE(initial_lease.granted);
  std::string reason;
  ASSERT_TRUE(arbiter.submit(candidate(now), now, reason)) << reason;

  const auto renewed = arbiter.acquire("vr", "session-a", 10, 1s, now + 50ms);
  ASSERT_TRUE(renewed.granted);
  EXPECT_EQ(renewed.reason, "renewed");
  EXPECT_EQ(renewed.lease_id, initial_lease.lease_id);
  EXPECT_EQ(renewed.expires_at, now + 1050ms);
  EXPECT_EQ(arbiter.commands(now + 60ms).size(), 1U);
  EXPECT_EQ(arbiter.status(now + 60ms).code, SafetyCode::kActive);

  EXPECT_FALSE(arbiter.submit(candidate(now, 1U), now + 60ms, reason));
  EXPECT_EQ(reason, "candidate sequence is stale or duplicated");
}

TEST(CommandArbiter, SameSourceSessionRolloverDoesNotWaitForOldLease)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  arbiter.setEnabled(true, now);
  const auto old_lease = arbiter.acquire("vr", "session-a", 10, 1s, now);
  ASSERT_TRUE(old_lease.granted);
  std::string reason;
  ASSERT_TRUE(arbiter.submit(candidate(now), now, reason)) << reason;

  const auto new_lease = arbiter.acquire("vr", "session-b", 10, 1s, now + 50ms);
  ASSERT_TRUE(new_lease.granted);
  EXPECT_NE(new_lease.lease_id, old_lease.lease_id);
  EXPECT_EQ(arbiter.status(now + 50ms).active_session, "session-b");
  EXPECT_TRUE(arbiter.commands(now + 50ms).empty());

  auto old_session = candidate(now, 2U);
  EXPECT_FALSE(arbiter.submit(old_session, now + 51ms, reason));
  EXPECT_EQ(reason, "candidate does not own the active lease");

  auto new_session = candidate(now, 1U);
  new_session.session_id = "session-b";
  EXPECT_TRUE(arbiter.submit(new_session, now + 51ms, reason)) << reason;
}

TEST(CommandArbiter, ExpiresOneGroupWithoutDroppingFreshGroups)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  arbiter.setEnabled(true, now);
  ASSERT_TRUE(arbiter.acquire("vr", "session-a", 10, 1s, now).granted);
  std::string reason;

  auto left = candidate(now, 1, "left_arm");
  auto right = candidate(now, 1, "right_arm");
  right.valid_until = now + 20ms;
  ASSERT_TRUE(arbiter.submit(left, now, reason)) << reason;
  ASSERT_TRUE(arbiter.submit(right, now, reason)) << reason;

  const auto commands = arbiter.commands(now + 25ms);
  ASSERT_EQ(commands.size(), 1U);
  EXPECT_EQ(commands[0].group_name, "left_arm");
  EXPECT_EQ(arbiter.status(now + 25ms).code, SafetyCode::kActive);
}

TEST(CommandArbiter, HigherPrioritySourceMayPreempt)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  ASSERT_TRUE(arbiter.acquire("vr", "session-a", 10, 1s, now).granted);
  EXPECT_FALSE(arbiter.acquire("replay", "session-b", 10, 1s, now).granted);
  const auto preempt = arbiter.acquire("safety", "session-c", 255, 1s, now);
  EXPECT_TRUE(preempt.granted);
  EXPECT_EQ(arbiter.status(now).active_source, "safety");
}

TEST(CommandArbiter, FaultResetAlwaysReturnsDisabled)
{
  const SteadyTime now{};
  CommandArbiter arbiter(150ms);
  arbiter.setEnabled(true, now);
  ASSERT_TRUE(arbiter.acquire("vr", "session-a", 10, 1s, now).granted);
  arbiter.latchFault("joint limit");
  EXPECT_EQ(arbiter.status(now).code, SafetyCode::kFault);
  EXPECT_TRUE(arbiter.resetFault());
  EXPECT_EQ(arbiter.status(now).code, SafetyCode::kDisabled);
}

TEST(CommandArbiter, AcceptsVelocityModeWithoutSyntheticPositions)
{
  hc_teleop_core::CommandArbiter arbiter(150ms);
  const auto now = hc_teleop_core::SteadyTime{};
  arbiter.setEnabled(true, now);
  ASSERT_TRUE(arbiter.acquire("vr", "session-a", 1, 1s, now).granted);
  auto value = candidate(now, 1U);
  value.control_mode = 2U;
  value.positions.clear();
  value.velocities = {0.1, -0.1};
  std::string reason;
  EXPECT_TRUE(arbiter.submit(std::move(value), now, reason)) << reason;
}

}  // namespace
