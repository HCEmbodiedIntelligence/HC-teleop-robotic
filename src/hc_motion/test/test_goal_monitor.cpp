#include <chrono>
#include <optional>

#include <gtest/gtest.h>

#include "hc_motion/goal_monitor.hpp"

namespace
{

using namespace std::chrono_literals;

hc_motion::JointTarget goal()
{
  return {{"j1", "j2"}, {0.1, -0.2}, {}, {}};
}

hc_motion::JointFeedback feedback(double velocity = 0.0)
{
  return {{"j2", "j1"}, {-0.2, 0.1}, {velocity, velocity}, hc_motion::SteadyTime{}};
}

TEST(GoalMonitor, RequiresContinuousStableMeasuredFeedback)
{
  hc_motion::GoalMonitorConfig config;
  config.stable_duration = 100ms;
  hc_motion::GoalMonitor monitor(config);
  const auto now = hc_motion::SteadyTime{};
  ASSERT_TRUE(monitor.begin("goal", goal(), std::nullopt, now).ok());
  EXPECT_EQ(monitor.observe("goal", feedback(), std::nullopt, now).observation,
    hc_motion::GoalObservation::kRunning);
  EXPECT_EQ(monitor.observe("goal", feedback(0.2), std::nullopt, now + 60ms).observation,
    hc_motion::GoalObservation::kRunning);
  EXPECT_EQ(monitor.observe("goal", feedback(), std::nullopt, now + 70ms).observation,
    hc_motion::GoalObservation::kRunning);
  EXPECT_EQ(monitor.observe("goal", feedback(), std::nullopt, now + 171ms).observation,
    hc_motion::GoalObservation::kReached);
}

TEST(GoalMonitor, CartesianGoalRequiresBackendFk)
{
  hc_motion::GoalMonitor monitor;
  const auto now = hc_motion::SteadyTime{};
  hc_motion::Pose pose;
  ASSERT_TRUE(monitor.begin("goal", goal(), pose, now).ok());
  const auto missing = monitor.observe("goal", feedback(), std::nullopt, now);
  EXPECT_EQ(missing.observation, hc_motion::GoalObservation::kInvalidFeedback);
  EXPECT_EQ(missing.status.code, hc_motion::StatusCode::kBackendError);
}

TEST(GoalMonitor, UsesSteadyClockTimeout)
{
  hc_motion::GoalMonitorConfig config;
  config.move_timeout = 1s;
  hc_motion::GoalMonitor monitor(config);
  const auto now = hc_motion::SteadyTime{};
  ASSERT_TRUE(monitor.begin("goal", goal(), std::nullopt, now).ok());
  const auto result = monitor.observe("goal", feedback(), std::nullopt, now + 1001ms);
  EXPECT_EQ(result.observation, hc_motion::GoalObservation::kTimedOut);
}

}  // namespace
