#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "hc_diagnostics/control_chain_monitor.hpp"

using namespace std::chrono_literals;

TEST(ControlChainMonitor, CorrelatesStagesBySessionSequenceAndGroup)
{
  hc_diagnostics::ControlChainMonitor monitor;
  const hc_diagnostics::SteadyTime start{};
  EXPECT_TRUE(monitor.observeVr("pico/1", 10U, {1000.0, 0.5}, start).empty());
  monitor.observeTarget("pico/1", 10U, {"left_arm"}, start + 2ms);
  EXPECT_TRUE(monitor.observeCandidate(
      "pico/1", 10U, "left_arm", {0.1, 0.2}, start + 7ms).empty());
  EXPECT_TRUE(monitor.observeCommand(
      "pico/1", 10U, "left_arm", start + 9ms).empty());

  const auto snapshot = monitor.snapshot(start + 10ms);
  EXPECT_EQ(snapshot.vr_to_target.samples, 1U);
  EXPECT_NEAR(snapshot.vr_to_target.last_ms, 2.0, 1e-9);
  EXPECT_NEAR(snapshot.target_to_candidate.last_ms, 5.0, 1e-9);
  EXPECT_NEAR(snapshot.candidate_to_command.last_ms, 2.0, 1e-9);
  EXPECT_NEAR(snapshot.vr_to_command.last_ms, 9.0, 1e-9);
  EXPECT_EQ(snapshot.anomaly_count, 0U);
}

TEST(ControlChainMonitor, DetectsGapsLatencyJointJumpsAndMissingCandidates)
{
  hc_diagnostics::MonitorConfig config;
  config.vr_receive_gap_warn_ms = 20.0;
  config.vr_callback_delay_warn_ms = 10.0;
  config.ik_latency_warn_ms = 10.0;
  config.joint_step_warn_rad = 0.2;
  config.missing_candidate_warn_ms = 30.0;
  hc_diagnostics::ControlChainMonitor monitor(config);
  const hc_diagnostics::SteadyTime start{};
  (void)monitor.observeVr("pico/1", 1U, {1000.0, 1.0}, start);
  auto anomalies = monitor.observeVr("pico/1", 2U, {1025.0, 2.0}, start + 25ms);
  ASSERT_EQ(anomalies.size(), 1U);
  EXPECT_EQ(anomalies.front().code, "vr_receive_gap");

  monitor.observeTarget("pico/1", 2U, {"left_arm"}, start + 26ms);
  anomalies = monitor.observeCandidate(
    "pico/1", 2U, "left_arm", {0.0, 0.0}, start + 40ms);
  ASSERT_EQ(anomalies.size(), 1U);
  EXPECT_EQ(anomalies.front().code, "ik_latency");
  anomalies = monitor.observeCandidate(
    "pico/1", 3U, "left_arm", {0.3, 0.0}, start + 50ms);
  ASSERT_EQ(anomalies.size(), 1U);
  EXPECT_EQ(anomalies.front().code, "ik_joint_step");

  monitor.observeTarget("pico/1", 4U, {"right_arm"}, start + 60ms);
  anomalies = monitor.tick(start + 91ms);
  ASSERT_EQ(anomalies.size(), 1U);
  EXPECT_EQ(anomalies.front().code, "missing_ik_candidate");
  EXPECT_TRUE(monitor.tick(start + 100ms).empty());
}

TEST(ControlChainMonitor, SeparatesGatewayReceiveTimingFromCallbackScheduling)
{
  hc_diagnostics::MonitorConfig config;
  config.vr_receive_gap_warn_ms = 20.0;
  config.vr_callback_delay_warn_ms = 10.0;
  hc_diagnostics::ControlChainMonitor monitor(config);
  const hc_diagnostics::SteadyTime start{};

  EXPECT_TRUE(monitor.observeVr("pico/1", 1U, {1000.0, 1.0}, start).empty());
  // The ROS callback ran 25 ms later, but the UDP gateway received frames at
  // the expected 16 ms interval. This must not be reported as a receive gap.
  EXPECT_TRUE(monitor.observeVr(
      "pico/1", 2U, {1016.0, 9.0}, start + 25ms).empty());

  auto anomalies = monitor.observeVr(
    "pico/1", 3U, {1045.0, 15.0}, start + 30ms);
  ASSERT_EQ(anomalies.size(), 2U);
  EXPECT_EQ(anomalies[0].code, "vr_receive_gap");
  EXPECT_EQ(anomalies[1].code, "vr_callback_delay");

  const auto snapshot = monitor.snapshot(start + 31ms);
  EXPECT_NEAR(snapshot.vr_receive_gap.last_ms, 29.0, 1e-9);
  EXPECT_NEAR(snapshot.vr_callback_delay.last_ms, 15.0, 1e-9);
  EXPECT_NEAR(snapshot.streams.at("vr").max_gap_ms, 25.0, 1e-9);

  // A new endpoint/session resets receive-gap correlation rather than
  // reporting the time between unrelated sessions as a giant gap.
  EXPECT_TRUE(monitor.observeVr(
      "pico/2", 0U, {5000.0, 1.0}, start + 40ms).empty());
}

TEST(ControlChainMonitor, TracksCommandToFeedbackError)
{
  hc_diagnostics::ControlChainMonitor monitor;
  const hc_diagnostics::SteadyTime start{};
  monitor.setCommandReference("left_arm", {"j1", "j2"}, {1.0, -1.0});
  monitor.observeJointFeedback({"j2", "j1"}, {-0.8, 0.9}, start);
  const auto snapshot = monitor.snapshot(start);
  ASSERT_EQ(snapshot.feedback_error_rad.count("left_arm"), 1U);
  EXPECT_NEAR(snapshot.feedback_error_rad.at("left_arm"), std::sqrt(0.05), 1e-12);
}
