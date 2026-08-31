#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "hc_motion_backend_kdl/joint_rate_limiter.hpp"

using namespace std::chrono_literals;

TEST(JointRateLimiter, BoundsVelocityAndAccelerationAcrossLargeTargetJump)
{
  hc_motion_backend_kdl::JointRateLimiter limiter(
    {4.0, 2.0}, 0.5, 12.0, 20ms, 250ms, 0.5);
  const auto start = hc_motion_backend_kdl::JointRateLimiter::Time{};
  const auto first = limiter.update({2.0, -2.0}, {0.0, 0.0}, "vr/1", start);
  EXPECT_NEAR(first[0], 12.0 * 0.02 * 0.02, 1e-12);
  EXPECT_NEAR(first[1], -12.0 * 0.02 * 0.02, 1e-12);

  auto previous = first;
  for (int index = 1; index <= 20; ++index) {
    const auto current = limiter.update(
      {2.0, -2.0}, {previous[0], previous[1]}, "vr/1", start + index * 20ms);
    EXPECT_LE(std::abs(current[0] - previous[0]), 2.0 * 0.02 + 1e-12);
    EXPECT_LE(std::abs(current[1] - previous[1]), 1.0 * 0.02 + 1e-12);
    previous = current;
  }
}

TEST(JointRateLimiter, ResetsToFeedbackAfterTimeoutOrSessionChange)
{
  hc_motion_backend_kdl::JointRateLimiter limiter(
    {4.0}, 0.5, 10.0, 20ms, 200ms, 0.5);
  const auto start = hc_motion_backend_kdl::JointRateLimiter::Time{};
  (void)limiter.update({1.0}, {0.0}, "vr/1", start);
  const auto after_timeout = limiter.update({1.5}, {0.4}, "vr/1", start + 300ms);
  EXPECT_GT(after_timeout[0], 0.4);
  EXPECT_LT(after_timeout[0], 0.41);
  const auto new_session = limiter.update({-1.0}, {-0.2}, "vr/2", start + 320ms);
  EXPECT_LT(new_session[0], -0.2);
  EXPECT_GT(new_session[0], -0.21);
}

TEST(JointRateLimiter, RejectsMalformedInput)
{
  EXPECT_THROW(
    hc_motion_backend_kdl::JointRateLimiter({1.0}, 1.5, 10.0, 20ms, 200ms, 0.5),
    std::invalid_argument);
  hc_motion_backend_kdl::JointRateLimiter limiter(
    {1.0}, 0.5, 10.0, 20ms, 200ms, 0.5);
  EXPECT_THROW(
    limiter.update({1.0, 2.0}, {0.0}, "vr/1", {}),
    std::invalid_argument);
}
