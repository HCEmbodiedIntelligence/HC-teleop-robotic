#include <gtest/gtest.h>
#include <limits>
#include "feedback_cache.hpp"
using hc_motion_backend_robo_manip::FeedbackCache;
using namespace std::chrono_literals;

TEST(FeedbackCache, PartialUpdatesDoNotRefreshOtherJoints) {
  FeedbackCache cache;
  sensor_msgs::msg::JointState message;
  message.name = {"left", "right"}; message.position = {0.1, 0.2};
  const auto start = FeedbackCache::SteadyClock::now();
  ASSERT_TRUE(cache.update(message, start));
  message.name = {"right"}; message.position = {0.3};
  ASSERT_TRUE(cache.update(message, start + 300ms));
  FeedbackCache::JointFeedback feedback;
  ASSERT_TRUE(cache.select({"left", "right"}, feedback));
  EXPECT_EQ(feedback.received_at, start);
  EXPECT_EQ(feedback.positions_rad, (std::vector<double>{0.1, 0.3}));
  ASSERT_TRUE(cache.select({"right"}, feedback));
  EXPECT_EQ(feedback.received_at, start + 300ms);
  EXPECT_FALSE(cache.select({"missing"}, feedback));
}
TEST(FeedbackCache, InvalidMessagesAreAtomicAndDoNotRenewAge) {
  FeedbackCache cache;
  sensor_msgs::msg::JointState message;
  message.name = {"left", "right"}; message.position = {0.1, 0.2};
  const auto start = FeedbackCache::SteadyClock::now();
  ASSERT_TRUE(cache.update(message, start));
  message.position = {9.0, std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(cache.update(message, start + 300ms));
  EXPECT_FALSE(cache.update(sensor_msgs::msg::JointState{}, start + 300ms));
  message.position = {9.0};
  EXPECT_FALSE(cache.update(message, start + 300ms));
  FeedbackCache::JointFeedback feedback;
  ASSERT_TRUE(cache.select({"left", "right"}, feedback));
  EXPECT_EQ(feedback.received_at, start);
  EXPECT_EQ(feedback.positions_rad, (std::vector<double>{0.1, 0.2}));
}
