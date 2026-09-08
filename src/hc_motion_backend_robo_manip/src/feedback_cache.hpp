#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include "sensor_msgs/msg/joint_state.hpp"
#include "humanoid_motion_server/motion/types.hpp"

namespace hc_motion_backend_robo_manip {
class FeedbackCache {
public:
  using JointFeedback = humanoid_motion_server::motion::JointFeedback;
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;
  bool update(const sensor_msgs::msg::JointState & message, SteadyTime received_at) {
    if (message.name.empty() || message.name.size() != message.position.size() ||
      (!message.velocity.empty() && message.velocity.size() != message.name.size())) {
      return false;
    }
    std::map<std::string, double> received;
    for (std::size_t index = 0; index < message.name.size(); ++index) {
      if (message.name[index].empty() || !std::isfinite(message.position[index]) ||
        (!message.velocity.empty() && !std::isfinite(message.velocity[index])) ||
        !received.emplace(message.name[index], message.position[index]).second) {
        return false;
      }
    }
    for (std::size_t index = 0; index < message.name.size(); ++index) {
      const auto & name = message.name[index];
      positions_[name] = message.position[index];
      velocities_[name] = message.velocity.empty() ? 0.0 : message.velocity[index];
      feedback_times_[name] = received_at;
    }
    return true;
  }
  bool select(const std::vector<std::string> & names, JointFeedback & output) const {
    output = JointFeedback{};
    output.received_at = SteadyTime::max();
    output.joint_names = names;
    output.positions_rad.reserve(names.size());
    for (const auto & name : names) {
      const auto found = positions_.find(name);
      if (found == positions_.end()) {
        return false;
      }
      output.positions_rad.push_back(found->second);
      output.velocities_rad_s.push_back(velocities_.at(name));
      output.received_at = std::min(output.received_at, feedback_times_.at(name));
    }
    return true;
  }
private:
  std::unordered_map<std::string, double> positions_;
  std::unordered_map<std::string, double> velocities_;
  std::unordered_map<std::string, SteadyTime> feedback_times_;
};
}
