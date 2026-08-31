#pragma once

#include <chrono>
#include <string>
#include <vector>

namespace hc_motion_backend_kdl
{

class JointRateLimiter
{
public:
  using Time = std::chrono::steady_clock::time_point;

  JointRateLimiter(
    std::vector<double> velocity_limits,
    double velocity_scale,
    double acceleration_limit,
    std::chrono::duration<double> nominal_period,
    std::chrono::duration<double> reset_timeout,
    double tracking_error_reset);

  std::vector<double> update(
    const std::vector<double> & desired,
    const std::vector<double> & measured,
    const std::string & session_id,
    Time now);

  void reset() noexcept;

private:
  void initialize(const std::vector<double> & measured, const std::string & session_id, Time now);

  std::vector<double> velocity_limits_;
  double velocity_scale_;
  double acceleration_limit_;
  std::chrono::duration<double> nominal_period_;
  std::chrono::duration<double> reset_timeout_;
  double tracking_error_reset_;
  std::vector<double> command_;
  std::vector<double> velocity_;
  std::string session_id_;
  Time last_update_{};
  bool initialized_{false};
};

}  // namespace hc_motion_backend_kdl
