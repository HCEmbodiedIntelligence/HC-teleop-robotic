#include "hc_motion_backend_kdl/joint_rate_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hc_motion_backend_kdl
{

JointRateLimiter::JointRateLimiter(
  std::vector<double> velocity_limits,
  const double velocity_scale,
  const double acceleration_limit,
  const std::chrono::duration<double> nominal_period,
  const std::chrono::duration<double> reset_timeout,
  const double tracking_error_reset)
: velocity_limits_(std::move(velocity_limits)),
  velocity_scale_(velocity_scale),
  acceleration_limit_(acceleration_limit),
  nominal_period_(nominal_period),
  reset_timeout_(reset_timeout),
  tracking_error_reset_(tracking_error_reset)
{
  if (velocity_limits_.empty() ||
    !std::all_of(velocity_limits_.begin(), velocity_limits_.end(), [](const double value) {
      return std::isfinite(value) && value > 0.0;
    }) ||
    !std::isfinite(velocity_scale_) || velocity_scale_ <= 0.0 || velocity_scale_ > 1.0 ||
    !std::isfinite(acceleration_limit_) || acceleration_limit_ <= 0.0 ||
    nominal_period_ <= std::chrono::duration<double>::zero() ||
    reset_timeout_ <= nominal_period_ ||
    !std::isfinite(tracking_error_reset_) || tracking_error_reset_ <= 0.0)
  {
    throw std::invalid_argument("joint rate limiter configuration is invalid");
  }
}

std::vector<double> JointRateLimiter::update(
  const std::vector<double> & desired,
  const std::vector<double> & measured,
  const std::string & session_id,
  const Time now)
{
  if (desired.size() != velocity_limits_.size() || measured.size() != desired.size() ||
    session_id.empty() ||
    !std::all_of(desired.begin(), desired.end(), [](const double value) {
      return std::isfinite(value);
    }) ||
    !std::all_of(measured.begin(), measured.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    throw std::invalid_argument("joint rate limiter input is invalid");
  }

  bool needs_reset = !initialized_ || session_id != session_id_ || now < last_update_ ||
    now - last_update_ > reset_timeout_;
  if (!needs_reset) {
    for (std::size_t index = 0; index < measured.size(); ++index) {
      if (std::abs(command_[index] - measured[index]) > tracking_error_reset_) {
        needs_reset = true;
        break;
      }
    }
  }
  if (needs_reset) {
    initialize(measured, session_id, now);
  }

  auto period = initialized_ && last_update_ < now ?
    std::chrono::duration<double>(now - last_update_) : nominal_period_;
  // A Wi-Fi stall must not grant a proportionally larger one-frame joint step.
  period = std::clamp(period, nominal_period_ * 0.25, nominal_period_ * 2.0);
  const double dt = period.count();

  for (std::size_t index = 0; index < desired.size(); ++index) {
    const double remaining = desired[index] - command_[index];
    const double maximum_velocity = velocity_limits_[index] * velocity_scale_;
    const double requested_velocity = std::clamp(remaining / dt, -maximum_velocity, maximum_velocity);
    const double maximum_velocity_change = acceleration_limit_ * dt;
    double next_velocity = velocity_[index] + std::clamp(
      requested_velocity - velocity_[index], -maximum_velocity_change, maximum_velocity_change);
    double step = next_velocity * dt;
    if (step * remaining <= 0.0 || std::abs(step) >= std::abs(remaining)) {
      step = remaining;
      next_velocity = step / dt;
    }
    command_[index] += step;
    velocity_[index] = next_velocity;
  }
  last_update_ = now;
  return command_;
}

void JointRateLimiter::reset() noexcept
{
  initialized_ = false;
  command_.clear();
  velocity_.clear();
  session_id_.clear();
}

void JointRateLimiter::initialize(
  const std::vector<double> & measured, const std::string & session_id, const Time now)
{
  command_ = measured;
  velocity_.assign(measured.size(), 0.0);
  session_id_ = session_id;
  // Use a nominal period for the first output rather than producing no motion.
  last_update_ = now - std::chrono::duration_cast<Time::duration>(nominal_period_);
  initialized_ = true;
}

}  // namespace hc_motion_backend_kdl
