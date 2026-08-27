#ifndef HC_MOTION__TYPES_HPP_
#define HC_MOTION__TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace hc_motion
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

enum class MotionKind : std::uint8_t
{
  kMoveJ,
  kMoveL,
  kMoveP,
  kServoJ,
  kServoP,
};

enum class StatusCode : std::uint8_t
{
  kOk,
  kInvalidArgument,
  kNotConfigured,
  kRejected,
  kPreempted,
  kCanceled,
  kTimeout,
  kStaleFeedback,
  kBackendError,
  kLimitViolation,
  kControlledStop,
  kInternalError,
};

struct MotionStatus
{
  StatusCode code{StatusCode::kOk};
  std::string message;
  std::string backend_api;
  std::int64_t backend_code{-1};

  MotionStatus() = default;
  MotionStatus(
    StatusCode status_code, std::string status_message,
    std::string api = {}, std::int64_t native_code = -1)
  : code(status_code), message(std::move(status_message)),
    backend_api(std::move(api)), backend_code(native_code)
  {
  }

  [[nodiscard]] bool ok() const noexcept {return code == StatusCode::kOk;}
  static MotionStatus Ok() {return {};}
};

struct Pose
{
  std::array<double, 3> position_m{{0.0, 0.0, 0.0}};
  std::array<double, 4> orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct JointTarget
{
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<double> accelerations_rad_s2;
};

struct JointFeedback
{
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  SteadyTime received_at{};
};

struct JointGroupModel
{
  std::string name;
  std::vector<std::string> joint_names;
  std::vector<double> lower_position_rad;
  std::vector<double> upper_position_rad;
};

[[nodiscard]] inline bool is_servo(MotionKind kind) noexcept
{
  return kind == MotionKind::kServoJ || kind == MotionKind::kServoP;
}

[[nodiscard]] inline bool is_move(MotionKind kind) noexcept
{
  return !is_servo(kind);
}

}  // namespace hc_motion

#endif  // HC_MOTION__TYPES_HPP_
