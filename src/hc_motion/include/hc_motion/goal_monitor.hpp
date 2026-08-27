#ifndef HC_MOTION__GOAL_MONITOR_HPP_
#define HC_MOTION__GOAL_MONITOR_HPP_

#include <chrono>
#include <map>
#include <optional>
#include <string>

#include "hc_motion/types.hpp"

namespace hc_motion
{

struct GoalMonitorConfig
{
  double joint_position_tolerance_rad{0.01};
  double joint_velocity_tolerance_rad_s{0.02};
  std::chrono::milliseconds stable_duration{200};
  std::chrono::milliseconds move_timeout{60000};
  double cartesian_position_tolerance_m{0.001};
  double cartesian_orientation_tolerance_rad{0.01};
};

enum class GoalObservation : std::uint8_t
{
  kRunning,
  kReached,
  kTimedOut,
  kInvalidFeedback,
};

struct GoalObservationResult
{
  GoalObservation observation{GoalObservation::kRunning};
  MotionStatus status;
};

/// Closed-loop completion monitor. Only measured feedback and backend FK may
/// complete a goal; an open-loop planned endpoint is never treated as proof.
class GoalMonitor
{
public:
  explicit GoalMonitor(GoalMonitorConfig config = {});
  MotionStatus begin(
    const std::string & session_id, const JointTarget & joint_goal,
    const std::optional<Pose> & cartesian_goal, SteadyTime now);
  GoalObservationResult observe(
    const std::string & session_id, const JointFeedback & feedback,
    const std::optional<Pose> & fk_pose, SteadyTime now);
  void erase(const std::string & session_id);
  [[nodiscard]] bool contains(const std::string & session_id) const;

private:
  struct GoalState
  {
    JointTarget joint_goal;
    std::optional<Pose> cartesian_goal;
    SteadyTime started_at{};
    std::optional<SteadyTime> stable_since;
  };

  GoalMonitorConfig config_;
  std::map<std::string, GoalState> goals_;
};

}  // namespace hc_motion

#endif  // HC_MOTION__GOAL_MONITOR_HPP_
