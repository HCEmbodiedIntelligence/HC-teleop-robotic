#ifndef HC_MOTION__CONTROL_ARBITER_HPP_
#define HC_MOTION__CONTROL_ARBITER_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "hc_motion/types.hpp"

namespace hc_motion
{

struct EndpointPolicy
{
  std::string endpoint_name;
  MotionKind kind{MotionKind::kMoveJ};
  std::string group_name;
  std::int64_t priority{0};
  std::chrono::milliseconds servo_lease{100};
};

struct CommandClaim
{
  std::string session_id;
  std::string endpoint_name;
  std::string group_name;
  MotionKind kind{MotionKind::kMoveJ};
  std::vector<std::string> joint_names;
};

struct ArbitrationResult
{
  MotionStatus status;
  bool admitted{false};
  std::vector<std::string> winner_session_ids;
  std::vector<std::string> preempted_move_ids;
  std::vector<std::string> expired_servo_ids;
};

/// Group-aware joint-name conflict arbiter for the HC motion runtime. Endpoint
/// priorities are configured locally and are never accepted from commands.
class ControlArbiter
{
public:
  MotionStatus registerEndpoint(const EndpointPolicy & policy);
  ArbitrationResult submitMove(const CommandClaim & claim, SteadyTime now);
  ArbitrationResult updateServo(const CommandClaim & claim, SteadyTime now);
  ArbitrationResult evaluate(SteadyTime now);
  void release(const std::string & session_id);
  [[nodiscard]] bool contains(const std::string & session_id) const;
  [[nodiscard]] std::vector<std::string> winners(SteadyTime now) const;

private:
  struct ClaimState
  {
    CommandClaim claim;
    std::int64_t priority{0};
    std::uint64_t receive_sequence{0};
    SteadyTime received_at{};
    std::chrono::milliseconds lease{100};
  };

  MotionStatus validateClaim(const CommandClaim & claim, bool servo) const;
  std::vector<std::string> computeWinners(SteadyTime now) const;
  std::vector<std::string> removePreemptedMoves(
    const std::vector<std::string> & old_winners,
    const std::vector<std::string> & new_winners);
  std::vector<std::string> removeExpiredServos(SteadyTime now);
  ArbitrationResult snapshot(
    MotionStatus status, bool admitted, std::vector<std::string> preempted,
    std::vector<std::string> expired, SteadyTime now) const;

  std::map<std::string, EndpointPolicy> policies_;
  std::map<std::string, ClaimState> claims_;
  std::uint64_t next_sequence_{1U};
};

}  // namespace hc_motion

#endif  // HC_MOTION__CONTROL_ARBITER_HPP_
