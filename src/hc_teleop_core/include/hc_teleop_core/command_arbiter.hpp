#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hc_teleop_core
{

using SteadyTime = std::chrono::steady_clock::time_point;
using Duration = std::chrono::steady_clock::duration;

enum class SafetyCode : std::uint8_t
{
  kDisabled = 0,
  kReady = 1,
  kActive = 2,
  kHolding = 3,
  kFault = 4,
};

struct JointCandidate
{
  std::string source_id;
  std::string session_id;
  std::string group_name;
  std::uint64_t sequence{0};
  std::uint8_t control_mode{0};
  SteadyTime valid_until{};
  std::int64_t valid_until_ros_nanoseconds{0};
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

struct ControlLease
{
  std::string lease_id;
  std::string source_id;
  std::string session_id;
  std::uint8_t priority{0};
  SteadyTime expires_at{};
};

struct AcquireResult
{
  bool granted{false};
  std::string lease_id;
  std::string reason;
  SteadyTime expires_at{};
};

struct ArbiterStatus
{
  SafetyCode code{SafetyCode::kDisabled};
  bool enabled{false};
  bool fault_latched{false};
  std::string reason{"disabled"};
  std::string active_source;
  std::string active_session;
  std::string lease_id;
  SteadyTime lease_expires_at{};
};

class CommandArbiter
{
public:
  explicit CommandArbiter(Duration command_timeout);

  void setEnabled(bool enabled, SteadyTime now);
  void latchFault(std::string reason);
  bool resetFault();

  AcquireResult acquire(
    std::string source_id,
    std::string session_id,
    std::uint8_t priority,
    Duration duration,
    SteadyTime now);
  bool release(const std::string & lease_id, SteadyTime now);

  bool submit(JointCandidate candidate, SteadyTime now, std::string & rejection_reason);
  std::vector<JointCandidate> commands(SteadyTime now);
  std::optional<JointCandidate> command(SteadyTime now);
  ArbiterStatus status(SteadyTime now) const;

private:
  struct TimedCandidate
  {
    JointCandidate candidate;
    SteadyTime received_at{};
  };

  bool leaseActive(SteadyTime now) const;
  void expireIfNeeded(SteadyTime now);
  void clearRuntimeState();
  static bool validCandidate(const JointCandidate & candidate, std::string & reason);
  static bool sequenceNewer(std::uint64_t sequence, std::uint64_t previous);
  static std::string makeLeaseId(
    const std::string & source_id,
    const std::string & session_id,
    std::uint64_t nonce);

  Duration command_timeout_;
  bool enabled_{false};
  bool fault_latched_{false};
  std::string fault_reason_;
  std::optional<ControlLease> lease_;
  std::unordered_map<std::string, TimedCandidate> latest_by_group_;
  std::unordered_map<std::string, std::uint64_t> last_sequence_by_stream_;
  std::uint64_t lease_nonce_{0};
  std::string hold_reason_{"disabled"};
};

}  // namespace hc_teleop_core
