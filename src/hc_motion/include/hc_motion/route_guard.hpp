#ifndef HC_MOTION__ROUTE_GUARD_HPP_
#define HC_MOTION__ROUTE_GUARD_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "hc_motion/types.hpp"

namespace hc_motion
{

struct CartesianTargetDescriptor
{
  std::string group_name;
  std::string reference_frame;
  std::string tip_frame;
  std::array<double, 3> position{};
  std::array<double, 4> orientation{};
};

struct TargetEnvelope
{
  std::string source_id;
  std::string session_id;
  std::uint64_t sequence{0U};
  SteadyTime valid_until{};
  std::vector<CartesianTargetDescriptor> targets;
};

struct CandidateEnvelope
{
  std::string source_id;
  std::string session_id;
  std::string group_name;
  std::uint64_t sequence{0U};
  SteadyTime valid_until{};
  std::uint8_t control_mode{0U};
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

/// Enforces the process boundary: a backend candidate is accepted only when
/// it answers a live routed target and advances its own component group.
class RouteGuard
{
public:
  explicit RouteGuard(std::vector<std::string> allowed_groups);
  bool route(const TargetEnvelope & envelope, SteadyTime now, std::string & reason);
  bool authorize(const CandidateEnvelope & envelope, SteadyTime now, std::string & reason);
  void prune(SteadyTime now);

private:
  struct Authorization
  {
    std::uint64_t sequence{0U};
    SteadyTime valid_until{};
  };

  static bool sequenceNewer(std::uint64_t sequence, std::uint64_t previous);
  static std::string key(
    const std::string & source, const std::string & session, const std::string & group);

  std::set<std::string> allowed_groups_;
  std::map<std::string, std::uint64_t> last_sequence_;
  std::map<std::string, std::map<std::uint64_t, Authorization>> authorizations_;
  std::map<std::string, std::uint64_t> last_candidate_sequence_;
};

}  // namespace hc_motion

#endif  // HC_MOTION__ROUTE_GUARD_HPP_
