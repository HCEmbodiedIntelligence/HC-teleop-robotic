#ifndef HC_TELEOP_CORE__VR_MAPPER_HPP_
#define HC_TELEOP_CORE__VR_MAPPER_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace hc_teleop_core
{

using MapperTime = std::chrono::steady_clock::time_point;

struct MapperPose
{
  std::array<double, 3> position{};
  std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
};

enum class ControllerSide : std::uint8_t {kLeft, kRight};

struct ArmBinding
{
  std::string group_name;
  ControllerSide controller{ControllerSide::kLeft};
  std::string reference_frame;
  std::string tip_frame;
  // Optional VR-to-reference-frame basis. When unset, MapperConfig's global
  // basis is used. Mirrored arm base frames require distinct bases even when
  // both controllers should move in the same robot-body direction.
  std::optional<std::array<double, 9>> axis_mapping;
};

struct MapperConfig
{
  std::vector<ArmBinding> bindings;
  std::array<double, 9> axis_mapping{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  double position_scale{1.0};
  double clutch_threshold{0.5};
  std::chrono::milliseconds feedback_max_age{200};
  // When set, all arm bindings use this controller's grip as the clutch.
  // An unset value preserves the legacy per-binding behavior.
  std::optional<ControllerSide> clutch_controller;
};

struct ControllerFrame
{
  bool tracked{false};
  double grip{0.0};
  MapperPose pose;
  double trigger{0.0};
};

struct MapperFrame
{
  std::string session_id;
  std::uint32_t sequence{0U};
  ControllerFrame left;
  ControllerFrame right;
};

struct MappedTarget
{
  std::string group_name;
  std::string reference_frame;
  std::string tip_frame;
  MapperPose pose;
};

/// Stateful relative VR mapper. Clutch engagement captures the latest measured
/// FK pose as the robot anchor, so restarting VR cannot cause an absolute jump.
class VrMapper
{
public:
  explicit VrMapper(MapperConfig config);
  bool updateFeedback(
    const std::string & group_name, const MapperPose & pose,
    MapperTime received_at, std::string & reason);
  std::vector<MappedTarget> map(const MapperFrame & frame, MapperTime now);
  void reset();

private:
  struct Feedback
  {
    MapperPose pose;
    MapperTime received_at{};
  };
  struct ClutchState
  {
    bool engaged{false};
    MapperPose controller_anchor;
    MapperPose robot_anchor;
  };

  [[nodiscard]] bool validPose(const MapperPose & pose) const;
  [[nodiscard]] MapperPose apply(
    const MapperPose & controller, const ClutchState & clutch,
    const std::array<double, 9> & axis_mapping) const;

  MapperConfig config_;
  std::map<std::string, Feedback> feedback_;
  std::map<std::string, ClutchState> clutches_;
  std::string session_id_;
  std::optional<std::uint32_t> last_sequence_;
};

}  // namespace hc_teleop_core

#endif  // HC_TELEOP_CORE__VR_MAPPER_HPP_
