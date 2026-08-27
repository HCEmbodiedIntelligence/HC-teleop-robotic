#ifndef HC_DEVICE_SDK__TYPES_HPP_
#define HC_DEVICE_SDK__TYPES_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace hc_device_sdk
{

enum class ComponentKind : std::uint8_t
{
  kUnknown = 0,
  kArm,
  kGripper,
  kDexterousHand,
  kCamera,
  kWaist,
  kBase,
  kSensor,
};

const char * toString(ComponentKind kind) noexcept;
bool componentKindFromString(const std::string & value, ComponentKind & output) noexcept;

enum class LifecycleState : std::uint8_t
{
  kUnconfigured = 0,
  kDisconnected,
  kInactive,
  kActive,
  kStopped,
  kFaulted,
};

enum class HealthLevel : std::uint8_t
{
  kOk = 0,
  kWarning,
  kError,
  kStale,
};

enum class DeviceError : std::uint8_t
{
  kNone = 0,
  kInvalidConfiguration,
  kInvalidState,
  kNotConnected,
  kNotActive,
  kCommunication,
  kRejectedCommand,
  kUnsupported,
  kInternal,
};

struct DeviceResult
{
  bool successful{true};
  DeviceError error{DeviceError::kNone};
  std::string message;

  explicit operator bool() const noexcept {return successful;}

  static DeviceResult success(std::string detail = {})
  {
    return DeviceResult{true, DeviceError::kNone, std::move(detail)};
  }

  static DeviceResult failure(DeviceError code, std::string detail)
  {
    return DeviceResult{false, code, std::move(detail)};
  }
};

struct ComponentDescriptor
{
  std::string component_id;
  ComponentKind kind{ComponentKind::kUnknown};
  std::string display_name;
  std::string model;
  std::uint32_t dof{0};
};

struct DeviceCapabilities
{
  ComponentDescriptor component;
  std::vector<std::string> capabilities;
  std::vector<std::string> command_interfaces;
  std::vector<std::string> state_interfaces;
};

// logical_position_rad = vendor_to_logical_scale * vendor_position +
// vendor_to_logical_offset_rad. Velocity and acceleration use the same scale;
// effort uses the reciprocal transform required by virtual work.
struct JointMapping
{
  std::string logical_name;
  std::string vendor_name;
  std::string vendor_group;
  double vendor_to_logical_scale{1.0};
  double vendor_to_logical_offset_rad{0.0};
};

using ParameterValue = std::variant<
  bool,
  std::int64_t,
  double,
  std::string,
  std::vector<std::int64_t>,
  std::vector<double>,
  std::vector<std::string>>;

struct DeviceConfiguration
{
  ComponentDescriptor component;
  std::vector<JointMapping> joint_mappings;
  std::map<std::string, ParameterValue> parameters;
};

struct DeviceHealth
{
  HealthLevel level{HealthLevel::kStale};
  LifecycleState lifecycle{LifecycleState::kUnconfigured};
  bool communication_ok{false};
  bool safe_stop_engaged{true};
  std::string component_id;
  std::string message{"not configured"};
  std::map<std::string, std::string> details;
  std::chrono::steady_clock::time_point observed_at{std::chrono::steady_clock::now()};
};

enum class JointControlMode : std::uint8_t
{
  kPosition = 1,
  kVelocity = 2,
  kEffort = 3,
};

// All values in the logical structs use SI units and are associated by name.
struct JointState
{
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<double> efforts_nm;
  std::chrono::steady_clock::time_point sample_time{std::chrono::steady_clock::now()};
};

struct JointCommand
{
  JointControlMode mode{JointControlMode::kPosition};
  std::string group_name;
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<double> accelerations_rad_s2;
  std::vector<double> efforts_nm;
};

// Vendor structs deliberately do not claim SI units. They only exist on the
// vendor side of JointMappingTable.
struct VendorJointState
{
  std::vector<std::string> vendor_joint_names;
  // Empty means the vendor protocol has no group field. Otherwise this must
  // have the same length as vendor_joint_names.
  std::vector<std::string> vendor_groups;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  std::chrono::steady_clock::time_point sample_time{std::chrono::steady_clock::now()};
};

struct VendorJointCommand
{
  JointControlMode mode{JointControlMode::kPosition};
  std::string logical_group_name;
  std::vector<std::string> vendor_joint_names;
  std::vector<std::string> vendor_groups;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> efforts;
};

}  // namespace hc_device_sdk

#endif  // HC_DEVICE_SDK__TYPES_HPP_
