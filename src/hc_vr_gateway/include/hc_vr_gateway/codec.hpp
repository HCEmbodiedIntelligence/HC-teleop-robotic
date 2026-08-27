#ifndef HC_VR_GATEWAY__CODEC_HPP_
#define HC_VR_GATEWAY__CODEC_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace hc_vr_gateway
{

inline constexpr std::size_t kLegacyPacketSize = 102;
inline constexpr std::size_t kPacketV2Size = 162;
inline constexpr std::uint8_t kLegacyProtocolVersion = 1;
inline constexpr std::uint8_t kProtocolVersion = 2;

inline constexpr std::uint8_t kHeadTrackedFlag = 1U << 0U;
inline constexpr std::uint8_t kLeftTrackedFlag = 1U << 1U;
inline constexpr std::uint8_t kRightTrackedFlag = 1U << 2U;

struct Pose
{
  std::array<float, 3> position{};
  // The wire protocol and geometry_msgs use x, y, z, w order.
  std::array<float, 4> quaternion{};
};

struct ControllerInput
{
  std::uint16_t held_mask{0};
  std::uint16_t pressed_mask{0};
  std::uint16_t released_mask{0};
  float trigger{0.0F};
  float grip{0.0F};
  std::array<float, 2> primary_axis{};
  std::array<float, 2> secondary_axis{};
};

struct PosePacket
{
  std::uint8_t protocol_version{0};
  std::uint32_t sequence{0};
  double device_timestamp{0.0};
  std::uint8_t tracking_flags{0};
  Pose head{};
  Pose left{};
  Pose right{};
  ControllerInput left_input{};
  ControllerInput right_input{};

  [[nodiscard]] bool tracked(std::uint8_t flag) const noexcept
  {
    return (tracking_flags & flag) != 0U;
  }
};

enum class DecodeStatus
{
  kOk,
  kInvalidSize,
  kInvalidMagic,
  kUnsupportedVersion,
  kNonFinite,
};

struct DecodeResult
{
  DecodeStatus status{DecodeStatus::kInvalidSize};
  PosePacket packet{};
  std::string error{};

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status == DecodeStatus::kOk;
  }
};

/// Decode the fixed little-endian PICO v1/v2 wire format.
[[nodiscard]] DecodeResult decode_pose_packet(
  const std::uint8_t * data, std::size_t size) noexcept;

/// RFC1982-style uint32 comparison used by the original Python gateway.
[[nodiscard]] bool is_sequence_newer(
  std::uint32_t sequence, std::optional<std::uint32_t> previous) noexcept;

/// Number of missing packets between two accepted sequence numbers.
[[nodiscard]] std::uint32_t packet_loss_count(
  std::uint32_t sequence, std::optional<std::uint32_t> previous) noexcept;

}  // namespace hc_vr_gateway

#endif  // HC_VR_GATEWAY__CODEC_HPP_
