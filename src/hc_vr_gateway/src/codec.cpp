#include "hc_vr_gateway/codec.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace hc_vr_gateway
{
namespace
{

class LittleEndianReader
{
public:
  LittleEndianReader(const std::uint8_t * data, std::size_t size) noexcept
  : data_(data), size_(size)
  {
  }

  bool bytes(std::uint8_t * destination, std::size_t count) noexcept
  {
    if (destination == nullptr || count > size_ - offset_) {
      return false;
    }
    std::memcpy(destination, data_ + offset_, count);
    offset_ += count;
    return true;
  }

  bool uint8(std::uint8_t & value) noexcept
  {
    if (offset_ >= size_) {
      return false;
    }
    value = data_[offset_++];
    return true;
  }

  bool uint16(std::uint16_t & value) noexcept
  {
    if (2U > size_ - offset_) {
      return false;
    }
    value = static_cast<std::uint16_t>(data_[offset_]) |
      (static_cast<std::uint16_t>(data_[offset_ + 1U]) << 8U);
    offset_ += 2U;
    return true;
  }

  bool uint32(std::uint32_t & value) noexcept
  {
    if (4U > size_ - offset_) {
      return false;
    }
    value = static_cast<std::uint32_t>(data_[offset_]) |
      (static_cast<std::uint32_t>(data_[offset_ + 1U]) << 8U) |
      (static_cast<std::uint32_t>(data_[offset_ + 2U]) << 16U) |
      (static_cast<std::uint32_t>(data_[offset_ + 3U]) << 24U);
    offset_ += 4U;
    return true;
  }

  bool uint64(std::uint64_t & value) noexcept
  {
    if (8U > size_ - offset_) {
      return false;
    }
    value = 0U;
    for (std::size_t index = 0; index < 8U; ++index) {
      value |= static_cast<std::uint64_t>(data_[offset_ + index]) << (index * 8U);
    }
    offset_ += 8U;
    return true;
  }

  bool float32(float & value) noexcept
  {
    static_assert(sizeof(float) == sizeof(std::uint32_t), "32-bit float required");
    std::uint32_t bits = 0U;
    if (!uint32(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  bool float64(double & value) noexcept
  {
    static_assert(sizeof(double) == sizeof(std::uint64_t), "64-bit double required");
    std::uint64_t bits = 0U;
    if (!uint64(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  [[nodiscard]] std::size_t remaining() const noexcept
  {
    return size_ - offset_;
  }

private:
  const std::uint8_t * data_;
  std::size_t size_;
  std::size_t offset_{0U};
};

bool read_pose(LittleEndianReader & reader, Pose & pose) noexcept
{
  for (float & value : pose.position) {
    if (!reader.float32(value) || !std::isfinite(value)) {
      return false;
    }
  }
  for (float & value : pose.quaternion) {
    if (!reader.float32(value) || !std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool read_controller_input(
  LittleEndianReader & reader, ControllerInput & input) noexcept
{
  if (!reader.uint16(input.held_mask) ||
    !reader.uint16(input.pressed_mask) ||
    !reader.uint16(input.released_mask))
  {
    return false;
  }
  if (!reader.float32(input.trigger) || !std::isfinite(input.trigger) ||
    !reader.float32(input.grip) || !std::isfinite(input.grip))
  {
    return false;
  }
  for (float & value : input.primary_axis) {
    if (!reader.float32(value) || !std::isfinite(value)) {
      return false;
    }
  }
  for (float & value : input.secondary_axis) {
    if (!reader.float32(value) || !std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

DecodeResult failure(DecodeStatus status, const char * message) noexcept
{
  DecodeResult result;
  result.status = status;
  try {
    result.error = message;
  } catch (...) {
    // Decoding is noexcept. An allocation failure must not terminate the UDP loop.
  }
  return result;
}

}  // namespace

DecodeResult decode_pose_packet(const std::uint8_t * data, std::size_t size) noexcept
{
  if (data == nullptr || (size != kLegacyPacketSize && size != kPacketV2Size)) {
    return failure(DecodeStatus::kInvalidSize, "expected a 102-byte v1 or 162-byte v2 packet");
  }

  LittleEndianReader reader(data, size);
  std::array<std::uint8_t, 4> magic{};
  PosePacket packet;
  if (!reader.bytes(magic.data(), magic.size()) ||
    magic != std::array<std::uint8_t, 4>{'P', 'I', 'C', 'O'})
  {
    return failure(DecodeStatus::kInvalidMagic, "invalid PICO packet magic");
  }

  if (!reader.uint8(packet.protocol_version)) {
    return failure(DecodeStatus::kInvalidSize, "truncated protocol version");
  }
  const std::uint8_t expected_version =
    size == kLegacyPacketSize ? kLegacyProtocolVersion : kProtocolVersion;
  if (packet.protocol_version != expected_version) {
    return failure(DecodeStatus::kUnsupportedVersion, "packet size and protocol version disagree");
  }
  if (!reader.uint32(packet.sequence) ||
    !reader.float64(packet.device_timestamp) ||
    !reader.uint8(packet.tracking_flags))
  {
    return failure(DecodeStatus::kInvalidSize, "truncated packet header");
  }
  if (!std::isfinite(packet.device_timestamp)) {
    return failure(DecodeStatus::kNonFinite, "device timestamp is not finite");
  }
  if (!read_pose(reader, packet.head) ||
    !read_pose(reader, packet.left) ||
    !read_pose(reader, packet.right))
  {
    return failure(DecodeStatus::kNonFinite, "pose contains a non-finite or truncated value");
  }
  if (packet.protocol_version >= kProtocolVersion &&
    (!read_controller_input(reader, packet.left_input) ||
    !read_controller_input(reader, packet.right_input)))
  {
    return failure(
      DecodeStatus::kNonFinite, "controller input contains a non-finite or truncated value");
  }
  if (reader.remaining() != 0U) {
    return failure(DecodeStatus::kInvalidSize, "packet has trailing data");
  }

  DecodeResult result;
  result.status = DecodeStatus::kOk;
  result.packet = packet;
  return result;
}

bool is_sequence_newer(
  std::uint32_t sequence, std::optional<std::uint32_t> previous) noexcept
{
  if (!previous.has_value()) {
    return true;
  }
  const std::uint32_t difference = sequence - *previous;
  return difference > 0U && difference < 0x80000000U;
}

std::uint32_t packet_loss_count(
  std::uint32_t sequence, std::optional<std::uint32_t> previous) noexcept
{
  if (!previous.has_value()) {
    return 0U;
  }
  const std::uint32_t difference = sequence - *previous;
  return difference > 1U && difference < 0x80000000U ? difference - 1U : 0U;
}

}  // namespace hc_vr_gateway
