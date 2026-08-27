#include "hc_vr_gateway/codec.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <vector>

namespace
{

template<typename Unsigned>
void append_unsigned(std::vector<std::uint8_t> & bytes, Unsigned value)
{
  for (std::size_t index = 0; index < sizeof(Unsigned); ++index) {
    bytes.push_back(static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU));
  }
}

void append_float(std::vector<std::uint8_t> & bytes, float value)
{
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(value));
  append_unsigned(bytes, bits);
}

void append_double(std::vector<std::uint8_t> & bytes, double value)
{
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(value));
  append_unsigned(bytes, bits);
}

void append_input(
  std::vector<std::uint8_t> & bytes,
  std::uint16_t held, std::uint16_t pressed, std::uint16_t released,
  float base)
{
  append_unsigned(bytes, held);
  append_unsigned(bytes, pressed);
  append_unsigned(bytes, released);
  for (int index = 0; index < 6; ++index) {
    append_float(bytes, base + static_cast<float>(index) * 0.1F);
  }
}

std::vector<std::uint8_t> make_packet(std::uint8_t version)
{
  std::vector<std::uint8_t> bytes{'P', 'I', 'C', 'O'};
  bytes.push_back(version);
  append_unsigned<std::uint32_t>(bytes, 42U);
  append_double(bytes, 1234.25);
  bytes.push_back(
    hc_vr_gateway::kHeadTrackedFlag |
    hc_vr_gateway::kLeftTrackedFlag |
    hc_vr_gateway::kRightTrackedFlag);
  for (int value = 1; value <= 21; ++value) {
    append_float(bytes, static_cast<float>(value));
  }
  if (version == hc_vr_gateway::kProtocolVersion) {
    append_input(bytes, 1U, 2U, 4U, 0.1F);
    append_input(bytes, 8U, 16U, 32U, 1.1F);
  }
  return bytes;
}

void overwrite_float(
  std::vector<std::uint8_t> & bytes, std::size_t offset, float value)
{
  std::vector<std::uint8_t> encoded;
  append_float(encoded, value);
  ASSERT_LE(offset + encoded.size(), bytes.size());
  std::copy(encoded.begin(), encoded.end(), bytes.begin() + offset);
}

}  // namespace

TEST(Codec, DecodesLegacyV1Packet)
{
  const auto bytes = make_packet(hc_vr_gateway::kLegacyProtocolVersion);
  ASSERT_EQ(bytes.size(), hc_vr_gateway::kLegacyPacketSize);

  const auto result = hc_vr_gateway::decode_pose_packet(bytes.data(), bytes.size());
  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.packet.protocol_version, 1U);
  EXPECT_EQ(result.packet.sequence, 42U);
  EXPECT_DOUBLE_EQ(result.packet.device_timestamp, 1234.25);
  EXPECT_TRUE(result.packet.tracked(hc_vr_gateway::kHeadTrackedFlag));
  EXPECT_TRUE(result.packet.tracked(hc_vr_gateway::kLeftTrackedFlag));
  EXPECT_TRUE(result.packet.tracked(hc_vr_gateway::kRightTrackedFlag));
  EXPECT_FLOAT_EQ(result.packet.head.position[0], 1.0F);
  EXPECT_FLOAT_EQ(result.packet.head.quaternion[3], 7.0F);
  EXPECT_FLOAT_EQ(result.packet.left.position[0], 8.0F);
  EXPECT_FLOAT_EQ(result.packet.right.quaternion[3], 21.0F);
  EXPECT_EQ(result.packet.left_input.held_mask, 0U);
  EXPECT_FLOAT_EQ(result.packet.right_input.trigger, 0.0F);
}

TEST(Codec, DecodesV2ControllerInputs)
{
  const auto bytes = make_packet(hc_vr_gateway::kProtocolVersion);
  ASSERT_EQ(bytes.size(), hc_vr_gateway::kPacketV2Size);

  const auto result = hc_vr_gateway::decode_pose_packet(bytes.data(), bytes.size());
  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.packet.protocol_version, 2U);
  EXPECT_EQ(result.packet.left_input.held_mask, 1U);
  EXPECT_EQ(result.packet.left_input.pressed_mask, 2U);
  EXPECT_EQ(result.packet.left_input.released_mask, 4U);
  EXPECT_FLOAT_EQ(result.packet.left_input.trigger, 0.1F);
  EXPECT_FLOAT_EQ(result.packet.left_input.grip, 0.2F);
  EXPECT_FLOAT_EQ(result.packet.left_input.primary_axis[0], 0.3F);
  EXPECT_FLOAT_EQ(result.packet.left_input.secondary_axis[1], 0.6F);
  EXPECT_EQ(result.packet.right_input.held_mask, 8U);
  EXPECT_EQ(result.packet.right_input.pressed_mask, 16U);
  EXPECT_EQ(result.packet.right_input.released_mask, 32U);
  EXPECT_FLOAT_EQ(result.packet.right_input.trigger, 1.1F);
  EXPECT_FLOAT_EQ(result.packet.right_input.secondary_axis[1], 1.6F);
}

TEST(Codec, RejectsInvalidPackets)
{
  auto truncated = make_packet(hc_vr_gateway::kLegacyProtocolVersion);
  truncated.pop_back();
  EXPECT_EQ(
    hc_vr_gateway::decode_pose_packet(truncated.data(), truncated.size()).status,
    hc_vr_gateway::DecodeStatus::kInvalidSize);

  auto bad_magic = make_packet(hc_vr_gateway::kLegacyProtocolVersion);
  bad_magic[0] = 'X';
  EXPECT_EQ(
    hc_vr_gateway::decode_pose_packet(bad_magic.data(), bad_magic.size()).status,
    hc_vr_gateway::DecodeStatus::kInvalidMagic);

  auto wrong_version = make_packet(hc_vr_gateway::kLegacyProtocolVersion);
  wrong_version[4] = hc_vr_gateway::kProtocolVersion;
  EXPECT_EQ(
    hc_vr_gateway::decode_pose_packet(wrong_version.data(), wrong_version.size()).status,
    hc_vr_gateway::DecodeStatus::kUnsupportedVersion);

  auto not_finite = make_packet(hc_vr_gateway::kProtocolVersion);
  constexpr std::size_t first_pose_float_offset = 4U + 1U + 4U + 8U + 1U;
  overwrite_float(
    not_finite, first_pose_float_offset, std::numeric_limits<float>::quiet_NaN());
  EXPECT_EQ(
    hc_vr_gateway::decode_pose_packet(not_finite.data(), not_finite.size()).status,
    hc_vr_gateway::DecodeStatus::kNonFinite);

  EXPECT_EQ(
    hc_vr_gateway::decode_pose_packet(nullptr, 0U).status,
    hc_vr_gateway::DecodeStatus::kInvalidSize);
}

TEST(Sequence, RejectsDuplicatesAndOldPackets)
{
  EXPECT_TRUE(hc_vr_gateway::is_sequence_newer(10U, std::nullopt));
  EXPECT_TRUE(hc_vr_gateway::is_sequence_newer(11U, 10U));
  EXPECT_FALSE(hc_vr_gateway::is_sequence_newer(10U, 10U));
  EXPECT_FALSE(hc_vr_gateway::is_sequence_newer(9U, 10U));
  EXPECT_EQ(hc_vr_gateway::packet_loss_count(14U, 10U), 3U);
  EXPECT_EQ(hc_vr_gateway::packet_loss_count(10U, 10U), 0U);
}

TEST(Sequence, HandlesUint32Wraparound)
{
  constexpr std::uint32_t almost_wrapped = 0xfffffffeU;
  EXPECT_TRUE(hc_vr_gateway::is_sequence_newer(0xffffffffU, almost_wrapped));
  EXPECT_TRUE(hc_vr_gateway::is_sequence_newer(0U, 0xffffffffU));
  EXPECT_TRUE(hc_vr_gateway::is_sequence_newer(1U, almost_wrapped));
  EXPECT_EQ(hc_vr_gateway::packet_loss_count(1U, almost_wrapped), 2U);
  EXPECT_FALSE(hc_vr_gateway::is_sequence_newer(almost_wrapped, 1U));
  EXPECT_FALSE(hc_vr_gateway::is_sequence_newer(0x80000001U, 1U));
}
