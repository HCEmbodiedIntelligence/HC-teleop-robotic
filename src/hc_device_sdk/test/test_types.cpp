#include <type_traits>

#include <gtest/gtest.h>

#include "hc_device_sdk/device_plugin.hpp"
#include "hc_device_sdk/types.hpp"

namespace
{

TEST(ComponentKind, HasStableManifestNames)
{
  using hc_device_sdk::ComponentKind;
  EXPECT_STREQ(hc_device_sdk::toString(ComponentKind::kArm), "arm");
  EXPECT_STREQ(
    hc_device_sdk::toString(ComponentKind::kDexterousHand), "dexterous_hand");

  ComponentKind parsed = ComponentKind::kUnknown;
  EXPECT_TRUE(hc_device_sdk::componentKindFromString("camera", parsed));
  EXPECT_EQ(parsed, ComponentKind::kCamera);
  EXPECT_FALSE(hc_device_sdk::componentKindFromString("vendor_magic", parsed));
}

TEST(DeviceResult, PreservesSuccessAndFailureDetails)
{
  const auto success = hc_device_sdk::DeviceResult::success("ready");
  EXPECT_TRUE(success);
  EXPECT_EQ(success.message, "ready");

  const auto failure = hc_device_sdk::DeviceResult::failure(
    hc_device_sdk::DeviceError::kCommunication, "feedback timeout");
  EXPECT_FALSE(failure);
  EXPECT_EQ(failure.error, hc_device_sdk::DeviceError::kCommunication);
  EXPECT_EQ(failure.message, "feedback timeout");
}

static_assert(std::is_abstract_v<hc_device_sdk::DevicePlugin>);
static_assert(std::is_abstract_v<hc_device_sdk::JointDevicePlugin>);

}  // namespace
