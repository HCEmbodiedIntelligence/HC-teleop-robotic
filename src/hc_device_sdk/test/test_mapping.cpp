#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "hc_device_sdk/mapping.hpp"

namespace
{

using hc_device_sdk::DeviceError;
using hc_device_sdk::JointCommand;
using hc_device_sdk::JointControlMode;
using hc_device_sdk::JointMapping;
using hc_device_sdk::JointMappingTable;
using hc_device_sdk::JointState;
using hc_device_sdk::VendorJointCommand;
using hc_device_sdk::VendorJointState;

std::vector<JointMapping> mappings()
{
  return {
    {"joint_a", "Axis1", "left", 2.0, 0.1},
    {"joint_b", "Axis2", "right", -0.5, -0.2},
  };
}

TEST(MappingValidation, RejectsInvalidAndDuplicateEntries)
{
  auto result = hc_device_sdk::validateJointMappings({});
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, DeviceError::kInvalidConfiguration);

  auto values = mappings();
  values[1].logical_name = values[0].logical_name;
  EXPECT_FALSE(hc_device_sdk::validateJointMappings(values));

  values = mappings();
  values[1].vendor_group = values[0].vendor_group;
  values[1].vendor_name = values[0].vendor_name;
  EXPECT_FALSE(hc_device_sdk::validateJointMappings(values));

  values = mappings();
  values[0].vendor_to_logical_scale = 0.0;
  EXPECT_FALSE(hc_device_sdk::validateJointMappings(values));

  values = mappings();
  values[0].vendor_to_logical_offset_rad =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(hc_device_sdk::validateJointMappings(values));

  EXPECT_THROW(JointMappingTable({}), std::invalid_argument);
}

TEST(MappingConversion, VendorStateBecomesOrderedLogicalSiState)
{
  const JointMappingTable table(mappings());
  VendorJointState vendor;
  // Reverse order and include an unrelated vendor joint. Output still follows
  // the configured logical order.
  vendor.vendor_joint_names = {"Axis2", "Unused", "Axis1"};
  vendor.vendor_groups = {"right", "other", "left"};
  vendor.positions = {1.0, 99.0, 0.5};
  vendor.velocities = {-2.0, 0.0, 0.25};
  vendor.efforts = {-3.0, 0.0, 4.0};

  JointState logical;
  const auto result = table.vendorToLogical(vendor, logical);
  ASSERT_TRUE(result) << result.message;
  EXPECT_EQ(logical.joint_names, (std::vector<std::string>{"joint_a", "joint_b"}));
  ASSERT_EQ(logical.positions_rad.size(), 2U);
  EXPECT_DOUBLE_EQ(logical.positions_rad[0], 1.1);
  EXPECT_DOUBLE_EQ(logical.positions_rad[1], -0.7);
  EXPECT_DOUBLE_EQ(logical.velocities_rad_s[0], 0.5);
  EXPECT_DOUBLE_EQ(logical.velocities_rad_s[1], 1.0);
  EXPECT_DOUBLE_EQ(logical.efforts_nm[0], 2.0);
  EXPECT_DOUBLE_EQ(logical.efforts_nm[1], 6.0);
}

TEST(MappingConversion, LogicalPartialCommandPreservesOrderAndUsesVirtualWork)
{
  const JointMappingTable table(mappings());
  JointCommand logical;
  logical.mode = JointControlMode::kPosition;
  logical.group_name = "two_arms";
  logical.joint_names = {"joint_b", "joint_a"};
  logical.positions_rad = {-0.7, 1.1};
  logical.velocities_rad_s = {1.0, 0.5};
  logical.accelerations_rad_s2 = {2.0, 1.0};
  logical.efforts_nm = {6.0, 2.0};

  VendorJointCommand vendor;
  const auto result = table.logicalToVendor(logical, vendor);
  ASSERT_TRUE(result) << result.message;
  EXPECT_EQ(vendor.vendor_joint_names, (std::vector<std::string>{"Axis2", "Axis1"}));
  EXPECT_EQ(vendor.vendor_groups, (std::vector<std::string>{"right", "left"}));
  EXPECT_EQ(vendor.logical_group_name, "two_arms");
  EXPECT_DOUBLE_EQ(vendor.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(vendor.positions[1], 0.5);
  EXPECT_DOUBLE_EQ(vendor.velocities[0], -2.0);
  EXPECT_DOUBLE_EQ(vendor.velocities[1], 0.25);
  EXPECT_DOUBLE_EQ(vendor.accelerations[0], -4.0);
  EXPECT_DOUBLE_EQ(vendor.accelerations[1], 0.5);
  EXPECT_DOUBLE_EQ(vendor.efforts[0], -3.0);
  EXPECT_DOUBLE_EQ(vendor.efforts[1], 4.0);
}

TEST(MappingConversion, SupportsGroupLessVendorStateOnlyWhenNamesAreUnambiguous)
{
  const JointMappingTable table(mappings());
  VendorJointState vendor;
  vendor.vendor_joint_names = {"Axis1", "Axis2"};
  vendor.positions = {0.5, 1.0};

  JointState logical;
  EXPECT_TRUE(table.vendorToLogical(vendor, logical));

  JointMappingTable ambiguous({
    {"left_joint", "Joint1", "left", 1.0, 0.0},
    {"right_joint", "Joint1", "right", 1.0, 0.0},
  });
  vendor.vendor_joint_names = {"Joint1"};
  vendor.positions = {0.0};
  const auto result = ambiguous.vendorToLogical(vendor, logical);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, DeviceError::kInvalidState);
}

TEST(MappingConversion, RejectsMalformedStateAndCommandsWithoutChangingOutput)
{
  const JointMappingTable table(mappings());
  JointState logical;
  logical.joint_names = {"sentinel"};

  VendorJointState vendor;
  vendor.vendor_joint_names = {"Axis1", "Axis2"};
  vendor.positions = {0.0};
  EXPECT_FALSE(table.vendorToLogical(vendor, logical));
  EXPECT_EQ(logical.joint_names, (std::vector<std::string>{"sentinel"}));

  JointCommand command;
  command.mode = JointControlMode::kVelocity;
  command.joint_names = {"joint_a"};
  command.positions_rad = {0.0};
  VendorJointCommand converted;
  converted.vendor_joint_names = {"sentinel"};
  EXPECT_FALSE(table.logicalToVendor(command, converted));
  EXPECT_EQ(converted.vendor_joint_names, (std::vector<std::string>{"sentinel"}));

  command.velocities_rad_s = {std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(table.logicalToVendor(command, converted));
}

}  // namespace
