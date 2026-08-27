#ifndef HC_DEVICE_SDK__MAPPING_HPP_
#define HC_DEVICE_SDK__MAPPING_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "hc_device_sdk/types.hpp"

namespace hc_device_sdk
{

DeviceResult validateJointMappings(const std::vector<JointMapping> & mappings);

double vendorPositionToLogical(const JointMapping & mapping, double value);
double logicalPositionToVendor(const JointMapping & mapping, double value);
double vendorVelocityToLogical(const JointMapping & mapping, double value);
double logicalVelocityToVendor(const JointMapping & mapping, double value);
double vendorAccelerationToLogical(const JointMapping & mapping, double value);
double logicalAccelerationToVendor(const JointMapping & mapping, double value);
double vendorEffortToLogical(const JointMapping & mapping, double value);
double logicalEffortToVendor(const JointMapping & mapping, double value);

// Immutable, validated conversion table. State conversion requires every
// configured mapping but ignores unrelated extra vendor joints. Logical command
// conversion supports safe partial commands and preserves input order.
class JointMappingTable
{
public:
  explicit JointMappingTable(std::vector<JointMapping> mappings);

  const std::vector<JointMapping> & mappings() const noexcept {return mappings_;}

  DeviceResult vendorToLogical(
    const VendorJointState & input, JointState & output) const;
  DeviceResult logicalToVendor(
    const JointCommand & input, VendorJointCommand & output) const;

private:
  std::vector<JointMapping> mappings_;
  std::map<std::string, std::size_t> logical_index_;
  std::map<std::pair<std::string, std::string>, std::size_t> vendor_index_;
  std::map<std::string, std::vector<std::size_t>> vendor_name_indices_;
};

}  // namespace hc_device_sdk

#endif  // HC_DEVICE_SDK__MAPPING_HPP_
