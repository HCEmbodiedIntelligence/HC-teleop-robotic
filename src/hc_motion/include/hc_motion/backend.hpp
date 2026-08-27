#ifndef HC_MOTION__BACKEND_HPP_
#define HC_MOTION__BACKEND_HPP_

#include <memory>
#include <string>

#include "hc_motion/types.hpp"

namespace hc_motion
{

struct InverseKinematicsRequest
{
  std::string group_name;
  std::string base_link;
  std::string tip_link;
  Pose target;
  JointTarget seed;
};

struct InverseKinematicsResult
{
  MotionStatus status;
  JointTarget solution;
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

struct ForwardKinematicsRequest
{
  std::string group_name;
  std::string base_link;
  std::string tip_link;
  JointTarget joints;
};

struct ForwardKinematicsResult
{
  MotionStatus status;
  Pose pose;
};

/// Optional same-build optimization boundary. Independently released backends
/// use the ROS process contract documented in backend_contract.md instead of
/// relying on this C++ ABI.
class KinematicsBackend
{
public:
  virtual ~KinematicsBackend() = default;
  [[nodiscard]] virtual std::string name() const = 0;
  virtual InverseKinematicsResult inverse(const InverseKinematicsRequest & request) = 0;
  virtual ForwardKinematicsResult forward(const ForwardKinematicsRequest & request) = 0;
};

using KinematicsBackendPtr = std::shared_ptr<KinematicsBackend>;

}  // namespace hc_motion

#endif  // HC_MOTION__BACKEND_HPP_
