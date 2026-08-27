#ifndef HC_ADAPTER_OPENARMX__SIM_JOINT_MODEL_HPP_
#define HC_ADAPTER_OPENARMX__SIM_JOINT_MODEL_HPP_

#include <chrono>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace hc_adapter_openarmx
{

using SteadyTime = std::chrono::steady_clock::time_point;

struct JointGroupConfig
{
  std::string group_name;
  std::vector<std::string> joint_names;
  std::vector<double> lower_limits;
  std::vector<double> upper_limits;
  std::vector<double> max_velocity;
  std::string reference_frame;
  std::string tip_frame;
};

class SimJointModel
{
public:
  explicit SimJointModel(std::vector<JointGroupConfig> groups);

  bool acceptCommand(
    const std::string & group_name,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    SteadyTime now,
    SteadyTime valid_until,
    std::string & reason);

  void step(double dt_seconds, SteadyTime now);

  [[nodiscard]] const std::vector<std::string> & jointNames() const;
  [[nodiscard]] const std::vector<double> & positions() const;
  [[nodiscard]] std::vector<double> groupPositions(const std::string & group_name) const;
  [[nodiscard]] bool hasGroup(const std::string & group_name) const;

private:
  struct GroupState
  {
    JointGroupConfig config;
    std::vector<std::size_t> indices;
    std::optional<SteadyTime> valid_until;
  };

  static void validateGroupConfig(const JointGroupConfig & group);
  static double clamp(double value, double lower, double upper);

  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::vector<double> targets_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> max_velocity_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::map<std::string, GroupState> groups_;
};

}  // namespace hc_adapter_openarmx

#endif  // HC_ADAPTER_OPENARMX__SIM_JOINT_MODEL_HPP_
