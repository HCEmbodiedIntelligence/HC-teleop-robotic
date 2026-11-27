#pragma once

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <filesystem>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hc::v23 {

struct JointSlice {
  std::string name;
  pinocchio::JointIndex joint_id = 0;
  int q_index = 0;
  int v_index = 0;
};

struct TargetTransform {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  pinocchio::SE3 as_se3() const {
    return pinocchio::SE3(rotation, translation);
  }
};

class TargetStore {
 public:
  using Key = std::pair<std::string, std::string>;

  void clear() { transforms_.clear(); }

  void set(const Key& key, const TargetTransform& transform) {
    transforms_[key] = transform;
  }

  void update(const std::map<Key, TargetTransform>& values) {
    for (const auto& [k, v] : values) {
      transforms_[k] = v;
    }
  }

  std::optional<pinocchio::SE3> relative_target(const std::string& root,
                                                const std::string& frame) const;

 private:
  std::map<Key, TargetTransform> transforms_;
};

struct FrameTaskSpec {
  std::string root;
  std::string frame;
  pinocchio::FrameIndex root_id = 0;
  pinocchio::FrameIndex frame_id = 0;
  double position_cost = 1.0;
  double orientation_cost = 1.0;
  double gain = 1.0;
};

struct AxisTaskSpec {
  std::string root;
  std::string frame;
  pinocchio::FrameIndex root_id = 0;
  pinocchio::FrameIndex frame_id = 0;
  int axis_index = 0;  // 0=X, 1=Y, 2=Z
  double cost = 1.0;
  double gain = 1.0;
};

struct JointTaskSpec {
  std::vector<std::string> joint_names;
  std::vector<int> free_columns;  // -1 if joint not in free_joints
  double cost = 1.0;
  double gain = 1.0;
  bool has_target = false;
  std::vector<double> target;
};

struct SolveIKResult {
  Eigen::VectorXd velocity;
  double residual_norm = 0.0;
  std::vector<bool> active_lower;
  std::vector<bool> active_upper;
  int rows = 0;
};

struct RosInterfaceConfig {
  std::string node_name = "controller_v2_3";
  double rate = 400.0;
  std::string sub_joint_state = "/observation/joint_states";
  std::string sub_reset;
  std::string sub_tf_target;
  std::string sub_ee_target;
  std::string sub_control_source = "/control/source";
  std::string pub_joint_target = "/control/solver/joint_targets";
  std::string pub_solver_state;
  std::string pub_reset_ack = "/status/solver/reset_ack";
};

struct PeerTeleopConfig {
  bool legacy_intervention = false;
  std::vector<std::string> arm_joint_names;
  std::unordered_map<std::string, double> gripper_limits;
};

class PinocchioInterfaceV3 {
 public:
  PinocchioInterfaceV3(const std::filesystem::path& urdf_path,
                       const std::vector<std::string>& free_joints);

  int nv_free() const { return static_cast<int>(joints_.size()); }
  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }

  const pinocchio::Model& model() const { return model_; }
  const std::vector<JointSlice>& joints() const { return joints_; }
  const std::vector<std::string>& free_joints() const { return free_joints_; }
  const std::vector<int>& q_indices() const { return q_indices_; }
  const std::vector<int>& v_indices() const { return v_indices_; }

  const Eigen::VectorXd& q() const { return q_; }
  Eigen::VectorXd copy_q() const { return q_; }
  void set_q(const Eigen::VectorXd& q);

  Eigen::VectorXd update_named_joints(const std::vector<std::string>& names,
                                      const std::vector<double>& positions);

  Eigen::VectorXd free_configuration(const Eigen::VectorXd& q) const;
  Eigen::VectorXd free_configuration() const { return free_configuration(q_); }

  void forward(const Eigen::VectorXd& q);
  void forward_with_jacobians(const Eigen::VectorXd& q);

  pinocchio::FrameIndex frame_id(const std::string& name) const;
  pinocchio::SE3 relative_pose(const std::string& root,
                               const std::string& child,
                               const Eigen::VectorXd& q);
  pinocchio::SE3 relative_pose_cached(pinocchio::FrameIndex root_id,
                                      pinocchio::FrameIndex child_id) const;

  Eigen::Matrix<double, 6, Eigen::Dynamic> relative_jacobian(
      const std::string& root,
      const std::string& child,
      const Eigen::VectorXd& q);

  Eigen::Matrix<double, 6, Eigen::Dynamic> relative_jacobian_cached(
      pinocchio::FrameIndex root_id,
      pinocchio::FrameIndex child_id);

  Eigen::VectorXd integrate_free(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& v_free,
                                 double dt) const;

  std::pair<Eigen::VectorXd, Eigen::VectorXd> configuration_bounds_free() const;
  Eigen::VectorXd clip_configuration(const Eigen::VectorXd& q) const;

 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;
  std::vector<std::string> free_joints_;
  std::vector<JointSlice> joints_;
  std::vector<int> q_indices_;
  std::vector<int> v_indices_;
  std::unordered_map<std::string, JointSlice> name_to_joint_;
  Eigen::VectorXd lower_free_;
  Eigen::VectorXd upper_free_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> root_jac_buf_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> child_jac_buf_;
};

Eigen::VectorXd solve_box_lsq(const Eigen::MatrixXd& matrix,
                              const Eigen::VectorXd& target,
                              double damping,
                              const Eigen::VectorXd& lower,
                              const Eigen::VectorXd& upper,
                              int max_iter = 0);

Eigen::Matrix3d quaternion_to_rotation(double x, double y, double z, double w);

class ControllerV23 {
 public:
  explicit ControllerV23(const std::filesystem::path& config_path,
                         std::optional<double> damping_override = std::nullopt);

  void update_joint_state(const std::vector<std::string>& names,
                          const std::vector<double>& positions);
  void update_tf_targets(
      const std::map<TargetStore::Key, TargetTransform>& transforms);
  void reset_to_feedback();

  std::pair<std::vector<std::string>, Eigen::VectorXd> step(
      std::optional<double> dt_override = std::nullopt);

  const RosInterfaceConfig& ros_config() const { return ros_config_; }
  const PeerTeleopConfig& peer_teleop_config() const { return peer_teleop_; }
  const std::vector<std::string>& free_joint_names() const {
    return free_joint_names_;
  }
  const std::vector<FrameTaskSpec>& pose_tasks() const { return pose_tasks_; }
  const std::vector<AxisTaskSpec>& axis_tasks() const { return axis_tasks_; }
  const std::vector<JointTaskSpec>& joint_tasks() const { return joint_tasks_; }

  PinocchioInterfaceV3& interface() { return *interface_; }
  const PinocchioInterfaceV3& interface() const { return *interface_; }

  TargetStore& targets() { return targets_; }
  const TargetStore& targets() const { return targets_; }

  const Eigen::VectorXd& q() const { return q_; }
  Eigen::VectorXd& command_q() { return command_q_; }
  const Eigen::VectorXd& command_q() const { return command_q_; }
  Eigen::VectorXd& previous_velocity() { return previous_velocity_; }
  const Eigen::VectorXd& previous_velocity() const {
    return previous_velocity_;
  }

  bool have_feedback() const { return have_feedback_; }
  double dt() const { return dt_; }
  const std::string& method() const { return method_; }
  void set_method(const std::string& method);
  double damping() const { return damping_; }
  const Eigen::VectorXd& velocity_limits() const { return velocity_limits_; }
  const std::optional<Eigen::VectorXd>& acceleration_limits() const {
    return acceleration_limits_;
  }
  const std::optional<SolveIKResult>& last_result() const {
    return last_result_;
  }
  int total_task_count() const {
    return static_cast<int>(pose_tasks_.size() + axis_tasks_.size() +
                            joint_tasks_.size());
  }

  std::pair<Eigen::MatrixXd, Eigen::VectorXd> stack_tasks(
      const Eigen::VectorXd& q);

 private:
  SolveIKResult solve_ik_internal(const Eigen::VectorXd& base_q, double step_dt);
  std::pair<Eigen::VectorXd, Eigen::VectorXd> compute_velocity_bounds(
      const Eigen::VectorXd& q, double dt) const;
  std::pair<Eigen::VectorXd, Eigen::VectorXd> apply_acceleration_bounds(
      const Eigen::VectorXd& q,
      double dt,
      Eigen::VectorXd lower,
      Eigen::VectorXd upper) const;
  double compute_adaptive_damping(const Eigen::MatrixXd& matrix) const;

  std::filesystem::path config_path_;
  RosInterfaceConfig ros_config_;
  PeerTeleopConfig peer_teleop_;
  std::unique_ptr<PinocchioInterfaceV3> interface_;
  std::vector<std::string> free_joint_names_;
  Eigen::VectorXd q_;
  Eigen::VectorXd command_q_;
  bool have_feedback_ = false;
  TargetStore targets_;
  double dt_ = 0.0025;
  std::string method_ = "control";
  double damping_ = 1e-5;
  std::unordered_map<std::string, double> joints_init_;
  std::vector<FrameTaskSpec> pose_tasks_;
  std::vector<AxisTaskSpec> axis_tasks_;
  std::vector<JointTaskSpec> joint_tasks_;
  Eigen::VectorXd velocity_limits_;
  std::optional<Eigen::VectorXd> acceleration_limits_;
  Eigen::VectorXd previous_velocity_;
  std::optional<SolveIKResult> last_result_;
};

}  // namespace hc::v23

