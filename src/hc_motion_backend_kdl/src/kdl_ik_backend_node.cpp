#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hc_teleop_interfaces/msg/cartesian_target_array.hpp"
#include "hc_teleop_interfaces/msg/joint_command_candidate.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "urdf/model.h"

namespace hc_motion_backend_kdl
{
namespace
{

std::int64_t timeNanoseconds(const builtin_interfaces::msg::Time & value)
{
  return static_cast<std::int64_t>(value.sec) * 1000000000LL + value.nanosec;
}

double positiveFinite(const double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive and finite");
  }
  return value;
}

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

KDL::Frame frameFromPose(const geometry_msgs::msg::Pose & pose)
{
  return KDL::Frame(
    KDL::Rotation::Quaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

struct GroupSolver
{
  std::string group_name;
  std::string reference_frame;
  std::string tip_frame;
  std::vector<std::string> joint_names;
  KDL::JntArray lower;
  KDL::JntArray upper;
  std::shared_ptr<KDL::Chain> chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
  unsigned int max_iterations{50};
  double eps{1e-4};
  double damping{0.02};
  double max_step{0.2};
  double orientation_weight{0.5};
};

bool solveDLS(
  const GroupSolver & solver,
  const KDL::JntArray & seed,
  const KDL::Frame & desired,
  const bool position_only,
  KDL::JntArray & output)
{
  const std::size_t n = solver.joint_names.size();
  Eigen::VectorXd q(n);
  Eigen::VectorXd q_ref(n);
  Eigen::VectorXd q_lower(n);
  Eigen::VectorXd q_upper(n);
  for (std::size_t i = 0; i < n; ++i) {
    q_lower(i) = solver.lower(i);
    q_upper(i) = solver.upper(i);
    q(i) = std::clamp(seed(i), solver.lower(i), solver.upper(i));
    q_ref(i) = q(i);
  }

  Eigen::VectorXd best_q = q;
  double best_error = 1e9;

  KDL::JntArray q_kdl(n);
  KDL::Jacobian jac_kdl(n);

  const double pos_tol = 0.002;  // 2mm
  const double rot_tol = 0.02;   // ~1 deg
  const double rot_weight = position_only ? 0.0 : solver.orientation_weight;
  const int task_dim = position_only ? 3 : 6;

  for (unsigned int iter = 0; iter < solver.max_iterations; ++iter) {
    for (std::size_t i = 0; i < n; ++i) {
      q_kdl(i) = q(i);
    }
    KDL::Frame current;
    if (solver.fk->JntToCart(q_kdl, current) < 0) {
      break;
    }

    const KDL::Vector pos_diff = desired.p - current.p;
    const KDL::Vector rot_diff = KDL::diff(current.M, desired.M);

    const double pos_norm = pos_diff.Norm();
    const double rot_norm = rot_diff.Norm();

    const double total_error = pos_norm + (position_only ? 0.0 : 0.1 * rot_norm);
    if (total_error < best_error) {
      best_error = total_error;
      best_q = q;
    }

    if (pos_norm <= pos_tol && (position_only || rot_norm <= rot_tol)) {
      best_q = q;
      break;
    }

    if (solver.jac_solver->JntToJac(q_kdl, jac_kdl) < 0) {
      break;
    }

    Eigen::VectorXd error(task_dim);
    error(0) = pos_diff.x();
    error(1) = pos_diff.y();
    error(2) = pos_diff.z();
    if (!position_only) {
      error(3) = rot_diff.x() * rot_weight;
      error(4) = rot_diff.y() * rot_weight;
      error(5) = rot_diff.z() * rot_weight;
    }

    Eigen::MatrixXd J(task_dim, n);
    for (int r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < n; ++c) {
        J(r, c) = jac_kdl(r, c);
      }
    }
    if (!position_only) {
      for (int r = 3; r < 6; ++r) {
        for (std::size_t c = 0; c < n; ++c) {
          J(r, c) = jac_kdl(r, c) * rot_weight;
        }
      }
    }

    // Damped Least Squares: delta_q = J^T (J J^T + lambda^2 I)^(-1) e
    const double lambda = solver.damping;
    Eigen::MatrixXd A = J * J.transpose() + (lambda * lambda) * Eigen::MatrixXd::Identity(task_dim, task_dim);
    Eigen::VectorXd solved_error = A.ldlt().solve(error);
    Eigen::VectorXd delta_q_task = J.transpose() * solved_error;

    // Nullspace projection for joint limit avoidance and posture preservation
    Eigen::MatrixXd pinv = J.transpose() * A.ldlt().solve(Eigen::MatrixXd::Identity(task_dim, task_dim));
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - pinv * J;

    Eigen::VectorXd h_avoid = Eigen::VectorXd::Zero(n);
    const double margin_fraction = 0.15;
    for (std::size_t i = 0; i < n; ++i) {
      const double span = q_upper(i) - q_lower(i);
      if (span > 1e-4) {
        const double norm_low = (q(i) - q_lower(i)) / span;
        const double norm_high = (q_upper(i) - q(i)) / span;
        if (norm_low < margin_fraction) {
          const double diff = 1.0 - norm_low / margin_fraction;
          h_avoid(i) += diff * diff;
        }
        if (norm_high < margin_fraction) {
          const double diff = 1.0 - norm_high / margin_fraction;
          h_avoid(i) -= diff * diff;
        }
      }
    }

    Eigen::VectorXd h_posture = q_ref - q;
    Eigen::VectorXd h = 0.2 * h_avoid + 0.05 * h_posture;
    Eigen::VectorXd delta_q = delta_q_task + N * h;

    const double delta_norm = delta_q.norm();
    if (delta_norm > solver.max_step) {
      delta_q *= (solver.max_step / delta_norm);
    }

    q += delta_q;
    for (std::size_t i = 0; i < n; ++i) {
      q(i) = std::clamp(q(i), q_lower(i), q_upper(i));
    }
  }

  output.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    output(i) = best_q(i);
  }
  return true;
}

}  // namespace

class KdlIkBackendNode final : public rclcpp::Node
{
public:
  explicit KdlIkBackendNode(const rclcpp::NodeOptions & options)
  : Node("kdl_ik_backend", options)
  {
    const auto urdf_path = declare_parameter<std::string>("urdf_path", "");
    if (urdf_path.empty()) {
      throw std::invalid_argument("urdf_path must be supplied");
    }
    const auto group_names = declare_parameter<std::vector<std::string>>(
      "group_names", std::vector<std::string>{});
    if (group_names.empty()) {
      throw std::invalid_argument("group_names must include at least one arm group");
    }
    feedback_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        positiveFinite(declare_parameter<double>("feedback_timeout_sec", 0.2),
        "feedback_timeout_sec")));
    const auto max_iterations = static_cast<unsigned int>(
      positiveFinite(declare_parameter<double>("ik_max_iterations", 50.0),
      "ik_max_iterations"));
    const auto eps = positiveFinite(declare_parameter<double>("ik_eps", 1e-4), "ik_eps");

    urdf::Model robot_model;
    if (!robot_model.initFile(urdf_path)) {
      throw std::runtime_error("unable to parse URDF: " + urdf_path);
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
      throw std::runtime_error("unable to build KDL tree from URDF: " + urdf_path);
    }
    for (const auto & group_name : group_names) {
      loadGroup(group_name, robot_model, tree, max_iterations, eps);
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    target_subscription_ =
      create_subscription<hc_teleop_interfaces::msg::CartesianTargetArray>(
      declare_parameter<std::string>("target_topic", "motion/backend/cartesian_targets"), qos,
      [this](hc_teleop_interfaces::msg::CartesianTargetArray::ConstSharedPtr message) {
        onTarget(*message);
      });
    state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "state/joints"), qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        onJointState(*message);
      });
    candidate_publisher_ =
      create_publisher<hc_teleop_interfaces::msg::JointCommandCandidate>(
      declare_parameter<std::string>("candidate_topic", "motion/backend/joint_candidate"), qos);

    RCLCPP_INFO(get_logger(), "KDL IK backend ready: groups=%zu urdf=%s",
      solvers_.size(), urdf_path.c_str());
  }

private:
  void loadGroup(
    const std::string & group_name,
    const urdf::Model & robot_model,
    const KDL::Tree & tree,
    const unsigned int max_iterations,
    const double eps)
  {
    if (solvers_.find(group_name) != solvers_.end()) {
      throw std::invalid_argument("duplicate IK group: " + group_name);
    }
    GroupSolver solver;
    solver.group_name = group_name;
    solver.reference_frame = declare_parameter<std::string>(group_name + ".base_frame", "");
    solver.tip_frame = declare_parameter<std::string>(group_name + ".tip_frame", "");
    solver.joint_names = declare_parameter<std::vector<std::string>>(
      group_name + ".joint_names", std::vector<std::string>{});
    if (solver.reference_frame.empty() || solver.tip_frame.empty() || solver.joint_names.empty()) {
      throw std::invalid_argument(group_name + " requires joint_names/base_frame/tip_frame");
    }

    solver.lower = KDL::JntArray(solver.joint_names.size());
    solver.upper = KDL::JntArray(solver.joint_names.size());
    for (std::size_t index = 0; index < solver.joint_names.size(); ++index) {
      const auto joint = robot_model.getJoint(solver.joint_names[index]);
      if (!joint || !joint->limits) {
        throw std::invalid_argument("URDF has no limits for joint: " + solver.joint_names[index]);
      }
      solver.lower(index) = joint->limits->lower;
      solver.upper(index) = joint->limits->upper;
    }

    solver.chain = std::make_shared<KDL::Chain>();
    if (!tree.getChain(solver.reference_frame, solver.tip_frame, *solver.chain)) {
      throw std::invalid_argument(
              "unable to build KDL chain from " + solver.reference_frame + " to " +
              solver.tip_frame);
    }
    if (solver.chain->getNrOfJoints() != solver.joint_names.size()) {
      throw std::invalid_argument(
              group_name + " KDL joint count does not match configured joint_names");
    }
    solver.fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(*solver.chain);
    solver.jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(*solver.chain);
    solver.max_iterations = max_iterations;
    solver.eps = eps;
    solvers_.emplace(group_name, std::move(solver));
  }

  void onJointState(const sensor_msgs::msg::JointState & message)
  {
    const auto count = std::min(message.name.size(), message.position.size());
    for (std::size_t index = 0; index < count; ++index) {
      if (std::isfinite(message.position[index])) {
        current_positions_[message.name[index]] = message.position[index];
      }
    }
    last_feedback_ = std::chrono::steady_clock::now();
  }

  void onTarget(const hc_teleop_interfaces::msg::CartesianTargetArray & message)
  {
    const auto now_ros = now();
    if (timeNanoseconds(message.valid_until) <= now_ros.nanoseconds()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ignored expired Cartesian target");
      return;
    }
    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady - last_feedback_ > feedback_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ignored Cartesian target: joint feedback is stale");
      return;
    }

    for (const auto & target : message.targets) {
      const auto solver_it = solvers_.find(target.group_name);
      if (solver_it == solvers_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "ignored target for unknown group %s",
          target.group_name.c_str());
        continue;
      }
      const auto & solver = solver_it->second;
      if (target.reference_frame != solver.reference_frame || target.tip_frame != solver.tip_frame) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "ignored target for %s: frame mismatch",
          target.group_name.c_str());
        continue;
      }
      solveAndPublish(message, target, solver);
    }
  }

  void solveAndPublish(
    const hc_teleop_interfaces::msg::CartesianTargetArray & envelope,
    const hc_teleop_interfaces::msg::CartesianTarget & target,
    const GroupSolver & solver)
  {
    if (!finitePose(target.pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ignored target for %s: pose is not finite",
        solver.group_name.c_str());
      return;
    }

    KDL::JntArray seed(solver.joint_names.size());
    for (std::size_t index = 0; index < solver.joint_names.size(); ++index) {
      const auto value = current_positions_.find(solver.joint_names[index]);
      if (value == current_positions_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "ignored target for %s: missing joint feedback",
          solver.group_name.c_str());
        return;
      }
      seed(index) = std::min(std::max(value->second, solver.lower(index)), solver.upper(index));
    }

    KDL::Frame desired = frameFromPose(target.pose);
    const bool position_only =
      target.pose_mode == hc_teleop_interfaces::msg::CartesianTarget::POSITION_ONLY;
    if (position_only) {
      KDL::Frame current;
      if (solver.fk->JntToCart(seed, current) >= 0) {
        desired.M = current.M;
      }
    }

    KDL::JntArray output(solver.joint_names.size());
    if (!solveDLS(solver, seed, desired, position_only, output)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "DLS IK failed for %s",
        solver.group_name.c_str());
      return;
    }

    hc_teleop_interfaces::msg::JointCommandCandidate candidate;
    candidate.header.stamp = now();
    candidate.source_id = envelope.source_id;
    candidate.session_id = envelope.session_id;
    candidate.sequence = envelope.sequence;
    candidate.valid_until = envelope.valid_until;
    candidate.control_mode =
      hc_teleop_interfaces::msg::JointCommandCandidate::CONTROL_MODE_POSITION;
    candidate.group_name = solver.group_name;
    candidate.command.header = candidate.header;
    candidate.command.name = solver.joint_names;
    candidate.command.position.resize(solver.joint_names.size());
    for (std::size_t index = 0; index < solver.joint_names.size(); ++index) {
      candidate.command.position[index] = output(index);
    }
    candidate_publisher_->publish(std::move(candidate));
    ++published_candidates_;
  }

  std::map<std::string, GroupSolver> solvers_;
  std::unordered_map<std::string, double> current_positions_;
  std::chrono::steady_clock::duration feedback_timeout_{std::chrono::milliseconds(200)};
  std::chrono::steady_clock::time_point last_feedback_{};
  rclcpp::Subscription<hc_teleop_interfaces::msg::CartesianTargetArray>::SharedPtr
    target_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Publisher<hc_teleop_interfaces::msg::JointCommandCandidate>::SharedPtr
    candidate_publisher_;
  std::uint64_t published_candidates_{0U};
};

}  // namespace hc_motion_backend_kdl

RCLCPP_COMPONENTS_REGISTER_NODE(hc_motion_backend_kdl::KdlIkBackendNode)
