#include "v23_ik_core.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/skew.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace hc::v23 {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;

std::string to_upper(std::string s) {
  for (char& ch : s) {
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }
  return s;
}

Eigen::VectorXd solve_linear_spd_or_lsq(const Eigen::MatrixXd& a,
                                        const Eigen::VectorXd& b) {
  Eigen::LDLT<Eigen::MatrixXd> ldlt(a);
  if (ldlt.info() == Eigen::Success) {
    Eigen::VectorXd x = ldlt.solve(b);
    if (x.allFinite() && (a * x - b).norm() <= 1e-6 * (b.norm() + 1.0)) {
      return x;
    }
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
  if (qr.isInvertible()) {
    Eigen::VectorXd x = qr.solve(b);
    if (x.allFinite()) {
      return x;
    }
  }
  return a.completeOrthogonalDecomposition().solve(b);
}

Eigen::MatrixXd invert_spd_or_lsq(const Eigen::MatrixXd& a) {
  const Eigen::Index n = a.rows();
  const Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
  Eigen::LDLT<Eigen::MatrixXd> ldlt(a);
  if (ldlt.info() == Eigen::Success) {
    Eigen::MatrixXd inv = ldlt.solve(eye);
    if (inv.allFinite()) {
      return inv;
    }
  }
  return a.completeOrthogonalDecomposition().solve(eye);
}

}  // namespace

Eigen::Matrix3d quaternion_to_rotation(double x, double y, double z, double w) {
  const double norm_sq = w * w + x * x + y * y + z * z;
  if (!std::isfinite(norm_sq)) {
    throw std::invalid_argument("target quaternion is non-finite");
  }
  const double norm = std::sqrt(norm_sq);
  if (norm < 1e-12) {
    throw std::invalid_argument("target quaternion has zero length");
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  Eigen::Matrix3d r;
  r << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
      2.0 * (x * z + y * w), 2.0 * (x * y + z * w),
      1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
      2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
      1.0 - 2.0 * (x * x + y * y);
  return r;
}

std::optional<pinocchio::SE3> TargetStore::relative_target(
    const std::string& root, const std::string& frame) const {
  auto direct = transforms_.find({root, frame});
  if (direct != transforms_.end()) {
    return direct->second.as_se3();
  }
  auto find_global = [this](const std::string& name) -> const TargetTransform* {
    auto it = transforms_.find({"", name});
    if (it != transforms_.end()) {
      return &it->second;
    }
    it = transforms_.find({"world", name});
    if (it != transforms_.end()) {
      return &it->second;
    }
    return nullptr;
  };
  const TargetTransform* root_tf = find_global(root);
  const TargetTransform* child_tf = find_global(frame);
  if (root_tf == nullptr || child_tf == nullptr) {
    return std::nullopt;
  }
  return root_tf->as_se3().actInv(child_tf->as_se3());
}

PinocchioInterfaceV3::PinocchioInterfaceV3(
    const std::filesystem::path& urdf_path,
    const std::vector<std::string>& free_joints)
    : free_joints_(free_joints) {
  const auto resolved = std::filesystem::weakly_canonical(urdf_path);
  pinocchio::urdf::buildModel(resolved.string(), model_);
  data_ = pinocchio::Data(model_);
  q_ = pinocchio::neutral(model_);

  joints_.reserve(free_joints_.size());
  q_indices_.reserve(free_joints_.size());
  v_indices_.reserve(free_joints_.size());

  for (const auto& name : free_joints_) {
    const pinocchio::JointIndex joint_id = model_.getJointId(name);
    if (joint_id == 0 || joint_id >= model_.joints.size()) {
      throw std::invalid_argument("joint not found in URDF: " + name);
    }
    const auto& joint = model_.joints[joint_id];
    if (joint.nq() != 1 || joint.nv() != 1) {
      throw std::invalid_argument("expected scalar joint " + name);
    }
    JointSlice slice{name, joint_id, joint.idx_q(), joint.idx_v()};
    joints_.push_back(slice);
    q_indices_.push_back(slice.q_index);
    v_indices_.push_back(slice.v_index);
    name_to_joint_.emplace(name, slice);
  }

  const int n = nv_free();
  lower_free_.resize(n);
  upper_free_.resize(n);
  for (int i = 0; i < n; ++i) {
    lower_free_[i] = model_.lowerPositionLimit[q_indices_[i]];
    upper_free_[i] = model_.upperPositionLimit[q_indices_[i]];
  }
  root_jac_buf_.resize(6, model_.nv);
  child_jac_buf_.resize(6, model_.nv);
  forward(q_);
}

void PinocchioInterfaceV3::set_q(const Eigen::VectorXd& q) {
  if (q.size() != model_.nq) {
    throw std::invalid_argument("q size mismatch");
  }
  q_ = q;
  forward(q_);
}

Eigen::VectorXd PinocchioInterfaceV3::update_named_joints(
    const std::vector<std::string>& names,
    const std::vector<double>& positions) {
  if (names.size() != positions.size()) {
    throw std::invalid_argument("joint name/position size mismatch");
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto it = name_to_joint_.find(names[i]);
    if (it != name_to_joint_.end()) {
      const double value = positions[i];
      if (!std::isfinite(value)) {
        throw std::invalid_argument("non-finite feedback for " + names[i]);
      }
      q_[it->second.q_index] = value;
    }
  }
  forward(q_);
  return q_;
}

Eigen::VectorXd PinocchioInterfaceV3::free_configuration(
    const Eigen::VectorXd& q) const {
  Eigen::VectorXd out(nv_free());
  for (int i = 0; i < nv_free(); ++i) {
    out[i] = q[q_indices_[i]];
  }
  return out;
}

void PinocchioInterfaceV3::forward(const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

void PinocchioInterfaceV3::forward_with_jacobians(const Eigen::VectorXd& q) {
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

pinocchio::FrameIndex PinocchioInterfaceV3::frame_id(
    const std::string& name) const {
  const pinocchio::FrameIndex id = model_.getFrameId(name);
  if (id >= model_.frames.size()) {
    throw std::invalid_argument("frame not found in URDF: " + name);
  }
  return id;
}

pinocchio::SE3 PinocchioInterfaceV3::relative_pose(const std::string& root,
                                                   const std::string& child,
                                                   const Eigen::VectorXd& q) {
  forward(q);
  return relative_pose_cached(frame_id(root), frame_id(child));
}

pinocchio::SE3 PinocchioInterfaceV3::relative_pose_cached(
    pinocchio::FrameIndex root_id, pinocchio::FrameIndex child_id) const {
  const pinocchio::SE3& world_root = data_.oMf[root_id];
  const pinocchio::SE3& world_child = data_.oMf[child_id];
  return world_root.actInv(world_child);
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioInterfaceV3::relative_jacobian(const std::string& root,
                                        const std::string& child,
                                        const Eigen::VectorXd& q) {
  forward_with_jacobians(q);
  return relative_jacobian_cached(frame_id(root), frame_id(child));
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioInterfaceV3::relative_jacobian_cached(pinocchio::FrameIndex root_id,
                                               pinocchio::FrameIndex child_id) {
  const pinocchio::SE3& world_root = data_.oMf[root_id];
  const pinocchio::SE3 root_child = world_root.actInv(data_.oMf[child_id]);

  root_jac_buf_.setZero();
  child_jac_buf_.setZero();
  pinocchio::getFrameJacobian(model_, data_, root_id,
                              pinocchio::LOCAL_WORLD_ALIGNED, root_jac_buf_);
  pinocchio::getFrameJacobian(model_, data_, child_id,
                              pinocchio::LOCAL_WORLD_ALIGNED, child_jac_buf_);

  const Eigen::Matrix3d world_to_root = world_root.rotation().transpose();
  const Eigen::Matrix3d skew_rc = pinocchio::skew(root_child.translation());

  const int n = nv_free();
  Eigen::Matrix<double, 6, Eigen::Dynamic> result(6, n);
  for (int col = 0; col < n; ++col) {
    const int v_idx = v_indices_[col];
    const Eigen::Vector3d dlin =
        child_jac_buf_.block<3, 1>(0, v_idx) -
        root_jac_buf_.block<3, 1>(0, v_idx);
    const Eigen::Vector3d dang =
        child_jac_buf_.block<3, 1>(3, v_idx) -
        root_jac_buf_.block<3, 1>(3, v_idx);
    const Eigen::Vector3d root_ang =
        world_to_root * root_jac_buf_.block<3, 1>(3, v_idx);
    result.block<3, 1>(0, col) = world_to_root * dlin + skew_rc * root_ang;
    result.block<3, 1>(3, col) = world_to_root * dang;
  }
  return result;
}

Eigen::VectorXd PinocchioInterfaceV3::integrate_free(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v_free, double dt) const {
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(model_.nv);
  for (int i = 0; i < nv_free(); ++i) {
    velocity[v_indices_[i]] = v_free[i] * dt;
  }
  return pinocchio::integrate(model_, q, velocity);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
PinocchioInterfaceV3::configuration_bounds_free() const {
  return {lower_free_, upper_free_};
}

Eigen::VectorXd PinocchioInterfaceV3::clip_configuration(
    const Eigen::VectorXd& q) const {
  Eigen::VectorXd out = q;
  for (int i = 0; i < nv_free(); ++i) {
    double v = out[q_indices_[i]];
    if (std::isfinite(lower_free_[i]) && v < lower_free_[i]) {
      v = lower_free_[i];
    }
    if (std::isfinite(upper_free_[i]) && v > upper_free_[i]) {
      v = upper_free_[i];
    }
    out[q_indices_[i]] = v;
  }
  return out;
}

Eigen::VectorXd solve_box_lsq(const Eigen::MatrixXd& matrix,
                              const Eigen::VectorXd& target,
                              double damping,
                              const Eigen::VectorXd& lower,
                              const Eigen::VectorXd& upper,
                              int max_iter) {
  const int count = static_cast<int>(matrix.cols());
  Eigen::MatrixXd hessian =
      matrix.transpose() * matrix +
      damping * Eigen::MatrixXd::Identity(count, count);
  const Eigen::VectorXd grad_target = matrix.transpose() * target;

  std::vector<int8_t> active(count, 0);  // -1 lower, +1 upper
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(count);
  const int iters = (max_iter > 0) ? max_iter : (6 * count + 20);
  constexpr double kTol = 1e-10;
  constexpr double kNegInf = -std::numeric_limits<double>::infinity();

  std::vector<int> free_idx;
  std::vector<int> fixed_idx;
  free_idx.reserve(count);
  fixed_idx.reserve(count);

  for (int iter = 0; iter < iters; ++iter) {
    free_idx.clear();
    fixed_idx.clear();
    for (int i = 0; i < count; ++i) {
      if (active[i] == 0) {
        free_idx.push_back(i);
      } else {
        fixed_idx.push_back(i);
      }
    }

    if (!free_idx.empty()) {
      const int nf = static_cast<int>(free_idx.size());
      Eigen::MatrixXd block(nf, nf);
      Eigen::VectorXd rhs(nf);
      for (int r = 0; r < nf; ++r) {
        const int ri = free_idx[r];
        double rval = grad_target[ri];
        for (int fi : fixed_idx) {
          rval -= hessian(ri, fi) * velocity[fi];
        }
        rhs[r] = rval;
        for (int c = 0; c < nf; ++c) {
          block(r, c) = hessian(ri, free_idx[c]);
        }
      }
      const Eigen::VectorXd sol = solve_linear_spd_or_lsq(block, rhs);
      for (int r = 0; r < nf; ++r) {
        velocity[free_idx[r]] = sol[r];
      }
    }

    double max_low_viol = kNegInf;
    int low_index = 0;
    double max_high_viol = kNegInf;
    int high_index = 0;
    for (int i = 0; i < count; ++i) {
      if (active[i] == 0) {
        const double lv = lower[i] - velocity[i];
        const double hv = velocity[i] - upper[i];
        if (lv > max_low_viol) {
          max_low_viol = lv;
          low_index = i;
        }
        if (hv > max_high_viol) {
          max_high_viol = hv;
          high_index = i;
        }
      }
    }

    if (max_low_viol > kTol || max_high_viol > kTol) {
      if (max_low_viol >= max_high_viol) {
        active[low_index] = -1;
        velocity[low_index] = lower[low_index];
      } else {
        active[high_index] = 1;
        velocity[high_index] = upper[high_index];
      }
      continue;
    }

    const Eigen::VectorXd gradient = hessian * velocity - grad_target;
    double max_rel_low = kNegInf;
    int rel_low_idx = 0;
    double max_rel_high = kNegInf;
    int rel_high_idx = 0;
    for (int i = 0; i < count; ++i) {
      if (active[i] == -1) {
        const double val = -gradient[i];
        if (val > max_rel_low) {
          max_rel_low = val;
          rel_low_idx = i;
        }
      } else if (active[i] == 1) {
        const double val = gradient[i];
        if (val > max_rel_high) {
          max_rel_high = val;
          rel_high_idx = i;
        }
      }
    }

    if (max_rel_low > kTol || max_rel_high > kTol) {
      const int release =
          (max_rel_low >= max_rel_high) ? rel_low_idx : rel_high_idx;
      active[release] = 0;
      continue;
    }
    break;
  }

  for (int i = 0; i < count; ++i) {
    if (velocity[i] < lower[i]) velocity[i] = lower[i];
    if (velocity[i] > upper[i]) velocity[i] = upper[i];
  }
  return velocity;
}

ControllerV23::ControllerV23(const std::filesystem::path& config_path,
                             std::optional<double> damping_override)
    : config_path_(std::filesystem::weakly_canonical(config_path)) {
  const YAML::Node cfg = YAML::LoadFile(config_path_.string());

  // Parse ros_interface section
  if (const YAML::Node ros = cfg["ros_interface"]; ros && ros.IsMap()) {
    if (ros["node_name"]) {
      ros_config_.node_name = ros["node_name"].as<std::string>();
    }
    if (ros["rate"]) {
      ros_config_.rate = ros["rate"].as<double>();
    }
    if (const YAML::Node sub = ros["sub_topic"]; sub && sub.IsMap()) {
      if (sub["joint_state"] && !sub["joint_state"].IsNull()) {
        ros_config_.sub_joint_state = sub["joint_state"].as<std::string>();
      }
      if (sub["reset"] && !sub["reset"].IsNull()) {
        ros_config_.sub_reset = sub["reset"].as<std::string>();
      }
      if (sub["tf_target"] && !sub["tf_target"].IsNull()) {
        ros_config_.sub_tf_target = sub["tf_target"].as<std::string>();
      }
      if (sub["ee_target"] && !sub["ee_target"].IsNull()) {
        ros_config_.sub_ee_target = sub["ee_target"].as<std::string>();
      }
      if (sub["control_source"] && !sub["control_source"].IsNull()) {
        ros_config_.sub_control_source = sub["control_source"].as<std::string>();
      }
    }
    if (const YAML::Node pub = ros["pub_topic"]; pub && pub.IsMap()) {
      if (pub["joint_target"] && !pub["joint_target"].IsNull()) {
        ros_config_.pub_joint_target = pub["joint_target"].as<std::string>();
      }
      if (pub["solver_state"] && !pub["solver_state"].IsNull()) {
        ros_config_.pub_solver_state = pub["solver_state"].as<std::string>();
      }
      if (pub["reset_ack"] && !pub["reset_ack"].IsNull()) {
        ros_config_.pub_reset_ack = pub["reset_ack"].as<std::string>();
      }
    }
  }

  // Parse model section
  const YAML::Node model_cfg = cfg["model"];
  if (!model_cfg || !model_cfg["urdf"]) {
    throw std::invalid_argument("model.urdf is required");
  }
  const auto urdf_path =
      config_path_.parent_path() / model_cfg["urdf"].as<std::string>();
  if (model_cfg["free_joints"] && model_cfg["free_joints"].IsSequence()) {
    for (const auto& j : model_cfg["free_joints"]) {
      free_joint_names_.push_back(j.as<std::string>());
    }
  }
  if (free_joint_names_.empty()) {
    throw std::invalid_argument("model.free_joints must not be empty");
  }

  interface_ =
      std::make_unique<PinocchioInterfaceV3>(urdf_path, free_joint_names_);
  q_ = interface_->copy_q();
  command_q_ = q_;

  // Parse control section
  const YAML::Node control = cfg["control"];
  if (control && control.IsMap()) {
    if (control["dt"]) dt_ = control["dt"].as<double>();
    if (control["method"]) method_ = control["method"].as<std::string>();
    if (control["damping"]) damping_ = control["damping"].as<double>();
  }
  if (dt_ <= 0.0) {
    throw std::invalid_argument("control.dt must be positive");
  }
  if (method_ != "control" && method_ != "retarget") {
    throw std::invalid_argument("control.method must be control or retarget");
  }
  if (damping_override.has_value()) {
    damping_ = *damping_override;
  }
  if (damping_ < 0.0) {
    throw std::invalid_argument("control.damping must be non-negative");
  }

  // Resolve joints_init from config or peer arm_teleop.yaml
  if (control && control["joints_init"]) {
    const YAML::Node jinit = control["joints_init"];
    if (jinit.IsMap()) {
      for (const auto& kv : jinit) {
        joints_init_[kv.first.as<std::string>()] = kv.second.as<double>();
      }
    } else if (jinit.IsSequence() &&
               jinit.size() == free_joint_names_.size()) {
      for (std::size_t i = 0; i < free_joint_names_.size(); ++i) {
        joints_init_[free_joint_names_[i]] = jinit[i].as<double>();
      }
    }
  }

  const auto arm_teleop_file = config_path_.parent_path() / "arm_teleop.yaml";
  if (std::filesystem::is_regular_file(arm_teleop_file)) {
    try {
      const YAML::Node arm_cfg = YAML::LoadFile(arm_teleop_file.string());
      if (joints_init_.empty() && arm_cfg["robot"] &&
          arm_cfg["robot"]["initial_joints"] &&
          arm_cfg["robot"]["initial_joints"].IsMap()) {
        const YAML::Node init_map = arm_cfg["robot"]["initial_joints"];
        for (const auto& name : free_joint_names_) {
          if (init_map[name]) {
            joints_init_[name] = init_map[name].as<double>();
          }
        }
      }
      if (arm_cfg["control"] && arm_cfg["control"]["legacy_intervention"]) {
        peer_teleop_.legacy_intervention =
            arm_cfg["control"]["legacy_intervention"].as<bool>();
      }
      if (arm_cfg["arms"] && arm_cfg["arms"].IsMap()) {
        for (const auto& kv : arm_cfg["arms"]) {
          const YAML::Node arm = kv.second;
          if (arm && arm["joint_names"] && arm["joint_names"].IsSequence()) {
            for (const auto& jn : arm["joint_names"]) {
              peer_teleop_.arm_joint_names.push_back(jn.as<std::string>());
            }
          }
        }
      }
      if (arm_cfg["grippers"] && arm_cfg["grippers"].IsMap()) {
        for (const std::string side : {"right", "left"}) {
          const YAML::Node g = arm_cfg["grippers"][side];
          if (g && g.IsMap() && g["sim_joint"] && g["sim_closed_position"]) {
            peer_teleop_.gripper_limits[g["sim_joint"].as<std::string>()] =
                g["sim_closed_position"].as<double>();
          }
        }
      }
    } catch (...) {
      // Match Python's tolerant peer arm_teleop.yaml parsing.
    }
  }

  // Build tasks
  std::unordered_map<std::string, int> name_to_col;
  for (int i = 0; i < static_cast<int>(free_joint_names_.size()); ++i) {
    name_to_col[free_joint_names_[i]] = i;
  }

  const YAML::Node task_cfg = cfg["task"];
  if (task_cfg && task_cfg.IsMap()) {
    if (const YAML::Node poses = task_cfg["pose"]; poses && poses.IsSequence()) {
      for (const auto& item : poses) {
        FrameTaskSpec spec;
        spec.root = item[0][0].as<std::string>();
        spec.frame = item[0][1].as<std::string>();
        spec.root_id = interface_->frame_id(spec.root);
        spec.frame_id = interface_->frame_id(spec.frame);
        spec.position_cost = item[1].as<double>();
        spec.orientation_cost = item[2].as<double>();
        spec.gain = (item.size() > 3) ? item[3].as<double>() : 1.0;
        pose_tasks_.push_back(std::move(spec));
      }
    }
    if (const YAML::Node axes = task_cfg["axis"]; axes && axes.IsSequence()) {
      for (const auto& item : axes) {
        AxisTaskSpec spec;
        spec.root = item[0][0].as<std::string>();
        spec.frame = item[0][1].as<std::string>();
        spec.root_id = interface_->frame_id(spec.root);
        spec.frame_id = interface_->frame_id(spec.frame);
        const std::string axis = to_upper(item[1].as<std::string>());
        if (axis == "X") {
          spec.axis_index = 0;
        } else if (axis == "Y") {
          spec.axis_index = 1;
        } else if (axis == "Z") {
          spec.axis_index = 2;
        } else {
          throw std::invalid_argument("axis must be X/Y/Z, got " + axis);
        }
        spec.cost = item[2].as<double>();
        spec.gain = (item.size() > 3) ? item[3].as<double>() : 1.0;
        axis_tasks_.push_back(std::move(spec));
      }
    }
    bool had_joint_items = false;
    if (const YAML::Node joints = task_cfg["joint"];
        joints && joints.IsSequence()) {
      for (const auto& item : joints) {
        had_joint_items = true;
        JointTaskSpec spec;
        for (const auto& jn : item[0]) {
          const std::string name = jn.as<std::string>();
          spec.joint_names.push_back(name);
          auto it = name_to_col.find(name);
          spec.free_columns.push_back(it == name_to_col.end() ? -1
                                                              : it->second);
        }
        spec.cost = item[1].as<double>();
        spec.gain = (item.size() > 2) ? item[2].as<double>() : 1.0;
        if (item.size() > 3 && !item[3].IsNull()) {
          if (item[3].IsSequence()) {
            for (const auto& v : item[3]) {
              spec.target.push_back(v.as<double>());
            }
          } else if (item[3].IsMap()) {
            for (const auto& name : spec.joint_names) {
              if (item[3][name]) {
                spec.target.push_back(item[3][name].as<double>());
              }
            }
          }
          if (spec.target.size() != spec.joint_names.size()) {
            throw std::invalid_argument("joint target size mismatch");
          }
          spec.has_target = true;
        } else if (!joints_init_.empty()) {
          std::vector<double> resolved;
          resolved.reserve(spec.joint_names.size());
          bool all_found = true;
          for (const auto& name : spec.joint_names) {
            auto it = joints_init_.find(name);
            if (it == joints_init_.end()) {
              all_found = false;
              break;
            }
            resolved.push_back(it->second);
          }
          if (all_found) {
            spec.target = std::move(resolved);
            spec.has_target = true;
          }
        }
        joint_tasks_.push_back(std::move(spec));
      }
    }

    if (!had_joint_items && control && control.IsMap() &&
        !joints_init_.empty()) {
      const double posture_cost =
          control["posture_cost"] ? control["posture_cost"].as<double>() : 0.0;
      if (posture_cost > 0.0) {
        const double posture_gain = control["posture_gain"]
                                        ? control["posture_gain"].as<double>()
                                        : 1.0;
        std::vector<double> resolved;
        resolved.reserve(free_joint_names_.size());
        bool all_found = true;
        for (const auto& name : free_joint_names_) {
          auto it = joints_init_.find(name);
          if (it == joints_init_.end()) {
            all_found = false;
            break;
          }
          resolved.push_back(it->second);
        }
        if (all_found) {
          JointTaskSpec spec;
          spec.joint_names = free_joint_names_;
          spec.free_columns.resize(free_joint_names_.size());
          for (int i = 0; i < static_cast<int>(free_joint_names_.size()); ++i) {
            spec.free_columns[i] = i;
          }
          spec.cost = posture_cost;
          spec.gain = posture_gain;
          spec.target = std::move(resolved);
          spec.has_target = true;
          joint_tasks_.push_back(std::move(spec));
        }
      }
    }
  }

  // Limits
  const int nv_free = interface_->nv_free();
  velocity_limits_ =
      Eigen::VectorXd::Constant(nv_free, std::numeric_limits<double>::infinity());
  const YAML::Node limit_cfg = cfg["limit"];
  if (limit_cfg && limit_cfg.IsMap()) {
    if (const YAML::Node vel = limit_cfg["velocity"];
        vel && vel.IsSequence() && vel.size() > 0) {
      if (static_cast<int>(vel.size()) != nv_free) {
        throw std::invalid_argument(
            "velocity limit count must match free_joints");
      }
      for (int i = 0; i < nv_free; ++i) {
        velocity_limits_[i] = vel[i].as<double>() * kDegToRad;
      }
    }
    if (const YAML::Node acc = limit_cfg["acceleration_rad_s2"];
        acc && !acc.IsNull()) {
      Eigen::VectorXd acc_vec(nv_free);
      if (acc.IsScalar()) {
        acc_vec.setConstant(acc.as<double>());
      } else if (acc.IsSequence() && static_cast<int>(acc.size()) == nv_free) {
        for (int i = 0; i < nv_free; ++i) {
          acc_vec[i] = acc[i].as<double>();
        }
      } else {
        throw std::invalid_argument(
            "acceleration_rad_s2 must be positive scalar or one per free joint");
      }
      for (int i = 0; i < nv_free; ++i) {
        if (!std::isfinite(acc_vec[i]) || acc_vec[i] <= 0.0) {
          throw std::invalid_argument(
              "acceleration_rad_s2 must be positive scalar or one per free joint");
        }
      }
      if (method_ != "retarget") {
        throw std::invalid_argument(
            "acceleration limits require continuous retarget integration");
      }
      acceleration_limits_ = acc_vec;
    }
  }

  previous_velocity_ = Eigen::VectorXd::Zero(nv_free);
}

void ControllerV23::set_method(const std::string& method) {
  if (method != "control" && method != "retarget") {
    throw std::invalid_argument("control.method must be control or retarget");
  }
  method_ = method;
}

void ControllerV23::update_joint_state(const std::vector<std::string>& names,
                                       const std::vector<double>& positions) {
  q_ = interface_->update_named_joints(names, positions);
  if (!have_feedback_) {
    command_q_ = q_;
    have_feedback_ = true;
  }
}

void ControllerV23::update_tf_targets(
    const std::map<TargetStore::Key, TargetTransform>& transforms) {
  targets_.update(transforms);
}

void ControllerV23::reset_to_feedback() {
  if (!have_feedback_) {
    return;
  }
  command_q_ = q_;
  previous_velocity_.setZero();
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> ControllerV23::stack_tasks(
    const Eigen::VectorXd& q) {
  const int nv_free = interface_->nv_free();
  interface_->forward_with_jacobians(q);

  std::vector<Eigen::RowVectorXd> prim_rows;
  std::vector<double> prim_targets;
  std::vector<Eigen::RowVectorXd> sec_rows;
  std::vector<double> sec_targets;

  prim_rows.reserve(pose_tasks_.size() * 6 + axis_tasks_.size() * 3);
  prim_targets.reserve(pose_tasks_.size() * 6 + axis_tasks_.size() * 3);

  for (const auto& task : pose_tasks_) {
    const auto target_opt = targets_.relative_target(task.root, task.frame);
    if (!target_opt.has_value()) {
      continue;
    }
    const pinocchio::SE3 current =
        interface_->relative_pose_cached(task.root_id, task.frame_id);
    const auto jac =
        interface_->relative_jacobian_cached(task.root_id, task.frame_id);

    const Eigen::Vector3d pos_err =
        target_opt->translation() - current.translation();
    const Eigen::Vector3d rot_err_local =
        pinocchio::log3(current.rotation().transpose() * target_opt->rotation());
    const Eigen::Vector3d rot_err = current.rotation() * rot_err_local;

    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = task.gain * pos_err;
    error.tail<3>() = task.gain * rot_err;

    const double w_pos = std::sqrt(task.position_cost);
    const double w_ori = std::sqrt(task.orientation_cost);
    for (int r = 0; r < 6; ++r) {
      const double w = (r < 3) ? w_pos : w_ori;
      const Eigen::RowVectorXd row = jac.row(r) * w;
      if (row.norm() > 0.0) {
        prim_rows.push_back(row);
        prim_targets.push_back(error[r] * w);
      }
    }
  }

  for (const auto& task : axis_tasks_) {
    const auto target_opt = targets_.relative_target(task.root, task.frame);
    if (!target_opt.has_value()) {
      continue;
    }
    const pinocchio::SE3 current =
        interface_->relative_pose_cached(task.root_id, task.frame_id);
    const auto jac =
        interface_->relative_jacobian_cached(task.root_id, task.frame_id);

    const Eigen::Vector3d cur_axis = current.rotation().col(task.axis_index);
    const Eigen::Vector3d tgt_axis = target_opt->rotation().col(task.axis_index);
    const Eigen::Vector3d error = task.gain * cur_axis.cross(tgt_axis);
    const double w = std::sqrt(task.cost);
    for (int r = 0; r < 3; ++r) {
      const Eigen::RowVectorXd row = jac.row(3 + r) * w;
      if (row.norm() > 0.0) {
        prim_rows.push_back(row);
        prim_targets.push_back(error[r] * w);
      }
    }
  }

  for (const auto& task : joint_tasks_) {
    if (!task.has_target) {
      continue;
    }
    const double w = std::sqrt(task.cost);
    if (w <= 0.0) {
      continue;
    }
    for (std::size_t i = 0; i < task.joint_names.size(); ++i) {
      const int col = task.free_columns[i];
      if (col < 0) {
        continue;
      }
      Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(nv_free);
      row[col] = w;
      const double current_val = q[interface_->joints()[col].q_index];
      const double err = task.gain * (task.target[i] - current_val) * w;
      sec_rows.push_back(std::move(row));
      sec_targets.push_back(err);
    }
  }

  auto to_matrix = [nv_free](const std::vector<Eigen::RowVectorXd>& rows) {
    Eigen::MatrixXd mat(static_cast<Eigen::Index>(rows.size()), nv_free);
    for (std::size_t i = 0; i < rows.size(); ++i) {
      mat.row(static_cast<Eigen::Index>(i)) = rows[i];
    }
    return mat;
  };
  auto to_vector = [](const std::vector<double>& vals) {
    Eigen::VectorXd vec(static_cast<Eigen::Index>(vals.size()));
    for (std::size_t i = 0; i < vals.size(); ++i) {
      vec[static_cast<Eigen::Index>(i)] = vals[i];
    }
    return vec;
  };

  if (prim_rows.empty()) {
    if (sec_rows.empty()) {
      return {Eigen::MatrixXd::Zero(0, nv_free), Eigen::VectorXd::Zero(0)};
    }
    return {to_matrix(sec_rows), to_vector(sec_targets)};
  }

  const Eigen::MatrixXd j_primary = to_matrix(prim_rows);
  const Eigen::VectorXd t_primary = to_vector(prim_targets);

  if (sec_rows.empty()) {
    return {j_primary, t_primary};
  }

  const Eigen::MatrixXd j_secondary = to_matrix(sec_rows);
  const Eigen::VectorXd t_secondary = to_vector(sec_targets);

  constexpr double kLam = 1e-3;
  const Eigen::Index m_prim = j_primary.rows();
  const Eigen::MatrixXd reg =
      j_primary * j_primary.transpose() +
      (kLam * kLam) * Eigen::MatrixXd::Identity(m_prim, m_prim);
  const Eigen::MatrixXd j_pinv = j_primary.transpose() * invert_spd_or_lsq(reg);
  const Eigen::MatrixXd n_proj =
      Eigen::MatrixXd::Identity(nv_free, nv_free) - j_pinv * j_primary;
  const Eigen::MatrixXd j_sec_proj = j_secondary * n_proj;

  Eigen::VectorXd v_des = Eigen::VectorXd::Zero(nv_free);
  for (Eigen::Index i = 0; i < t_secondary.size(); ++i) {
    const double row_norm = j_secondary.row(i).norm();
    if (row_norm > 1e-12) {
      Eigen::Index col = 0;
      j_secondary.row(i).cwiseAbs().maxCoeff(&col);
      v_des[col] = t_secondary[i] / row_norm;
    }
  }
  const Eigen::VectorXd t_sec_proj = j_secondary * (n_proj * v_des);

  Eigen::MatrixXd stacked(j_primary.rows() + j_sec_proj.rows(), nv_free);
  stacked.topRows(j_primary.rows()) = j_primary;
  stacked.bottomRows(j_sec_proj.rows()) = j_sec_proj;

  Eigen::VectorXd stacked_t(t_primary.size() + t_sec_proj.size());
  stacked_t.head(t_primary.size()) = t_primary;
  stacked_t.tail(t_sec_proj.size()) = t_sec_proj;
  return {stacked, stacked_t};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
ControllerV23::compute_velocity_bounds(const Eigen::VectorXd& q,
                                       double dt) const {
  const int count = interface_->nv_free();
  Eigen::VectorXd lower = -velocity_limits_;
  Eigen::VectorXd upper = velocity_limits_;
  const auto [q_lower, q_upper] = interface_->configuration_bounds_free();
  const Eigen::VectorXd q_free = interface_->free_configuration(q);
  if (dt > 0.0) {
    for (int i = 0; i < count; ++i) {
      if (std::isfinite(q_lower[i])) {
        lower[i] = std::max(lower[i], (q_lower[i] - q_free[i]) / dt);
      }
      if (std::isfinite(q_upper[i])) {
        upper[i] = std::min(upper[i], (q_upper[i] - q_free[i]) / dt);
      }
    }
  }
  for (int i = 0; i < count; ++i) {
    if (lower[i] > upper[i]) {
      throw std::invalid_argument("inconsistent velocity/configuration bounds");
    }
  }
  return {lower, upper};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
ControllerV23::apply_acceleration_bounds(const Eigen::VectorXd& q,
                                         double dt,
                                         Eigen::VectorXd lower,
                                         Eigen::VectorXd upper) const {
  const auto& acc = *acceleration_limits_;
  const auto [q_lower, q_upper] = interface_->configuration_bounds_free();
  const Eigen::VectorXd q_free = interface_->free_configuration(q);
  const int count = interface_->nv_free();

  auto stopping_speed = [dt](double dist, double a) -> double {
    if (!std::isfinite(dist)) {
      return std::numeric_limits<double>::infinity();
    }
    const double d = std::max(dist, 0.0);
    const double n =
        std::floor((std::sqrt(1.0 + 8.0 * d / (a * dt * dt)) - 1.0) / 2.0);
    return d / ((n + 1.0) * dt) + 0.5 * a * dt * n;
  };

  for (int i = 0; i < count; ++i) {
    lower[i] =
        std::max(lower[i], -stopping_speed(q_free[i] - q_lower[i], acc[i]));
    upper[i] =
        std::min(upper[i], stopping_speed(q_upper[i] - q_free[i], acc[i]));
    lower[i] = std::max(lower[i], previous_velocity_[i] - acc[i] * dt);
    upper[i] = std::min(upper[i], previous_velocity_[i] + acc[i] * dt);
    if (lower[i] > upper[i]) {
      const double mid = 0.5 * (q_lower[i] + q_upper[i]);
      const double safe_val = (q_free[i] > mid) ? upper[i] : lower[i];
      lower[i] = safe_val;
      upper[i] = safe_val;
    }
    const double lo = std::min(lower[i], upper[i]);
    const double hi = std::max(lower[i], upper[i]);
    lower[i] = lo;
    upper[i] = hi;
  }
  return {lower, upper};
}

double ControllerV23::compute_adaptive_damping(
    const Eigen::MatrixXd& matrix) const {
  const double base = std::max(damping_, 1.5e-3);
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    return base;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  const auto& s = svd.singularValues();
  if (s.size() > 0) {
    const double s_min = s[s.size() - 1];
    constexpr double kSigmaThresh = 0.08;
    if (s_min < kSigmaThresh) {
      const double ratio = s_min / kSigmaThresh;
      return base + 0.05 * (1.0 - ratio) * (1.0 - ratio);
    }
  }
  return base;
}

SolveIKResult ControllerV23::solve_ik_internal(const Eigen::VectorXd& base_q,
                                               double step_dt) {
  const auto [matrix, target] = stack_tasks(base_q);
  auto [lower, upper] = compute_velocity_bounds(base_q, step_dt);
  if (acceleration_limits_.has_value()) {
    std::tie(lower, upper) = apply_acceleration_bounds(
        base_q, step_dt, std::move(lower), std::move(upper));
  }

  const int count = interface_->nv_free();
  Eigen::VectorXd velocity;
  double residual = 0.0;
  if (matrix.rows() == 0) {
    velocity = Eigen::VectorXd::Zero(count);
  } else {
    const double eff_damping = compute_adaptive_damping(matrix);
    velocity = solve_box_lsq(matrix, target, eff_damping, lower, upper);
    residual = (matrix * velocity - target).norm();
  }

  SolveIKResult res;
  res.velocity = velocity;
  res.residual_norm = residual;
  res.rows = static_cast<int>(matrix.rows());
  res.active_lower.resize(count, false);
  res.active_upper.resize(count, false);
  constexpr double kTol = 1e-9;
  for (int i = 0; i < count; ++i) {
    if (std::isfinite(lower[i]) &&
        std::abs(velocity[i] - lower[i]) <= kTol + 1e-5 * std::abs(lower[i])) {
      res.active_lower[i] = true;
    }
    if (std::isfinite(upper[i]) &&
        std::abs(velocity[i] - upper[i]) <= kTol + 1e-5 * std::abs(upper[i])) {
      res.active_upper[i] = true;
    }
  }
  return res;
}

std::pair<std::vector<std::string>, Eigen::VectorXd> ControllerV23::step(
    std::optional<double> dt_override) {
  if (!have_feedback_) {
    throw std::runtime_error("joint feedback has not been received");
  }
  const double step_dt = dt_override.value_or(dt_);
  if (step_dt <= 0.0) {
    throw std::invalid_argument("step dt must be positive");
  }
  const Eigen::VectorXd& base_q = (method_ == "control") ? q_ : command_q_;
  SolveIKResult result = solve_ik_internal(base_q, step_dt);
  Eigen::VectorXd next_q =
      interface_->integrate_free(base_q, result.velocity, step_dt);
  next_q = interface_->clip_configuration(next_q);
  if (!next_q.allFinite()) {
    throw std::runtime_error("IK produced a non-finite configuration");
  }
  command_q_ = next_q;
  previous_velocity_ = result.velocity;
  last_result_ = std::move(result);
  return {free_joint_names_, interface_->free_configuration(next_q)};
}

}  // namespace hc::v23

