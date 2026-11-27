#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace py = pybind11;

namespace {

inline bool is_finite(double v) { return std::isfinite(v); }

inline double clamp_d(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Fast extraction of fixed-size 1D double vector from list/tuple/ndarray.
// Returns false if shape does not match N or contains non-numeric nested structures.
template <std::size_t N>
bool try_read_fixed_vec(const py::handle& obj, double (&out)[N]) {
  PyObject* ptr = obj.ptr();
  if (PyList_CheckExact(ptr)) {
    if (PyList_GET_SIZE(ptr) != static_cast<Py_ssize_t>(N)) return false;
    for (std::size_t i = 0; i < N; ++i) {
      PyObject* item = PyList_GET_ITEM(ptr, static_cast<Py_ssize_t>(i));
      double v = PyFloat_AsDouble(item);
      if (v == -1.0 && PyErr_Occurred()) {
        PyErr_Clear();
        return false;
      }
      out[i] = v;
    }
    return true;
  }
  if (PyTuple_CheckExact(ptr)) {
    if (PyTuple_GET_SIZE(ptr) != static_cast<Py_ssize_t>(N)) return false;
    for (std::size_t i = 0; i < N; ++i) {
      PyObject* item = PyTuple_GET_ITEM(ptr, static_cast<Py_ssize_t>(i));
      double v = PyFloat_AsDouble(item);
      if (v == -1.0 && PyErr_Occurred()) {
        PyErr_Clear();
        return false;
      }
      out[i] = v;
    }
    return true;
  }
  if (PyObject_CheckBuffer(ptr)) {
    Py_buffer view;
    if (PyObject_GetBuffer(ptr, &view, PyBUF_FORMAT | PyBUF_STRIDES) == 0) {
      const bool ok =
          (view.ndim == 1 && view.shape[0] == static_cast<Py_ssize_t>(N) &&
           view.format != nullptr && view.format[0] == 'd' &&
           view.format[1] == '\0');
      if (ok) {
        const auto* base = static_cast<const char*>(view.buf);
        const Py_ssize_t stride = view.strides[0];
        for (std::size_t i = 0; i < N; ++i) {
          out[i] = *reinterpret_cast<const double*>(base + i * stride);
        }
        PyBuffer_Release(&view);
        return true;
      }
      PyBuffer_Release(&view);
    } else {
      PyErr_Clear();
    }
  }
  // General fallback via NumPy array coercion.
  py::array_t<double, py::array::c_style | py::array::forcecast> arr;
  try {
    arr = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(obj);
  } catch (...) {
    return false;
  }
  if (!arr || arr.ndim() != 1 || arr.shape(0) != static_cast<py::ssize_t>(N)) {
    return false;
  }
  const double* data = arr.data();
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = data[i];
  }
  return true;
}

bool try_read_mat3(const py::handle& obj, Eigen::Matrix3d& out) {
  PyObject* ptr = obj.ptr();
  if (PyObject_CheckBuffer(ptr)) {
    Py_buffer view;
    if (PyObject_GetBuffer(ptr, &view, PyBUF_FORMAT | PyBUF_STRIDES) == 0) {
      const bool ok = (view.ndim == 2 && view.shape[0] == 3 && view.shape[1] == 3 &&
                       view.format != nullptr && view.format[0] == 'd' &&
                       view.format[1] == '\0');
      if (ok) {
        const auto* base = static_cast<const char*>(view.buf);
        const Py_ssize_t s0 = view.strides[0];
        const Py_ssize_t s1 = view.strides[1];
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            out(r, c) = *reinterpret_cast<const double*>(base + r * s0 + c * s1);
          }
        }
        PyBuffer_Release(&view);
        return true;
      }
      PyBuffer_Release(&view);
    } else {
      PyErr_Clear();
    }
  }
  if (PyList_CheckExact(ptr) && PyList_GET_SIZE(ptr) == 3) {
    double row[3];
    bool all_ok = true;
    for (int r = 0; r < 3; ++r) {
      if (!try_read_fixed_vec<3>(PyList_GET_ITEM(ptr, r), row)) {
        all_ok = false;
        break;
      }
      out(r, 0) = row[0];
      out(r, 1) = row[1];
      out(r, 2) = row[2];
    }
    if (all_ok) return true;
  }
  py::array_t<double, py::array::c_style | py::array::forcecast> arr;
  try {
    arr = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(obj);
  } catch (...) {
    return false;
  }
  if (!arr || arr.ndim() != 2 || arr.shape(0) != 3 || arr.shape(1) != 3) {
    return false;
  }
  const double* d = arr.data();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out(r, c) = d[r * 3 + c];
    }
  }
  return true;
}

Eigen::Matrix3d require_mat3(const py::handle& obj, const char* err_msg) {
  Eigen::Matrix3d m;
  if (!try_read_mat3(obj, m) || !m.allFinite()) {
    throw py::value_error(err_msg);
  }
  return m;
}

Eigen::Vector3d require_vec3(const py::handle& obj) {
  double v[3];
  if (!try_read_fixed_vec<3>(obj, v)) {
    throw py::value_error("expected a 3-element sequence");
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

Eigen::Vector4d normalize_quat_core(double x, double y, double z, double w) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw py::value_error("invalid quaternion");
  }
  const double inv = 1.0 / norm;
  return Eigen::Vector4d(x * inv, y * inv, z * inv, w * inv);
}

Eigen::Vector4d require_quat(const py::handle& obj) {
  double v[4];
  if (!try_read_fixed_vec<4>(obj, v)) {
    throw py::value_error("invalid quaternion");
  }
  return normalize_quat_core(v[0], v[1], v[2], v[3]);
}

inline Eigen::Vector4d quat_mul_core(const Eigen::Vector4d& q1,
                                     const Eigen::Vector4d& q2) {
  const double x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
  const double x2 = q2[0], y2 = q2[1], z2 = q2[2], w2 = q2[3];
  return normalize_quat_core(
      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
      w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2);
}

inline Eigen::Vector4d quat_inv_core(const Eigen::Vector4d& q) {
  return Eigen::Vector4d(-q[0], -q[1], -q[2], q[3]);
}

inline Eigen::Matrix3d quat_to_mat_core(const Eigen::Vector4d& q) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  Eigen::Matrix3d R;
  R(0, 0) = 1.0 - 2.0 * (y * y + z * z);
  R(0, 1) = 2.0 * (x * y - z * w);
  R(0, 2) = 2.0 * (x * z + y * w);
  R(1, 0) = 2.0 * (x * y + z * w);
  R(1, 1) = 1.0 - 2.0 * (x * x + z * z);
  R(1, 2) = 2.0 * (y * z - x * w);
  R(2, 0) = 2.0 * (x * z - y * w);
  R(2, 1) = 2.0 * (y * z + x * w);
  R(2, 2) = 1.0 - 2.0 * (x * x + y * y);
  return R;
}

Eigen::Vector4d mat_to_quat_core(const Eigen::Matrix3d& R) {
  const double trace = R(0, 0) + R(1, 1) + R(2, 2);
  double x, y, z, w;
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    x = (R(2, 1) - R(1, 2)) / scale;
    y = (R(0, 2) - R(2, 0)) / scale;
    z = (R(1, 0) - R(0, 1)) / scale;
    w = 0.25 * scale;
  } else {
    int index = 0;
    if (R(1, 1) > R(0, 0)) index = 1;
    if (R(2, 2) > R(index, index)) index = 2;
    if (index == 0) {
      const double scale = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
      x = 0.25 * scale;
      y = (R(0, 1) + R(1, 0)) / scale;
      z = (R(0, 2) + R(2, 0)) / scale;
      w = (R(2, 1) - R(1, 2)) / scale;
    } else if (index == 1) {
      const double scale = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
      x = (R(0, 1) + R(1, 0)) / scale;
      y = 0.25 * scale;
      z = (R(1, 2) + R(2, 1)) / scale;
      w = (R(0, 2) - R(2, 0)) / scale;
    } else {
      const double scale = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
      x = (R(0, 2) + R(2, 0)) / scale;
      y = (R(1, 2) + R(2, 1)) / scale;
      z = 0.25 * scale;
      w = (R(1, 0) - R(0, 1)) / scale;
    }
  }
  return normalize_quat_core(x, y, z, w);
}

Eigen::Vector4d quat_from_axis_angle_core(const Eigen::Vector3d& axis,
                                          double angle) {
  const double norm = axis.norm();
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw py::value_error("axis must be a finite non-zero 3D vector");
  }
  const Eigen::Vector3d u = axis / norm;
  const double half = angle * 0.5;
  const double s = std::sin(half);
  const double c = std::cos(half);
  return normalize_quat_core(u[0] * s, u[1] * s, u[2] * s, c);
}

Eigen::Vector3d orientation_error_core(const Eigen::Vector4d& target,
                                       const Eigen::Vector4d& current) {
  Eigen::Vector4d diff = quat_mul_core(target, quat_inv_core(current));
  if (diff[3] < 0.0) {
    diff = -diff;
  }
  const double vnorm = diff.head<3>().norm();
  if (vnorm < 1e-9) {
    return Eigen::Vector3d::Zero();
  }
  const double angle = 2.0 * std::atan2(vnorm, diff[3]);
  return diff.head<3>() * (angle / vnorm);
}

inline py::array_t<double> to_np_vec3(const Eigen::Vector3d& v) {
  py::array_t<double> out(3);
  double* d = out.mutable_data();
  d[0] = v[0];
  d[1] = v[1];
  d[2] = v[2];
  return out;
}

inline py::array_t<double> to_np_vec4(const Eigen::Vector4d& v) {
  py::array_t<double> out(4);
  double* d = out.mutable_data();
  d[0] = v[0];
  d[1] = v[1];
  d[2] = v[2];
  d[3] = v[3];
  return out;
}

inline py::array_t<double> to_np_mat3(const Eigen::Matrix3d& m) {
  py::array_t<double> out({3, 3});
  double* d = out.mutable_data();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      d[r * 3 + c] = m(r, c);
    }
  }
  return out;
}

// ---------- Exported functions ----------

double binary_trigger_state_py(double trigger, double threshold = 0.5) {
  if (!is_finite(trigger) || !is_finite(threshold)) {
    throw py::value_error("trigger mapping values must be finite");
  }
  if (threshold < 0.0 || threshold > 1.0) {
    throw py::value_error("trigger threshold must be in [0, 1]");
  }
  return clamp_d(trigger, 0.0, 1.0) >= threshold ? 1.0 : 0.0;
}

py::array_t<double> normalize_quaternion_py(py::object value) {
  return to_np_vec4(require_quat(value));
}

py::array_t<double> quaternion_multiply_py(py::object left, py::object right) {
  return to_np_vec4(quat_mul_core(require_quat(left), require_quat(right)));
}

py::array_t<double> quaternion_inverse_py(py::object value) {
  return to_np_vec4(quat_inv_core(require_quat(value)));
}

py::array_t<double> quaternion_from_axis_angle_py(py::object axis,
                                                  double angle) {
  double v[3];
  if (!try_read_fixed_vec<3>(axis, v)) {
    throw py::value_error("axis must be a finite non-zero 3D vector");
  }
  return to_np_vec4(
      quat_from_axis_angle_core(Eigen::Vector3d(v[0], v[1], v[2]), angle));
}

py::array_t<double> quaternion_to_matrix_py(py::object value) {
  return to_np_mat3(quat_to_mat_core(require_quat(value)));
}

py::array_t<double> matrix_to_quaternion_py(py::object matrix) {
  return to_np_vec4(
      mat_to_quat_core(require_mat3(matrix, "rotation matrix must be finite 3x3")));
}

py::array_t<double> orientation_error_py(py::object target,
                                         py::object current) {
  return to_np_vec3(
      orientation_error_core(require_quat(target), require_quat(current)));
}

double mapped_relative_yaw_py(py::object orientation,
                              py::object reference_orientation,
                              py::object axis_mapping) {
  const Eigen::Matrix3d M =
      require_mat3(axis_mapping, "axis_mapping must be finite 3x3");
  const Eigen::Vector4d rel =
      quat_mul_core(require_quat(orientation),
                    quat_inv_core(require_quat(reference_orientation)));
  const Eigen::Matrix3d mapped = M * quat_to_mat_core(rel) * M.transpose();
  const Eigen::Vector4d q = mat_to_quat_core(mapped);
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double mapped_relative_pitch_py(py::object orientation,
                                py::object reference_orientation,
                                py::object axis_mapping) {
  const Eigen::Matrix3d M =
      require_mat3(axis_mapping, "axis_mapping must be finite 3x3");
  const Eigen::Vector3d src_fwd = M.transpose().col(0);
  auto pitch = [&](const Eigen::Vector4d& q) -> double {
    const Eigen::Vector3d fwd = M * (quat_to_mat_core(q) * src_fwd);
    return std::atan2(-fwd[2], std::hypot(fwd[0], fwd[1]));
  };
  return pitch(require_quat(orientation)) -
         pitch(require_quat(reference_orientation));
}

py::tuple stabilize_pose_py(py::object target_position,
                            py::object target_orientation,
                            py::object previous_position,
                            py::object previous_orientation,
                            double position_deadband,
                            double orientation_deadband,
                            double filter_alpha,
                            std::optional<double> max_linear_step,
                            std::optional<double> max_angular_step) {
  if (position_deadband < 0.0 || orientation_deadband < 0.0) {
    throw py::value_error("pose deadbands must be non-negative");
  }
  if (!(filter_alpha > 0.0 && filter_alpha <= 1.0)) {
    throw py::value_error("filter_alpha must be in (0, 1]");
  }

  const Eigen::Vector3d t_pos = require_vec3(target_position);
  const Eigen::Vector3d p_pos = require_vec3(previous_position);
  const Eigen::Vector3d pos_delta = t_pos - p_pos;
  const double pos_dist = pos_delta.norm();

  Eigen::Vector3d out_pos;
  if (pos_dist <= position_deadband) {
    out_pos = p_pos;
  } else {
    Eigen::Vector3d step_delta =
        pos_delta * (filter_alpha * (pos_dist - position_deadband) / pos_dist);
    if (max_linear_step.has_value() && *max_linear_step > 0.0) {
      const double step_norm = step_delta.norm();
      if (step_norm > *max_linear_step) {
        step_delta *= (*max_linear_step / step_norm);
      }
    }
    out_pos = p_pos + step_delta;
  }

  const Eigen::Vector4d prev_q = require_quat(previous_orientation);
  const Eigen::Vector4d targ_q = require_quat(target_orientation);
  const Eigen::Vector3d rot_delta = orientation_error_core(targ_q, prev_q);
  const double rot_angle = rot_delta.norm();

  Eigen::Vector4d out_q;
  if (rot_angle <= orientation_deadband) {
    out_q = prev_q;
  } else {
    double step_angle = filter_alpha * (rot_angle - orientation_deadband);
    if (max_angular_step.has_value() && *max_angular_step > 0.0) {
      step_angle = std::min(step_angle, *max_angular_step);
    }
    const Eigen::Vector4d step =
        quat_from_axis_angle_core(rot_delta / rot_angle, step_angle);
    out_q = quat_mul_core(step, prev_q);
  }
  return py::make_tuple(to_np_vec3(out_pos), to_np_vec4(out_q));
}

bool sticks_outward_py(double left_x, double right_x, double threshold) {
  if (!is_finite(left_x) || !is_finite(right_x) || !is_finite(threshold)) {
    return false;
  }
  if (!(threshold > 0.0 && threshold <= 1.0)) {
    throw py::value_error("stick threshold must be in (0, 1]");
  }
  return left_x <= -threshold && right_x >= threshold;
}

py::tuple joystick_base_velocity_py(double stick_x,
                                    double stick_y,
                                    double deadzone,
                                    double max_linear_velocity,
                                    double max_lateral_velocity,
                                    double forward_direction = 1.0,
                                    double lateral_direction = -1.0) {
  if (!is_finite(stick_x) || !is_finite(stick_y) || !is_finite(deadzone) ||
      !is_finite(max_linear_velocity) || !is_finite(max_lateral_velocity) ||
      !is_finite(forward_direction) || !is_finite(lateral_direction)) {
    throw py::value_error("joystick base mapping values must be finite");
  }
  if (!(deadzone >= 0.0 && deadzone < 1.0)) {
    throw py::value_error("joystick deadzone must be in [0, 1)");
  }
  if (max_linear_velocity < 0.0 || max_lateral_velocity < 0.0) {
    throw py::value_error("joystick velocity limits must be non-negative");
  }
  if ((forward_direction != -1.0 && forward_direction != 1.0) ||
      (lateral_direction != -1.0 && lateral_direction != 1.0)) {
    throw py::value_error("joystick directions must be -1 or 1");
  }
  auto shape = [deadzone](double v) -> double {
    v = clamp_d(v, -1.0, 1.0);
    const double av = std::abs(v);
    if (av <= deadzone) return 0.0;
    return std::copysign((av - deadzone) / (1.0 - deadzone), v);
  };
  return py::make_tuple(shape(stick_y) * max_linear_velocity * forward_direction,
                        shape(stick_x) * max_lateral_velocity * lateral_direction);
}

double joystick_yaw_velocity_py(double stick_yaw,
                                double deadzone,
                                double max_angular_velocity,
                                double yaw_direction = -1.0) {
  if (!is_finite(stick_yaw) || !is_finite(deadzone) ||
      !is_finite(max_angular_velocity) || !is_finite(yaw_direction)) {
    throw py::value_error("joystick yaw mapping values must be finite");
  }
  if (!(deadzone >= 0.0 && deadzone < 1.0)) {
    throw py::value_error("joystick deadzone must be in [0, 1)");
  }
  if (max_angular_velocity < 0.0) {
    throw py::value_error("max_angular_velocity must be non-negative");
  }
  if (yaw_direction != -1.0 && yaw_direction != 1.0) {
    throw py::value_error("yaw_direction must be -1 or 1");
  }
  const double val = clamp_d(stick_yaw, -1.0, 1.0);
  const double aval = std::abs(val);
  if (aval <= deadzone) return 0.0;
  const double shaped = std::copysign((aval - deadzone) / (1.0 - deadzone), val);
  return shaped * max_angular_velocity * yaw_direction;
}

double adaptive_damping_py(double smallest_singular_value,
                           double minimum,
                           double maximum,
                           double singularity_threshold) {
  if (!is_finite(smallest_singular_value) || !is_finite(minimum) ||
      !is_finite(maximum) || !is_finite(singularity_threshold)) {
    throw py::value_error("adaptive damping values must be finite");
  }
  if (minimum <= 0.0 || maximum < minimum || singularity_threshold <= 0.0) {
    throw py::value_error("invalid adaptive damping bounds");
  }
  if (smallest_singular_value >= singularity_threshold) {
    return minimum;
  }
  const double ratio = std::max(0.0, smallest_singular_value) / singularity_threshold;
  const double diff = 1.0 - ratio;
  return minimum + (maximum - minimum) * diff * diff;
}

py::array_t<double> joint_limit_avoidance_py(py::object joints,
                                             py::object lower,
                                             py::object upper,
                                             double margin_fraction) {
  auto q = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(joints);
  auto lo = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(lower);
  auto hi = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(upper);
  if (!q || !lo || !hi || q.ndim() != 1 || lo.ndim() != 1 || hi.ndim() != 1 ||
      q.shape(0) != lo.shape(0) || q.shape(0) != hi.shape(0)) {
    throw py::value_error("joint limit arrays must be matching vectors");
  }
  const py::ssize_t n = q.shape(0);
  const double* qp = q.data();
  const double* lop = lo.data();
  const double* hip = hi.data();
  for (py::ssize_t i = 0; i < n; ++i) {
    if (!is_finite(qp[i]) || !is_finite(lop[i]) || !is_finite(hip[i])) {
      throw py::value_error("joint limit arrays must be finite");
    }
  }
  if (!(margin_fraction > 0.0 && margin_fraction < 0.5)) {
    throw py::value_error("margin_fraction must be in (0, 0.5)");
  }
  py::array_t<double> out(n);
  double* op = out.mutable_data();
  for (py::ssize_t i = 0; i < n; ++i) {
    const double span = hip[i] - lop[i];
    if (span <= 0.0) {
      throw py::value_error("joint upper limits must exceed lower limits");
    }
    const double nl = (qp[i] - lop[i]) / span;
    const double nh = (hip[i] - qp[i]) / span;
    double val = 0.0;
    if (nl < margin_fraction) {
      const double t = 1.0 - std::max(nl, 0.0) / margin_fraction;
      val += t * t;
    }
    if (nh < margin_fraction) {
      const double t = 1.0 - std::max(nh, 0.0) / margin_fraction;
      val -= t * t;
    }
    op[i] = clamp_d(val, -1.0, 1.0);
  }
  return out;
}

py::tuple relative_target_py(py::object vr_position,
                             py::object vr_orientation,
                             py::object reference_vr_position,
                             py::object reference_vr_orientation,
                             py::object reference_robot_position,
                             py::object reference_robot_orientation,
                             py::object axis_mapping,
                             double position_scale,
                             double max_displacement,
                             bool orientation_enabled) {
  const Eigen::Matrix3d M =
      require_mat3(axis_mapping, "axis_mapping must be finite 3x3");
  const Eigen::Vector3d vr_p = require_vec3(vr_position);
  const Eigen::Vector3d ref_vr_p = require_vec3(reference_vr_position);
  const Eigen::Vector3d ref_rb_p = require_vec3(reference_robot_position);

  Eigen::Vector3d delta = M * (vr_p - ref_vr_p) * position_scale;
  const double length = delta.norm();
  if (max_displacement > 0.0 && length > max_displacement) {
    delta *= (max_displacement / length);
  }
  const Eigen::Vector3d target_pos = ref_rb_p + delta;

  Eigen::Vector4d target_q = require_quat(reference_robot_orientation);
  if (orientation_enabled) {
    const Eigen::Vector4d vr_delta =
        quat_mul_core(require_quat(vr_orientation),
                      quat_inv_core(require_quat(reference_vr_orientation)));
    const Eigen::Matrix3d mapped_delta =
        M * quat_to_mat_core(vr_delta) * M.transpose();
    target_q = quat_mul_core(mat_to_quat_core(mapped_delta), target_q);
  }
  return py::make_tuple(to_np_vec3(target_pos), to_np_vec4(target_q));
}

py::array_t<double> clamp_step_py(py::object target,
                                  py::object previous,
                                  py::object lower,
                                  py::object upper,
                                  double max_step) {
  auto t = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(target);
  auto p = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(previous);
  auto lo = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(lower);
  auto hi = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(upper);
  if (!t || !p || !lo || !hi || t.ndim() != 1 || p.ndim() != 1 ||
      lo.ndim() != 1 || hi.ndim() != 1 || t.shape(0) != p.shape(0) ||
      t.shape(0) != lo.shape(0) || t.shape(0) != hi.shape(0)) {
    throw py::value_error("clamp_step inputs must be matching 1D sequences");
  }
  const py::ssize_t n = t.shape(0);
  const double* tp = t.data();
  const double* pp = p.data();
  const double* lop = lo.data();
  const double* hip = hi.data();

  double max_abs = 0.0;
  for (py::ssize_t i = 0; i < n; ++i) {
    const double ad = std::abs(tp[i] - pp[i]);
    if (ad > max_abs) max_abs = ad;
  }
  const double scale =
      (max_abs > max_step && max_abs > 1e-12) ? (max_step / max_abs) : 1.0;

  py::array_t<double> out(n);
  double* op = out.mutable_data();
  for (py::ssize_t i = 0; i < n; ++i) {
    const double limited = pp[i] + (tp[i] - pp[i]) * scale;
    op[i] = clamp_d(limited, lop[i], hip[i]);
  }
  return out;
}

double head_yaw_orientation_velocity_py(double relative_yaw,
                                        double deadzone_rad,
                                        double active_threshold_rad,
                                        double slow_gain,
                                        double fast_gain,
                                        double slow_max_velocity,
                                        double max_angular_velocity,
                                        double yaw_direction) {
  if (!is_finite(relative_yaw)) return 0.0;
  const double abs_yaw = std::abs(relative_yaw);
  if (abs_yaw <= deadzone_rad || deadzone_rad >= active_threshold_rad) {
    return 0.0;
  }
  const double sign = std::copysign(1.0, relative_yaw) * yaw_direction;
  double vel = 0.0;
  if (abs_yaw <= active_threshold_rad) {
    vel = std::min(slow_gain * (abs_yaw - deadzone_rad), slow_max_velocity);
  } else {
    vel = std::min(slow_max_velocity + fast_gain * (abs_yaw - active_threshold_rad),
                   max_angular_velocity);
  }
  return sign * vel;
}

py::tuple hands_reach_assist_velocity_py(py::sequence hand_offsets,
                                         double deadzone_forward,
                                         double deadzone_backward,
                                         double deadzone_lateral,
                                         double gain_forward,
                                         double gain_backward,
                                         double gain_lateral,
                                         double max_linear_velocity,
                                         double max_lateral_velocity) {
  const py::ssize_t count = py::len(hand_offsets);
  if (count == 0) return py::make_tuple(0.0, 0.0);

  double sum_vx = 0.0, sum_vy = 0.0;
  int active_vx = 0, active_vy = 0;
  int valid = 0;

  for (py::ssize_t i = 0; i < count; ++i) {
    py::sequence pair = hand_offsets[i].cast<py::sequence>();
    const double dx = pair[0].cast<double>();
    const double dy = pair[1].cast<double>();
    if (!is_finite(dx) || !is_finite(dy)) continue;
    ++valid;

    double vx = 0.0;
    if (dx > deadzone_forward) {
      vx = std::min(gain_forward * (dx - deadzone_forward), max_linear_velocity);
    } else if (dx < -deadzone_backward) {
      vx = std::max(gain_backward * (dx + deadzone_backward), -max_linear_velocity);
    }
    sum_vx += vx;
    if (std::abs(vx) > 1e-4) ++active_vx;

    double vy = 0.0;
    if (dy > deadzone_lateral) {
      vy = std::min(gain_lateral * (dy - deadzone_lateral), max_lateral_velocity);
    } else if (dy < -deadzone_lateral) {
      vy = std::max(gain_lateral * (dy + deadzone_lateral), -max_lateral_velocity);
    }
    sum_vy += vy;
    if (std::abs(vy) > 1e-4) ++active_vy;
  }

  if (valid == 0) return py::make_tuple(0.0, 0.0);
  const double avg_vx = active_vx > 0 ? (sum_vx / static_cast<double>(active_vx)) : 0.0;
  const double avg_vy = active_vy > 0 ? (sum_vy / static_cast<double>(active_vy)) : 0.0;
  return py::make_tuple(avg_vx, avg_vy);
}

py::tuple rotate_velocity_to_base_frame_py(double forward,
                                           double lateral,
                                           double heading_yaw) {
  const double c = std::cos(heading_yaw);
  const double s = std::sin(heading_yaw);
  return py::make_tuple(c * forward - s * lateral, s * forward + c * lateral);
}

py::tuple project_displacement_to_heading_frame_py(double dx,
                                                   double dy,
                                                   double heading_yaw) {
  const double c = std::cos(heading_yaw);
  const double s = std::sin(heading_yaw);
  return py::make_tuple(c * dx + s * dy, -s * dx + c * dy);
}

py::tuple blend_three_layer_base_velocity_py(py::object cmd_vel_user,
                                             py::object cmd_vel_assist,
                                             double cmd_vel_head,
                                             double alpha_reach,
                                             double alpha_head,
                                             double max_linear_velocity,
                                             double max_lateral_velocity,
                                             double max_angular_velocity) {
  double u[3];
  if (!try_read_fixed_vec<3>(cmd_vel_user, u)) {
    py::sequence us = cmd_vel_user.cast<py::sequence>();
    u[0] = us[0].cast<double>();
    u[1] = us[1].cast<double>();
    u[2] = us[2].cast<double>();
  }
  py::sequence as = cmd_vel_assist.cast<py::sequence>();
  const double a_fwd = as[0].cast<double>();
  const double a_lat = as[1].cast<double>();

  const double fwd = clamp_d(u[0] + alpha_reach * a_fwd,
                             -max_linear_velocity, max_linear_velocity);
  const double lat = clamp_d(u[1] + alpha_reach * a_lat,
                             -max_lateral_velocity, max_lateral_velocity);
  const double yaw = clamp_d(u[2] + alpha_head * cmd_vel_head,
                             -max_angular_velocity, max_angular_velocity);
  return py::make_tuple(yaw, fwd, lat);
}

py::array_t<double> clutch_base_axis_mapping_py(py::object axis_mapping,
                                                py::object head_orientation,
                                                py::object root_orientation,
                                                py::object task_orientation) {
  const Eigen::Matrix3d M =
      require_mat3(axis_mapping, "axis_mapping must be finite 3x3");
  const Eigen::Matrix3d R_head = quat_to_mat_core(require_quat(head_orientation));
  const Eigen::Vector3d forward = M * R_head.col(2);
  if (std::hypot(forward[0], forward[1]) < 1e-4) {
    throw py::value_error("headset forward has no usable horizontal heading");
  }
  const double yaw = std::atan2(forward[1], forward[0]);
  const double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix3d heading_inv;
  heading_inv << c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0;
  const Eigen::Matrix3d base_to_task =
      quat_to_mat_core(require_quat(task_orientation)).transpose() *
      quat_to_mat_core(require_quat(root_orientation));
  return to_np_mat3(base_to_task * heading_inv * M);
}

}  // namespace

PYBIND11_MODULE(_arm_teleop_math_cpp, m) {
  m.doc() = "C++17 Eigen-accelerated 100 Hz pose/quaternion/slew teleop math";

  constexpr double kDeg15 = 15.0 * M_PI / 180.0;
  constexpr double kDeg45 = 45.0 * M_PI / 180.0;

  m.def("binary_trigger_state", &binary_trigger_state_py,
        py::arg("trigger"), py::arg("threshold") = 0.5);
  m.def("normalize_quaternion", &normalize_quaternion_py, py::arg("value"));
  m.def("quaternion_multiply", &quaternion_multiply_py,
        py::arg("left"), py::arg("right"));
  m.def("quaternion_inverse", &quaternion_inverse_py, py::arg("value"));
  m.def("quaternion_from_axis_angle", &quaternion_from_axis_angle_py,
        py::arg("axis"), py::arg("angle"));
  m.def("quaternion_to_matrix", &quaternion_to_matrix_py, py::arg("value"));
  m.def("matrix_to_quaternion", &matrix_to_quaternion_py, py::arg("matrix"));
  m.def("orientation_error", &orientation_error_py,
        py::arg("target"), py::arg("current"));
  m.def("mapped_relative_yaw", &mapped_relative_yaw_py,
        py::arg("orientation"), py::arg("reference_orientation"),
        py::arg("axis_mapping"));
  m.def("mapped_relative_pitch", &mapped_relative_pitch_py,
        py::arg("orientation"), py::arg("reference_orientation"),
        py::arg("axis_mapping"));
  m.def("stabilize_pose", &stabilize_pose_py,
        py::arg("target_position"), py::arg("target_orientation"),
        py::arg("previous_position"), py::arg("previous_orientation"),
        py::arg("position_deadband"), py::arg("orientation_deadband"),
        py::arg("filter_alpha"),
        py::arg("max_linear_step") = std::nullopt,
        py::arg("max_angular_step") = std::nullopt);
  m.def("sticks_outward", &sticks_outward_py,
        py::arg("left_x"), py::arg("right_x"), py::arg("threshold"));
  m.def("joystick_base_velocity", &joystick_base_velocity_py,
        py::arg("stick_x"), py::arg("stick_y"), py::arg("deadzone"),
        py::arg("max_linear_velocity"), py::arg("max_lateral_velocity"),
        py::arg("forward_direction") = 1.0,
        py::arg("lateral_direction") = -1.0);
  m.def("joystick_yaw_velocity", &joystick_yaw_velocity_py,
        py::arg("stick_yaw"), py::arg("deadzone"),
        py::arg("max_angular_velocity"), py::arg("yaw_direction") = -1.0);
  m.def("adaptive_damping", &adaptive_damping_py,
        py::arg("smallest_singular_value"), py::arg("minimum"),
        py::arg("maximum"), py::arg("singularity_threshold"));
  m.def("joint_limit_avoidance", &joint_limit_avoidance_py,
        py::arg("joints"), py::arg("lower"), py::arg("upper"),
        py::arg("margin_fraction"));
  m.def("relative_target", &relative_target_py,
        py::arg("vr_position"), py::arg("vr_orientation"),
        py::arg("reference_vr_position"), py::arg("reference_vr_orientation"),
        py::arg("reference_robot_position"), py::arg("reference_robot_orientation"),
        py::arg("axis_mapping"), py::arg("position_scale"),
        py::arg("max_displacement"), py::arg("orientation_enabled"));
  m.def("clamp_step", &clamp_step_py,
        py::arg("target"), py::arg("previous"), py::arg("lower"),
        py::arg("upper"), py::arg("max_step"));
  m.def("head_yaw_orientation_velocity", &head_yaw_orientation_velocity_py,
        py::arg("relative_yaw"),
        py::arg("deadzone_rad") = kDeg15,
        py::arg("active_threshold_rad") = kDeg45,
        py::arg("slow_gain") = 0.25,
        py::arg("fast_gain") = 0.70,
        py::arg("slow_max_velocity") = 0.15,
        py::arg("max_angular_velocity") = 0.45,
        py::arg("yaw_direction") = 1.0);
  m.def("hands_reach_assist_velocity", &hands_reach_assist_velocity_py,
        py::arg("hand_offsets"),
        py::arg("deadzone_forward") = 0.08,
        py::arg("deadzone_backward") = 0.06,
        py::arg("deadzone_lateral") = 0.06,
        py::arg("gain_forward") = 0.6,
        py::arg("gain_backward") = 0.5,
        py::arg("gain_lateral") = 0.6,
        py::arg("max_linear_velocity") = 0.20,
        py::arg("max_lateral_velocity") = 0.18);
  m.def("rotate_velocity_to_base_frame", &rotate_velocity_to_base_frame_py,
        py::arg("forward"), py::arg("lateral"), py::arg("heading_yaw"));
  m.def("project_displacement_to_heading_frame",
        &project_displacement_to_heading_frame_py,
        py::arg("dx"), py::arg("dy"), py::arg("heading_yaw"));
  m.def("blend_three_layer_base_velocity", &blend_three_layer_base_velocity_py,
        py::arg("cmd_vel_user"), py::arg("cmd_vel_assist"),
        py::arg("cmd_vel_head"),
        py::arg("alpha_reach") = 1.0,
        py::arg("alpha_head") = 1.0,
        py::arg("max_linear_velocity") = 0.35,
        py::arg("max_lateral_velocity") = 0.25,
        py::arg("max_angular_velocity") = 0.45);
  m.def("clutch_base_axis_mapping", &clutch_base_axis_mapping_py,
        py::arg("axis_mapping"), py::arg("head_orientation"),
        py::arg("root_orientation"), py::arg("task_orientation"));
}

