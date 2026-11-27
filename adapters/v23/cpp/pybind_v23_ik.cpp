#include "v23_ik_core.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_v23_ik_cpp, m) {
  m.doc() = "C++17 Pinocchio + Eigen v23 differential IK core";

  py::class_<hc::v23::TargetTransform>(m, "TargetTransform")
      .def(py::init<>())
      .def(py::init([](const Eigen::Vector3d& t, const Eigen::Matrix3d& r) {
        return hc::v23::TargetTransform{t, r};
      }))
      .def_readwrite("translation", &hc::v23::TargetTransform::translation)
      .def_readwrite("rotation", &hc::v23::TargetTransform::rotation);

  py::class_<hc::v23::SolveIKResult>(m, "SolveIKResult")
      .def_readonly("velocity", &hc::v23::SolveIKResult::velocity)
      .def_readonly("residual_norm", &hc::v23::SolveIKResult::residual_norm)
      .def_readonly("active_lower", &hc::v23::SolveIKResult::active_lower)
      .def_readonly("active_upper", &hc::v23::SolveIKResult::active_upper)
      .def_readonly("rows", &hc::v23::SolveIKResult::rows);

  m.def("solve_box_lsq", &hc::v23::solve_box_lsq,
        py::arg("matrix"), py::arg("target"), py::arg("damping"),
        py::arg("lower"), py::arg("upper"), py::arg("max_iter") = 0);

  m.def("quaternion_to_rotation", &hc::v23::quaternion_to_rotation);

  py::class_<hc::v23::ControllerV23>(m, "ControllerV23Cpp")
      .def(py::init<const std::string&, std::optional<double>>(),
           py::arg("config_path"), py::arg("damping") = std::nullopt)
      .def("update_joint_state", &hc::v23::ControllerV23::update_joint_state)
      .def("update_tf_targets", &hc::v23::ControllerV23::update_tf_targets)
      .def("reset_to_feedback", &hc::v23::ControllerV23::reset_to_feedback)
      .def("step",
           [](hc::v23::ControllerV23& self, std::optional<double> dt) {
             py::gil_scoped_release release;
             return self.step(dt);
           },
           py::arg("dt") = std::nullopt)
      .def("relative_pose",
           [](hc::v23::ControllerV23& self, const std::string& root,
              const std::string& child,
              std::optional<Eigen::VectorXd> q) -> std::pair<Eigen::Vector3d, Eigen::Matrix3d> {
             const Eigen::VectorXd& use_q = q.has_value() ? *q : self.q();
             const auto se3 = self.interface().relative_pose(root, child, use_q);
             return {se3.translation(), se3.rotation()};
           },
           py::arg("root"), py::arg("child"), py::arg("q") = std::nullopt)
      .def("relative_jacobian",
           [](hc::v23::ControllerV23& self, const std::string& root,
              const std::string& child, std::optional<Eigen::VectorXd> q) {
             const Eigen::VectorXd& use_q = q.has_value() ? *q : self.q();
             return self.interface().relative_jacobian(root, child, use_q);
           },
           py::arg("root"), py::arg("child"), py::arg("q") = std::nullopt)
      .def("stack_tasks",
           [](hc::v23::ControllerV23& self, std::optional<Eigen::VectorXd> q) {
             const Eigen::VectorXd& use_q = q.has_value() ? *q : self.q();
             return self.stack_tasks(use_q);
           },
           py::arg("q") = std::nullopt)
      .def("free_configuration",
           [](const hc::v23::ControllerV23& self,
              std::optional<Eigen::VectorXd> q) {
             return q.has_value() ? self.interface().free_configuration(*q)
                                  : self.interface().free_configuration(self.q());
           },
           py::arg("q") = std::nullopt)
      .def_property_readonly("free_joint_names",
                             &hc::v23::ControllerV23::free_joint_names)
      .def_property_readonly("q", &hc::v23::ControllerV23::q)
      .def_property(
          "command_q",
          [](const hc::v23::ControllerV23& self) { return self.command_q(); },
          [](hc::v23::ControllerV23& self, const Eigen::VectorXd& v) {
            self.command_q() = v;
          })
      .def_property(
          "method", &hc::v23::ControllerV23::method,
          &hc::v23::ControllerV23::set_method)
      .def_property_readonly("dt", &hc::v23::ControllerV23::dt)
      .def_property_readonly("velocity_limits",
                             &hc::v23::ControllerV23::velocity_limits)
      .def_property_readonly("last_result",
                             &hc::v23::ControllerV23::last_result)
      .def_property_readonly("pose_task_keys", [](const hc::v23::ControllerV23& self) {
        std::vector<std::pair<std::string, std::string>> keys;
        for (const auto& t : self.pose_tasks()) {
          keys.emplace_back(t.root, t.frame);
        }
        return keys;
      });
}

