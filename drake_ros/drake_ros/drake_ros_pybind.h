#pragma once

// Provides centralized pybind11 alias and utilities for use in headers for
// this project.

#include <pybind11/pybind11.h>

/** DRAKE_ROS_NO_EXPORT sets C++ code to use hidden linker visibility.

For more details, see:
https://github.com/RobotLocomotion/drake/blob/v1.13.0/common/drake_export.h
*/
#define DRAKE_ROS_NO_EXPORT __attribute__((visibility("hidden")))

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

namespace py = pybind11;

/**
Imported from pydrake_pybind.h
*/
template <typename Class>
auto ParamInit() {
  return py::init([](py::kwargs kwargs) {
    Class obj{};
    py::object py_obj = py::cast(&obj, py::return_value_policy::reference);
    py::module m = py::module::import("drake_ros.core");
    m.attr("_setattr_kwargs")(py_obj, kwargs);
    return obj;
  });
}

// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
