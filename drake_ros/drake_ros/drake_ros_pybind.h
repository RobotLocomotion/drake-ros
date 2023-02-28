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

// TODO(eric.cousineau): Fix or work around ament clang_format's failures.
}  // namespace DRAKE_ROS_NO_EXPORT
}  // namespace drake_ros
