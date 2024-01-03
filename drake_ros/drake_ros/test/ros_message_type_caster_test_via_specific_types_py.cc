#include "geometry_msgs/msg/polygon.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake_ros/core/ros_idl_pybind.h"
#include "drake_ros/drake_ros_pybind.h"

// This is meant to be a test module, to just test
// the typecaster macro.
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Quaternion);
// N.B. We explicitly do not add a typecaster for Polygon.

namespace drake_ros {
namespace drake_ros_py {
namespace {

template <typename T>
T TestTypecastingViaSpecificMacro(T msg) {
  return msg;
}

PYBIND11_MODULE(ros_message_type_caster_test_via_specific_types, m) {
  m.def("TestTypecastingViaSpecificMacro",
        &TestTypecastingViaSpecificMacro<geometry_msgs::msg::Quaternion>,
        py::arg("msg"));
  m.def("TestTypecastingViaSpecificMacro",
        &TestTypecastingViaSpecificMacro<geometry_msgs::msg::Polygon>,
        py::arg("msg"));
}

}  // namespace
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
