#include "geometry_msgs/msg/polygon.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake_ros/core/ros_idl_pybind.h"
#include "drake_ros/drake_ros_pybind.h"

// This is meant to be a test module, to just test
// the generic typecaster macro.
ROS_MSG_PYBIND_TYPECAST_ALL();

namespace drake_ros {
namespace drake_ros_py {
namespace {

template <typename T>
T TestTypecastingViaAllMacro(T msg) {
  return msg;
}

PYBIND11_MODULE(ros_message_type_caster_test_via_all, m) {
  m.def("TestTypecastingViaAllMacro",
        &TestTypecastingViaAllMacro<geometry_msgs::msg::Polygon>,
        py::arg("msg"));
  m.def("TestTypecastingViaAllMacro",
        &TestTypecastingViaAllMacro<geometry_msgs::msg::Quaternion>,
        py::arg("msg"));
}

}  // namespace
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
