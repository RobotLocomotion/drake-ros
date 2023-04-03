#include "geometry_msgs/msg/polygon.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake_ros/core/drake_ros.h"
#include "drake_ros/core/ros_idl_pybind.h"
#include "drake_ros/drake_ros_pybind.h"

// This is meant to be a test module, to just test
// the generic typecaster macro.
namespace PYBIND11_NAMESPACE {
namespace detail {
ROS_MSG_PYBIND_TYPECAST_ALL();
}
}  // namespace PYBIND11_NAMESPACE

namespace drake_ros {
namespace mock_test {
template <typename T>
T testTypecasting(T msg) {
  std::cout << "Message type: " << rosidl_generator_traits::name<T>()
            << std::endl;
  return msg;
}
}  // namespace mock_test
}  // namespace drake_ros

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

using drake_ros_core::DrakeRos;

void DefTestTypecasting(py::module m) {
  m.doc() = "Python bindings for drake_ros.mock";

  py::class_<DrakeRos>(m, "DrakeRos");

  m.def("testTypecasting",
        &drake_ros::mock_test::testTypecasting<geometry_msgs::msg::Polygon>,
        py::arg("msg"));
  m.def("testTypecasting",
        &drake_ros::mock_test::testTypecasting<geometry_msgs::msg::Quaternion>,
        py::arg("msg"));
}
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
