#include <pybind11/pybind11.h>

#include "drake_ros/drake_ros_pybind.h"

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

void DefTestTypecasting(py::module m);

namespace {
PYBIND11_MODULE(_cc_generic_typecaster, m) {
  m.doc() = "Mock python bindings for drake_ros";

  DefTestTypecasting(m);
}
}  // namespace
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
