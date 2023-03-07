#include <pybind11/pybind11.h>

#include "drake_ros/drake_ros_pybind.h"

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

void DefCore(py::module m);
void DefTf2(py::module m);
void DefViz(py::module m);

namespace {

PYBIND11_MODULE(_cc, m) {
  m.doc() = "Python bindings for drake_ros";

  DefCore(m.def_submodule("core"));
  DefTf2(m.def_submodule("tf2"));
  DefViz(m.def_submodule("viz"));
}

}  // namespace
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
