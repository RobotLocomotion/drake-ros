#include <pybind11/pybind11.h>

#include "drake/geometry/geometry_ids.h"
#include "drake_ros/drake_ros_pybind.h"

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

void DefCore(py::module m);
void DefTf2(py::module m);
void DefViz(py::module m);

namespace {

void DefTest(py::module m) {
  // The following is used for `odr_py_test.py`. We purposefully use a function
  // whose "linkage" may have state.
  // In this case, `get_new_id()` will effectively function as a counter
  // whose state *should* come from `drake_shared_library`.
  m.def("NextGeometryId", []() {
    return drake::geometry::GeometryId::get_new_id();
  });
}

PYBIND11_MODULE(_cc, m) {
  m.doc() = "Python bindings for drake_ros";

  DefTest(m.def_submodule("_test"));
  DefCore(m.def_submodule("core"));
  DefTf2(m.def_submodule("tf2"));
  DefViz(m.def_submodule("viz"));
}

}  // namespace
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
