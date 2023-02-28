#include <pybind11/pybind11.h>

#include "drake/geometry/geometry_ids.h"

namespace py = pybind11;

namespace drake_ros {
namespace {

// We purposefully use a function whose "linkage" may have state.
// In this case, `get_new_id()` will effectively function as a counter
// whose state *should* come from `drake_shared_library`.
drake::geometry::GeometryId NextGeometryId() {
  return drake::geometry::GeometryId::get_new_id();
}

PYBIND11_MODULE(odr_py_helpers, m) {
  py::module::import("pydrake.geometry");
  m.def("NextGeometryId", &NextGeometryId);
}

}  // namespace
}  // namespace drake_ros
