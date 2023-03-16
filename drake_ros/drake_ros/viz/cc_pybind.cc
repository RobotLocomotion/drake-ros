#include <memory>

#include <drake/systems/framework/diagram.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake_ros/core/drake_ros.h"
#include "drake_ros/drake_ros_pybind.h"
#include "drake_ros/viz/rviz_visualizer.h"

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

using drake_ros::core::DrakeRos;
using drake_ros::viz::RvizVisualizer;
using drake_ros::viz::RvizVisualizerParams;

using drake::systems::Diagram;
using drake::systems::TriggerType;

void DefViz(py::module m) {
  m.doc() = "Python wrapper for drake_ros_viz";

  py::module::import("drake_ros.core");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.multibody.plant");

  const RvizVisualizerParams default_params{};
  py::class_<RvizVisualizerParams>(m, "RvizVisualizerParams")
      .def(py::init([](const std::unordered_set<drake::systems::TriggerType>&
                           publish_triggers,
                       double publish_period, bool publish_tf) {
             return RvizVisualizerParams{publish_triggers, publish_period,
                                         publish_tf};
           }),
           py::kw_only(),
           py::arg("publish_triggers") = default_params.publish_triggers,
           py::arg("publish_period") = default_params.publish_period,
           py::arg("publish_tf") = default_params.publish_tf)
      .def_readwrite("publish_triggers",
                     &RvizVisualizerParams::publish_triggers)
      .def_readwrite("publish_period", &RvizVisualizerParams::publish_period)
      .def_readwrite("publish_tf", &RvizVisualizerParams::publish_tf);

  py::class_<RvizVisualizer, Diagram<double>>(m, "RvizVisualizer")
      .def(py::init<DrakeRos*, RvizVisualizerParams>(), py::arg("ros"),
           py::arg("params") = RvizVisualizerParams{})
      .def("RegisterMultibodyPlant", &RvizVisualizer::RegisterMultibodyPlant)
      .def("ComputeFrameHierarchy", &RvizVisualizer::ComputeFrameHierarchy)
      .def("get_graph_query_input_port",
           &RvizVisualizer::get_graph_query_input_port,
           py::return_value_policy::reference_internal);
}

// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
