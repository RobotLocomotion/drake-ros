// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <drake/systems/framework/leaf_system.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <unordered_set>

#include "drake_ros_systems/drake_ros.hpp"
#include "drake_ros_systems/ros_interface_system.hpp"
#include "drake_ros_systems/ros_publisher_system.hpp"
#include "drake_ros_systems/ros_subscriber_system.hpp"
#include "drake_ros_systems/rviz_visualizer.hpp"
#include "drake_ros_systems/scene_markers_system.hpp"
#include "drake_ros_systems/tf_broadcaster_system.hpp"

#include "py_serializer.hpp"
#include "qos_type_caster.hpp"

namespace py = pybind11;

using drake::systems::Diagram;
using drake::systems::LeafSystem;
using drake::systems::TriggerType;

using drake_ros_systems::DrakeRos;
using drake_ros_systems::DrakeRosInterface;
using drake_ros_systems::PySerializer;
using drake_ros_systems::RosInterfaceSystem;
using drake_ros_systems::RosPublisherSystem;
using drake_ros_systems::RosSubscriberSystem;
using drake_ros_systems::RvizVisualizer;
using drake_ros_systems::SceneMarkersSystem;
using drake_ros_systems::SerializerInterface;
using drake_ros_systems::TfBroadcasterSystem;


PYBIND11_MODULE(drake_ros_systems, m) {
  m.doc() = "Python wrapper for drake_ros_systems";

  py::module::import("pydrake.systems.framework");

  // Use std::shared_ptr holder so pybind11 doesn't try to delete interfaces returned from
  // get_ros_interface
  py::class_<DrakeRosInterface, std::shared_ptr<DrakeRosInterface>>(m, "DrakeRosInterface");

  py::class_<RosInterfaceSystem, LeafSystem<double>>(m, "RosInterfaceSystem")
  .def(
    py::init(
      []() {return std::make_unique<RosInterfaceSystem>(std::make_unique<DrakeRos>());}))
  .def("get_ros_interface", &RosInterfaceSystem::get_ros_interface);

  py::class_<RosPublisherSystem, LeafSystem<double>>(m, "RosPublisherSystem")
  .def(
    py::init(
      [](
        pybind11::object type,
        const char * topic_name,
        drake_ros_systems::QoS qos,
        std::shared_ptr<DrakeRosInterface> ros_interface)
      {
        std::unique_ptr<SerializerInterface> serializer = std::make_unique<PySerializer>(type);
        return std::make_unique<RosPublisherSystem>(
          serializer,
          topic_name,
          qos,
          ros_interface,
          std::unordered_set<TriggerType>{TriggerType::kPerStep, TriggerType::kForced},
          0.0);
      }))
  .def(
    py::init(
      [](
        pybind11::object type,
        const char * topic_name,
        drake_ros_systems::QoS qos,
        std::shared_ptr<DrakeRosInterface> ros_interface,
        std::unordered_set<TriggerType> publish_triggers,
        double publish_period)
      {
        std::unique_ptr<SerializerInterface> serializer = std::make_unique<PySerializer>(type);
        return std::make_unique<RosPublisherSystem>(
          serializer,
          topic_name,
          qos,
          ros_interface,
          publish_triggers,
          publish_period);
      }));

  py::class_<RosSubscriberSystem, LeafSystem<double>>(m, "RosSubscriberSystem")
  .def(
    py::init(
      [](
        pybind11::object type,
        const char * topic_name,
        drake_ros_systems::QoS qos,
        std::shared_ptr<DrakeRosInterface> ros_interface)
      {
        std::unique_ptr<SerializerInterface> serializer = std::make_unique<PySerializer>(type);
        return std::make_unique<RosSubscriberSystem>(
          serializer,
          topic_name,
          qos,
          ros_interface);
      }));

  py::class_<TfBroadcasterSystem, LeafSystem<double>>(m, "TfBroadcasterSystem")
  .def(
    py::init(
      [](DrakeRosInterface * ros_interface)
      {
        constexpr double kZeroPublishPeriod{0.0};
        return std::make_unique<TfBroadcasterSystem>(
          ros_interface,
          std::unordered_set<TriggerType>{
            TriggerType::kPerStep, TriggerType::kForced},
          kZeroPublishPeriod);
      }))
  .def(
    py::init(
      [](
        DrakeRosInterface * ros_interface,
        std::unordered_set<TriggerType> publish_triggers,
        double publish_period)
      {
        return std::make_unique<TfBroadcasterSystem>(
          ros_interface, publish_triggers, publish_period);
      }));

  py::class_<SceneMarkersSystem, LeafSystem<double>>(m, "SceneMarkersSystem")
  .def(py::init([]() { return std::make_unique<SceneMarkersSystem>(); }))
  .def(
    py::init(
      [](const drake::geometry::Role & role,
         const drake::geometry::Rgba & default_color)
      {
        return std::make_unique<SceneMarkersSystem>(role, default_color);
      }));

  py::class_<RvizVisualizer, Diagram<double>>(m, "RvizVisualizer")
  .def(
    py::init(
      [](std::shared_ptr<DrakeRosInterface> ros_interface)
      {
        return std::make_unique<RvizVisualizer>(ros_interface);
      }))
  .def(
    py::init(
      [](std::shared_ptr<DrakeRosInterface> ros_interface,
         std::unordered_set<TriggerType> publish_triggers,
         double publish_period, bool publish_tf)
      {
        return std::make_unique<RvizVisualizer>(
          ros_interface, publish_triggers, publish_period, publish_tf);
      }));
}
