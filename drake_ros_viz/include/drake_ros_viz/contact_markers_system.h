// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#pragma once

#include <memory>
#include <optional>  // NOLINT(build/include_order)
#include <string>

#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/rgba.h>
#include "drake/multibody/plant/contact_results.h"
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake_ros_core/drake_ros.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace drake_ros_viz {

/// Set of parameters that configure a ContactMarkersSystem.
struct ContactMarkersParams {
  /// Origin Frame Name
  std::string origin_frame_name{"world"};

  /// Default marker color if no ("phong", "diffuse") property is found.
  drake::geometry::Rgba default_color{0.6, 1.0, 0.6, 0.35};
};

/// System for visualizing contacts as a ROS markers array.
///
/// This system outputs a `visualization_msgs/msg/MarkerArray` populated with
/// markers for all contacts found in a SceneGraph, using Context time for
/// each `visualization_msgs/msg/Marker` message.
///
/// It has one input port:
/// - *contact_results* (abstract): expects contact results from a
///   MultibodyPlant.
///
/// It has one output port:
/// - *contact_markers* (abstract): all scene geometries, as a
///   visualization_msg::msg::MarkerArray message.
class ContactMarkersSystem : public drake::systems::LeafSystem<double> {
 public:
  ContactMarkersSystem(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::SceneGraph<double>& scene_graph,
    ContactMarkersParams params = {});
  virtual ~ContactMarkersSystem();

  const ContactMarkersParams& params() const;

  const drake::systems::InputPort<double>& get_contact_results_port() const;

  const drake::systems::OutputPort<double>& get_markers_output_port() const;

 private:
  void CalcContactMarkers(
      const drake::systems::Context<double>& context,
      visualization_msgs::msg::MarkerArray* output_value) const;

  // PIMPL forward declaration
  class ContactMarkersSystemPrivate;

  std::unique_ptr<ContactMarkersSystemPrivate> impl_;
};

/// Publish contacts from a multibody plant for visualization in RViz.
///
/// @param builder The diagram builder this method should add systems to.
/// @param plant The multibody plant whose contacts are to be visualized.
/// @param scene_graph The scene graph to query for geometry names.
/// @param ros A DrakeROS instance to use to create ROS publishers.
/// @param params Parameters to control how contacts are visualized.
/// @param markers_topic The name of a ROS topic to publish contact markers to.
/// @param markers_qos The QoS settings to set on the ROS publisher for the
///  contact markers topic.
/// @returns A created ContactMarkersSystem which has been added to the builder.
ContactMarkersSystem * ConnectContactResultsToRviz(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::SceneGraph<double>& scene_graph,
    drake_ros_core::DrakeRos* ros,
    ContactMarkersParams params = {},
    const std::string & markers_topic = "/contacts",
    const rclcpp::QoS & markers_qos = rclcpp::QoS(1));
}  // namespace drake_ros_viz
