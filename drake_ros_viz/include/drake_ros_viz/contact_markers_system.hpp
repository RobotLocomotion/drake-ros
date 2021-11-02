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
#ifndef DRAKE_ROS_SYSTEMS__CONTACT_MARKERS_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__CONTACT_MARKERS_SYSTEM_HPP_

#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/rgba.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>  // NOLINT(build/include_order)
#include <string>


namespace drake_ros_systems
{

/// Set of parameters that configure a ContactMarkersSystem.
struct ContactMarkersParams
{
  /// Configure ContactMarkersSystem to depict strict hydroelastic collisions.
  static ContactMarkersParams Strict() {return {};}

  /// Configure ContactMarkersSystem to depict collisions.
  static ContactMarkersParams NoStrict()
  {
    ContactMarkersParams params;
    params.use_strict_hydro_ = false;
    return params;
  }

  /// Optional namespace for all markers, defaults
  /// to utilities::GetMarkerNamespace() outcome.
  std::optional<std::string> marker_namespace{std::nullopt};

  /// Origin Frame Name
  std::string origin_frame_name{"world"};

  /// Default marker color if no ("phong", "diffuse") property is found.
  drake::geometry::Rgba default_color{0.6, 1.0, 0.6, 1.0};

  /// Use strict hydroelastic collisions.
  bool use_strict_hydro_{true};
};

/// System for SceneGraph depiction as a ROS markers array.
///
/// This system outputs a `visualization_msgs/msg/MarkerArray` populated with
/// all geometries found in a SceneGraph, using Context time to timestamp
/// each `visualization_msgs/msg/Marker` message.
///
/// It has one input port:
/// - *graph_query* (abstract): expects a QueryObject from the SceneGraph.
///
/// It has one output port:
/// - *contact_markers* (abstract): all scene geometries, as a
///   visualization_msg::msg::MarkerArray message.
///
/// This system provides the same base functionality in terms of SceneGraph
/// geometries lookup and message conversion for ROS-based applications as
/// the DrakeVisualizer system does for LCM-based applications.
/// Thus, discussions in DrakeVisualizer's documentation about geometry
/// versioning, geometry roles, and so on are equally applicable here.
class ContactMarkersSystem : public drake::systems::LeafSystem<double>
{
public:
  explicit ContactMarkersSystem(ContactMarkersParams params = {});
  virtual ~ContactMarkersSystem();

  /// Register a MultibodyPlant present in the scene
  /**
   * This provides the system with additional information to generate
   * semantically meaningful frame string IDs and marker namespaces.
   */
  void
  RegisterMultibodyPlant(
    const drake::multibody::MultibodyPlant<double> * plant);

  const ContactMarkersParams & params() const;

  const drake::systems::InputPort<double> & get_graph_query_port() const;

  const drake::systems::OutputPort<double> & get_markers_output_port() const;

private:
  // Inspects the SceneGraph and carries out the conversion
  // to visualization_msgs::msg::MarkerArray message unconditionally.
  void
  CalcContactMarkers(
    const drake::systems::Context<double> & context,
    visualization_msgs::msg::MarkerArray * output_value) const;

  // PIMPL forward declaration
  class ContactMarkersSystemPrivate;

  std::unique_ptr<ContactMarkersSystemPrivate> impl_;
};

}  // namespace drake_ros_systems

#endif  // DRAKE_ROS_SYSTEMS__CONTACT_MARKERS_SYSTEM_HPP_
