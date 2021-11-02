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

#include <drake/common/drake_copyable.h>
#include <drake/common/eigen_types.h>
#include <drake/geometry/geometry_properties.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/leaf_system.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

#include <iostream>

#include "drake_ros_systems/contact_markers_system.hpp"
#include "drake_ros_systems/utilities/name_conventions.hpp"
#include "drake_ros_systems/utilities/type_conversion.hpp"
#include "lodepng/lodepng.h"

namespace drake_ros_systems
{

namespace
{

double calc_uv(double pressure, double min_pressure, double max_pressure) {
  double u = ((pressure - min_pressure) / (max_pressure - min_pressure));
  return std::clamp(u, 0.0, 1.0);
}

void create_color(double value, double & red, double & green, double & blue) {
  red = std::clamp(((value - 0.25) * 4.0), 0.0, 1.0);
  green = std::clamp(((value - 0.5) * 4.0), 0.0, 1.0);
  if (value < 0.25) {
    blue = std::clamp(value * 4.0, 0.0, 1.0);
  }
  else if (value > 0.75) {
    blue = std::clamp((value - 0.75) * 4.0, 0.0, 1.0);
  }
  else {
    blue = std::clamp(1.0 - (value - 0.25) * 4.0, 0.0, 1.0);
  }
}

class ContactGeometryToMarkers : public drake::geometry::ShapeReifier
{
private:
  const size_t TEXTURE_SIZE = 1024;

  const ContactMarkersParams & params_;
  visualization_msgs::msg::MarkerArray * marker_array_{nullptr};
  drake::math::RigidTransform<double> X_FG_{};
  std::vector<uint8_t> texture_;

public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactGeometryToMarkers)

  explicit ContactGeometryToMarkers(const ContactMarkersParams & params)
  : params_(params)
  {
    std::vector<uint8_t> image(TEXTURE_SIZE * 4);
    double red, green, blue;
    for (size_t i = 0; i < TEXTURE_SIZE; i++) {
      create_color((double) i / TEXTURE_SIZE, red, green, blue);
      image[(i*4) + 0] = (uint8_t)(red * 255.0);
      image[(i*4) + 1] = (uint8_t)(green * 255.0);
      image[(i*4) + 2] = (uint8_t)(blue * 255.0);
      image[(i*4) + 3] = (uint8_t)(255);
    }
    lodepng::encode(texture_, image, TEXTURE_SIZE, 1);
  }

  ~ContactGeometryToMarkers() override = default;

  void Populate(
    const std::vector<drake::geometry::ContactSurface<double>> & surfaces,
    const std::vector<drake::geometry::PenetrationAsPointPair<double>> & points,
    visualization_msgs::msg::MarkerArray * marker_array)
  {
    DRAKE_ASSERT(nullptr != marker_array);
    marker_array_ = marker_array;

    const int num_surfaces = static_cast<int>(surfaces.size());
    const int num_pairs = static_cast<int>(points.size());

    marker_array_->markers.reserve(num_surfaces + num_pairs);

    // Translate Drake Contact Surface into ROS List of Triangles.
    int name_index = 0;
    for (const drake::geometry::ContactSurface<double>& surface: surfaces) {
      visualization_msgs::msg::Marker face_msg;
      face_msg.header.frame_id = params_.origin_frame_name;
      face_msg.ns = std::to_string(name_index) + "_faces";
      face_msg.id = marker_array_->markers.size();
      face_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      face_msg.action = visualization_msgs::msg::Marker::ADD;

      face_msg.lifetime = rclcpp::Duration::from_nanoseconds(0);
      face_msg.frame_locked = true;

      drake::geometry::Rgba color = params_.default_color;
      face_msg.color.r = color.r();
      face_msg.color.g = color.g();
      face_msg.color.b = color.b();
      face_msg.color.a = color.a();

      face_msg.scale.x = 1.0;
      face_msg.scale.y = 1.0;
      face_msg.scale.z = 1.0;

      const drake::geometry::SurfaceMesh<double>& mesh_W = surface.mesh_W();
      face_msg.points.clear();
      face_msg.points.resize(mesh_W.num_faces() * 3);
      face_msg.colors.clear();
      face_msg.colors.resize(mesh_W.num_faces() * 3);
      face_msg.uv_coordinates.clear();
      face_msg.uv_coordinates.resize(mesh_W.num_faces() * 3);

      Eigen::VectorXd pressures;
      pressures.resize(mesh_W.num_faces() * 3);

      // Make lines for the edges
      visualization_msgs::msg::Marker edge_msg;
      edge_msg.header.frame_id = params_.origin_frame_name;
      edge_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
      edge_msg.action = visualization_msgs::msg::Marker::ADD;
      edge_msg.lifetime = rclcpp::Duration::from_nanoseconds(0);
      edge_msg.frame_locked = true;
      edge_msg.ns = std::to_string(name_index++) + "_edges";
      edge_msg.id = 1;
      // Set the size of the individual markers (depends on scale)
      //edge_msg.scale = ToScale(edge_scale * scale);
      edge_msg.scale.x = 0.01;
      edge_msg.scale.y = 0.01;
      edge_msg.scale.z = 1.0;

      // Set the edge color
      edge_msg.color.r = 1.0;
      edge_msg.color.g = 1.0;
      edge_msg.color.b = 1.0;
      edge_msg.color.a = 1.0;

      // Generate the surface markers for each mesh.
      size_t index = 0;
      const auto& field = surface.e_MN();
      for (drake::geometry::SurfaceFaceIndex j(0); j < mesh_W.num_faces(); ++j) {
        // Get the three vertices.
        const auto& face = mesh_W.element(j);
        const drake::geometry::SurfaceVertex<double>& vA = mesh_W.vertex(face.vertex(0));
        const drake::geometry::SurfaceVertex<double>& vB = mesh_W.vertex(face.vertex(1));
        const drake::geometry::SurfaceVertex<double>& vC = mesh_W.vertex(face.vertex(2));

        face_msg.points.at(index + 0) = tf2::toMsg(vA.r_MV());
        face_msg.points.at(index + 1) = tf2::toMsg(vB.r_MV());
        face_msg.points.at(index + 2) = tf2::toMsg(vC.r_MV());

        for (size_t vert_index = 0; vert_index < 3; vert_index++) {
          pressures[index + vert_index] =
            field.EvaluateAtVertex(face.vertex(vert_index));
        }

        // 0->1
        edge_msg.points.push_back(tf2::toMsg(vA.r_MV()));
        edge_msg.points.push_back(tf2::toMsg(vB.r_MV()));
        // 1->2
        edge_msg.points.push_back(tf2::toMsg(vB.r_MV()));
        edge_msg.points.push_back(tf2::toMsg(vC.r_MV()));
        // 2->0
        edge_msg.points.push_back(tf2::toMsg(vC.r_MV()));
        edge_msg.points.push_back(tf2::toMsg(vA.r_MV()));

        index += 3;
      }

      // Color based on pressures.
      for (size_t tri_index = 0; tri_index < (size_t)mesh_W.num_faces(); tri_index++) {
        for (size_t vert_index = 0; vert_index < 3; vert_index++) {
          size_t arr_index = (tri_index * 3) + vert_index;
          double norm_data = calc_uv(
              pressures(arr_index), 0.0, pressures.maxCoeff());

          face_msg.colors.at(arr_index).r = 1.0;
          face_msg.colors.at(arr_index).g = 1.0;
          face_msg.colors.at(arr_index).b = 1.0;
          face_msg.colors.at(arr_index).a = 1.0;
          face_msg.uv_coordinates.at(arr_index).u = norm_data;
          face_msg.uv_coordinates.at(arr_index).v = 0;
        }
      }

      face_msg.texture.data = texture_;
      face_msg.texture_resource = "embedded://heat_map.png";
      face_msg.texture.format = "png";

      marker_array_->markers.push_back(face_msg);
      marker_array_->markers.push_back(edge_msg);
    }
  }
};

}  // namespace

class ContactMarkersSystem::ContactMarkersSystemPrivate
{
public:
  explicit ContactMarkersSystemPrivate(ContactMarkersParams _params)
  : params(std::move(_params))
  {
  }

  const ContactMarkersParams params;
  drake::systems::InputPortIndex graph_query_port_index;
  drake::systems::OutputPortIndex contact_markers_port_index;
  std::unordered_set<const drake::multibody::MultibodyPlant<double> *> plants;
  mutable drake::geometry::GeometryVersion version;
};

ContactMarkersSystem::ContactMarkersSystem(ContactMarkersParams params)
: impl_(new ContactMarkersSystemPrivate(std::move(params)))
{
  impl_->graph_query_port_index =
    this->DeclareAbstractInputPort(
    "graph_query", drake::Value<drake::geometry::QueryObject<double>>{}).get_index();

  impl_->contact_markers_port_index = this->DeclareAbstractOutputPort(
    "contact_markers", &ContactMarkersSystem::CalcContactMarkers).get_index();
}

ContactMarkersSystem::~ContactMarkersSystem()
{
}

void
ContactMarkersSystem::RegisterMultibodyPlant(
  const drake::multibody::MultibodyPlant<double> * plant)
{
  DRAKE_THROW_UNLESS(plant != nullptr);
  impl_->plants.insert(plant);
}

namespace
{

visualization_msgs::msg::Marker MakeDeleteAllMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace

void
ContactMarkersSystem::CalcContactMarkers(
  const drake::systems::Context<double> & context,
  visualization_msgs::msg::MarkerArray * output_value) const
{
  output_value->markers.clear();
  output_value->markers.insert(
    output_value->markers.begin(),
    MakeDeleteAllMarker());

  const auto& query_object =
    get_input_port(impl_->graph_query_port_index)
    .Eval<drake::geometry::QueryObject<double>>(context);

  std::vector<drake::geometry::ContactSurface<double>> surfaces;
  std::vector<drake::geometry::PenetrationAsPointPair<double>> points;

  if (impl_->params.use_strict_hydro_) {
    surfaces = query_object.ComputeContactSurfaces();
  } else {
    query_object.ComputeContactSurfacesWithFallback(&surfaces, &points);
  }

  ContactGeometryToMarkers(impl_->params).Populate(
    surfaces, points, output_value);
}

const ContactMarkersParams &
ContactMarkersSystem::params() const
{
  return impl_->params;
}

const drake::systems::InputPort<double> &
ContactMarkersSystem::get_graph_query_port() const
{
  return get_input_port(impl_->graph_query_port_index);
}

const drake::systems::OutputPort<double> &
ContactMarkersSystem::get_markers_output_port() const
{
  return get_output_port(impl_->contact_markers_port_index);
}

}  // namespace drake_ros_systems
