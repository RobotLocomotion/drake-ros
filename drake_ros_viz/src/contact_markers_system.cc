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

#include "drake_ros_viz/contact_markers_system.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <utility>
#include <vector>

#include "lodepng/lodepng.h"
#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
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
#include <drake_ros_core/drake_ros.h>
#include <drake_ros_core/ros_publisher_system.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "drake_ros_viz/utilities/name_conventions.h"
#include "drake_ros_viz/utilities/type_conversion.h"

namespace drake_ros_viz {

namespace {
// Copied from:
// https://github.com/RobotLocomotion/drake/blob/
// c246c0d4480a5b4cc2cdc07cfda9aabe6b25b9a1/
// multibody/plant/contact_results_to_lcm.h#L43-L49
struct FullBodyName {
  std::string model;
  std::string body;
  std::string geometry;
};
// End copied code

std::string contact_name(const std::string& name1, const std::string& name2) {
  // Sort so names are consistent
  if (name2 < name1) {
    return name2 + "//" + name1;
  }
  return name1 + "//" + name2;
}

std::string contact_name(const FullBodyName& name1, const FullBodyName& name2) {
  auto make_full_name = [](const FullBodyName& name) -> std::string {
    std::stringstream full_name;
    if (!name.model.empty()) {
      full_name << name.model << "::";
    }
    if (!name.body.empty()) {
      full_name << name.body << "::";
    }
    if (!name.geometry.empty()) {
      full_name << name.geometry;
    }
    return full_name.str();
  };

  return contact_name(make_full_name(name1), make_full_name(name2));
}

double calc_uv(double pressure, double min_pressure, double max_pressure) {
  double u = ((pressure - min_pressure) / (max_pressure - min_pressure));
  return std::clamp(u, 0.0, 1.0);
}

void create_color(double value, double& red, double& green, double& blue) {
  red = std::clamp(((value - 0.25) * 4.0), 0.0, 1.0);
  green = std::clamp(((value - 0.5) * 4.0), 0.0, 1.0);
  if (value < 0.25) {
    blue = std::clamp(value * 4.0, 0.0, 1.0);
  } else if (value > 0.75) {
    blue = std::clamp((value - 0.75) * 4.0, 0.0, 1.0);
  } else {
    blue = std::clamp(1.0 - (value - 0.25) * 4.0, 0.0, 1.0);
  }
}
}  // namespace

class ContactMarkersSystem::ContactMarkersSystemPrivate {
 public:
  explicit ContactMarkersSystemPrivate(ContactMarkersParams _params)
      : params(std::move(_params)) {}

  const ContactMarkersParams params;
  drake::systems::InputPortIndex contact_results_port_index;
  drake::systems::OutputPortIndex contact_markers_port_index;

  std::vector<uint8_t> texture;

  // A mapping from geometry IDs to per-body name data.
  std::unordered_map<drake::geometry::GeometryId, FullBodyName>
      geometry_id_to_body_name_map;

  // A mapping from body index values to body names.
  std::vector<std::string> body_names;
};

ContactMarkersSystem::ContactMarkersSystem(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::SceneGraph<double>& scene_graph,
    ContactMarkersParams params)
    : impl_(new ContactMarkersSystemPrivate(std::move(params))) {
  impl_->contact_results_port_index =
      this->DeclareAbstractInputPort(
              drake::systems::kUseDefaultName,
              drake::Value<drake::multibody::ContactResults<double>>())
          .get_index();

  impl_->contact_markers_port_index =
      this->DeclareAbstractOutputPort("contact_markers",
                                      &ContactMarkersSystem::CalcContactMarkers)
          .get_index();

  // Make a heatmap texture
  const size_t kTextureSize = 1024;
  std::vector<uint8_t> image(kTextureSize * 4);
  double red, green, blue;
  for (size_t i = 0; i < kTextureSize; i++) {
    create_color(static_cast<double>(i) / kTextureSize, red, green, blue);
    image[(i * 4) + 0] = (uint8_t)(red * 255.0);
    image[(i * 4) + 1] = (uint8_t)(green * 255.0);
    image[(i * 4) + 2] = (uint8_t)(blue * 255.0);
    image[(i * 4) + 3] = (uint8_t)(255);
  }
  lodepng::encode(impl_->texture, image, kTextureSize, 1);

  // Mostly Copied from:
  // https://github.com/RobotLocomotion/drake/blob/
  // 8994f6809fb86d23438c3456ba086eebc737864d/
  // multibody/plant/contact_results_to_lcm.cc#L87-L120
  const int body_count = plant.num_bodies();

  impl_->body_names.reserve(body_count);
  const drake::geometry::SceneGraphInspector<double>& inspector =
      scene_graph.model_inspector();
  for (drake::multibody::BodyIndex i{0}; i < body_count; ++i) {
    const drake::multibody::Body<double>& body = plant.get_body(i);
    impl_->body_names.push_back(body.name() + "(" +
                                std::to_string(body.model_instance()) + ")");
    for (auto geometry_id : plant.GetCollisionGeometriesForBody(body)) {
      const std::string& model_name =
          plant.GetModelInstanceName(body.model_instance());
      // TODO(SeanCurtis-TRI): collision geometries can be added to SceneGraph
      //  after the plant has been finalized. Those geometries will not be found
      //  in this map. What *should* happen is that this should *also* be
      //  connected to SceneGraph's query object output port and it should ask
      //  scene graph about things like this when evaluating the output port.
      //  However, this is not an immediate problem for *this* system, because
      //  MultibodyPlant is authored such that if someone were to add such a
      //  geometry and it participated in collision, MultibodyPlant would have
      //  already thrown an exception in computing the contact. Until MbP gets
      //  out of the way, there's no reason to update here.
      impl_->geometry_id_to_body_name_map[geometry_id] = {
          model_name, body.name(), inspector.GetName(geometry_id)};
    }
  }
  // End copied code
}

ContactMarkersSystem::~ContactMarkersSystem() {}

namespace {
visualization_msgs::msg::Marker MakeDeleteAllMarker() {
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}
}  // namespace

void ContactMarkersSystem::CalcContactMarkers(
    const drake::systems::Context<double>& context,
    visualization_msgs::msg::MarkerArray* output_value) const {
  output_value->markers.clear();
  output_value->markers.insert(output_value->markers.begin(),
                               MakeDeleteAllMarker());
  const auto& contact_results =
      get_contact_results_port()
          .template Eval<drake::multibody::ContactResults<double>>(context);

  const rclcpp::Duration kMarkerLifetime =
      rclcpp::Duration::from_nanoseconds(1e9);

  const double kPointBallDiameter = 0.025;
  const double kPointNormalLength = kPointBallDiameter * 4.0;

  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    // Point contacts
    const drake::multibody::PointPairContactInfo<double>& contact_info =
        contact_results.point_pair_contact_info(i);

    const std::string cname =
        contact_name(impl_->body_names.at(contact_info.bodyA_index()),
                     impl_->body_names.at(contact_info.bodyB_index()));

    // Create a ball at the point of contact
    visualization_msgs::msg::Marker ball_msg;
    ball_msg.header.frame_id = impl_->params.origin_frame_name;
    ball_msg.ns = "Pt|" + cname;
    ball_msg.id = output_value->markers.size();
    ball_msg.type = visualization_msgs::msg::Marker::SPHERE;
    ball_msg.action = visualization_msgs::msg::Marker::ADD;

    ball_msg.lifetime = kMarkerLifetime;
    ball_msg.frame_locked = true;

    const auto& color = impl_->params.default_color;
    ball_msg.color.r = color.r();
    ball_msg.color.g = color.g();
    ball_msg.color.b = color.b();
    ball_msg.color.a = color.a();

    ball_msg.scale.x = kPointBallDiameter;
    ball_msg.scale.y = kPointBallDiameter;
    ball_msg.scale.z = kPointBallDiameter;

    ball_msg.pose.position.x = contact_info.contact_point()[0];
    ball_msg.pose.position.y = contact_info.contact_point()[1];
    ball_msg.pose.position.z = contact_info.contact_point()[2];

    output_value->markers.push_back(ball_msg);

    // Create line representing contact normal
    visualization_msgs::msg::Marker normal_msg;
    normal_msg.header.frame_id = impl_->params.origin_frame_name;
    normal_msg.ns = cname;
    normal_msg.id = output_value->markers.size();
    normal_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    normal_msg.action = visualization_msgs::msg::Marker::ADD;

    normal_msg.lifetime = kMarkerLifetime;
    normal_msg.frame_locked = true;

    normal_msg.color.r = color.r();
    normal_msg.color.g = color.g();
    normal_msg.color.b = color.b();
    normal_msg.color.a = color.a();

    // Set line width
    normal_msg.scale.x = kPointNormalLength / 20.0;

    const auto& nhat_BA_W = contact_info.point_pair().nhat_BA_W;
    // C = Center of contact, l = one of the end points of the normal
    const auto p_Cl_W = kPointNormalLength / 2.0 * nhat_BA_W;

    normal_msg.points.resize(2);
    normal_msg.points.front().x = p_Cl_W.x() + contact_info.contact_point().x();
    normal_msg.points.front().y = p_Cl_W.y() + contact_info.contact_point().y();
    normal_msg.points.front().z = p_Cl_W.z() + contact_info.contact_point().z();
    normal_msg.points.back().x = -p_Cl_W.x() + contact_info.contact_point().x();
    normal_msg.points.back().y = -p_Cl_W.y() + contact_info.contact_point().y();
    normal_msg.points.back().z = -p_Cl_W.z() + contact_info.contact_point().z();

    output_value->markers.push_back(normal_msg);
  }

  for (int i = 0; i < contact_results.num_hydroelastic_contacts(); ++i) {
    // Hydroelastic Contacts
    const drake::multibody::HydroelasticContactInfo<double>&
        hydroelastic_contact_info =
            contact_results.hydroelastic_contact_info(i);
    const drake::geometry::ContactSurface<double>& surface =
        hydroelastic_contact_info.contact_surface();

    const FullBodyName& name1 =
        impl_->geometry_id_to_body_name_map.at(surface.id_M());
    const FullBodyName& name2 =
        impl_->geometry_id_to_body_name_map.at(surface.id_N());

    const std::string cname = contact_name(name1, name2);

    visualization_msgs::msg::Marker face_msg;
    face_msg.header.frame_id = impl_->params.origin_frame_name;
    face_msg.ns = cname;
    face_msg.id = output_value->markers.size();
    face_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    face_msg.action = visualization_msgs::msg::Marker::ADD;

    face_msg.lifetime = kMarkerLifetime;
    face_msg.frame_locked = true;

    drake::geometry::Rgba color = impl_->params.default_color;
    face_msg.color.r = color.r();
    face_msg.color.g = color.g();
    face_msg.color.b = color.b();
    face_msg.color.a = color.a();

    face_msg.scale.x = 1.0;
    face_msg.scale.y = 1.0;
    face_msg.scale.z = 1.0;

    const drake::geometry::TriangleSurfaceMesh<double>& mesh_W =
        surface.tri_mesh_W();
    face_msg.points.clear();
    face_msg.points.resize(mesh_W.num_triangles() * 3);
    face_msg.colors.clear();
    face_msg.colors.resize(mesh_W.num_triangles() * 3);
    face_msg.uv_coordinates.clear();
    face_msg.uv_coordinates.resize(mesh_W.num_triangles() * 3);

    Eigen::VectorXd pressures;
    pressures.resize(mesh_W.num_triangles() * 3);

    // Make lines for the edges
    visualization_msgs::msg::Marker edge_msg;
    edge_msg.header.frame_id = impl_->params.origin_frame_name;
    edge_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_msg.action = visualization_msgs::msg::Marker::ADD;
    edge_msg.lifetime = kMarkerLifetime;
    edge_msg.frame_locked = true;
    edge_msg.ns = cname;
    edge_msg.id = 1;
    // Set the size of the individual markers (depends on scale)
    // edge_msg.scale = ToScale(edge_scale * scale);
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
    const auto& field = surface.tri_e_MN();
    for (int j = 0; j < mesh_W.num_triangles(); ++j) {
      // Get the three vertices.
      const auto& face = mesh_W.element(j);
      const Eigen::Vector3d& vA = mesh_W.vertex(face.vertex(0));
      const Eigen::Vector3d& vB = mesh_W.vertex(face.vertex(1));
      const Eigen::Vector3d& vC = mesh_W.vertex(face.vertex(2));

      face_msg.points.at(index + 0) = tf2::toMsg(vA);
      face_msg.points.at(index + 1) = tf2::toMsg(vB);
      face_msg.points.at(index + 2) = tf2::toMsg(vC);

      for (size_t vert_index = 0; vert_index < 3; vert_index++) {
        pressures[index + vert_index] =
            field.EvaluateAtVertex(face.vertex(vert_index));
      }

      // 0->1
      edge_msg.points.push_back(tf2::toMsg(vA));
      edge_msg.points.push_back(tf2::toMsg(vB));
      // 1->2
      edge_msg.points.push_back(tf2::toMsg(vB));
      edge_msg.points.push_back(tf2::toMsg(vC));
      // 2->0
      edge_msg.points.push_back(tf2::toMsg(vC));
      edge_msg.points.push_back(tf2::toMsg(vA));

      index += 3;
    }

    // Color based on pressures.
    for (size_t tri_index = 0; tri_index < (size_t)mesh_W.num_triangles();
         tri_index++) {
      for (size_t vert_index = 0; vert_index < 3; vert_index++) {
        size_t arr_index = (tri_index * 3) + vert_index;
        double norm_data =
            calc_uv(pressures(arr_index), 0.0, pressures.maxCoeff());

        face_msg.colors.at(arr_index).r = 1.0;
        face_msg.colors.at(arr_index).g = 1.0;
        face_msg.colors.at(arr_index).b = 1.0;
        face_msg.colors.at(arr_index).a = 1.0;
        face_msg.uv_coordinates.at(arr_index).u = norm_data;
        face_msg.uv_coordinates.at(arr_index).v = 0;
      }
    }

    face_msg.texture.data = impl_->texture;
    face_msg.texture_resource = "embedded://heat_map.png";
    face_msg.texture.format = "png";

    output_value->markers.push_back(face_msg);
    output_value->markers.push_back(edge_msg);
  }
}

const ContactMarkersParams& ContactMarkersSystem::params() const {
  return impl_->params;
}

const drake::systems::InputPort<double>&
ContactMarkersSystem::get_contact_results_port() const {
  return get_input_port(impl_->contact_results_port_index);
}

const drake::systems::OutputPort<double>&
ContactMarkersSystem::get_markers_output_port() const {
  return get_output_port(impl_->contact_markers_port_index);
}

ContactMarkersSystem* ConnectContactResultsToRviz(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::SceneGraph<double>& scene_graph,
    drake_ros_core::DrakeRos* ros, ContactMarkersParams params,
    const std::string& markers_topic, const rclcpp::QoS& markers_qos) {
  // System that publishes ROS messages
  auto* markers_publisher =
      builder->AddSystem(drake_ros_core::RosPublisherSystem::Make<
                         visualization_msgs::msg::MarkerArray>(
          markers_topic, markers_qos, ros));

  // System that turns contact results into ROS Messages
  ContactMarkersSystem* contact_markers =
      builder->AddSystem<ContactMarkersSystem>(plant, scene_graph, params);

  builder->Connect(plant.get_contact_results_output_port(),
                   contact_markers->get_contact_results_port());

  builder->Connect(contact_markers->get_markers_output_port(),
                   markers_publisher->get_input_port());

  return contact_markers;
}
}  // namespace drake_ros_viz