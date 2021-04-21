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
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/leaf_system.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "drake_ros_systems/scene_markers_system.hpp"
#include "drake_ros_systems/utilities/type_conversion.hpp"


namespace drake_ros_systems {

namespace {

class SceneGeometryToMarkers : public drake::geometry::ShapeReifier {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SceneGeometryToMarkers)

  SceneGeometryToMarkers(
    const drake::geometry::Role & role,
    const drake::geometry::Rgba & default_color)
  : role_(role), default_color_(default_color)
  {
  }

  ~SceneGeometryToMarkers() override = default;

  void Populate(
    const drake::geometry::SceneGraphInspector<double> & inspector,
    const drake::geometry::GeometryId & geometry_id,
    visualization_msgs::msg::MarkerArray * marker_array)
  {
    DRAKE_ASSERT(nullptr != marker_array);

    marker_array_ = marker_array;

    prototype_marker_.header.frame_id =
      inspector.GetName(inspector.GetFrameId(geometry_id));
    prototype_marker_.ns = inspector.GetOwningSourceName(geometry_id)
                           + "::" + inspector.GetName(geometry_id);
    prototype_marker_.id = 0;
    prototype_marker_.action = visualization_msgs::msg::Marker::MODIFY;

    prototype_marker_.lifetime = rclcpp::Duration::from_nanoseconds(0);
    prototype_marker_.frame_locked = true;

    const drake::geometry::GeometryProperties * props =
      inspector.GetProperties(geometry_id, role_);
    DRAKE_ASSERT(nullptr != props);
    drake::geometry::Rgba default_color = default_color_;
    if (drake::geometry::Role::kIllustration != role_) {
      const drake::geometry::IllustrationProperties * illustration_props =
        inspector.GetIllustrationProperties(geometry_id);
      if (illustration_props) {
        default_color =
          illustration_props->GetPropertyOrDefault(
            "phong", "diffuse", default_color);
      }
    }
    const drake::geometry::Rgba & color =
      props->GetPropertyOrDefault("phong", "diffuse", default_color);
    prototype_marker_.color.r = color.r();
    prototype_marker_.color.g = color.g();
    prototype_marker_.color.b = color.b();
    prototype_marker_.color.a = color.a();

    X_FG_ = inspector.GetPoseInFrame(geometry_id);

    inspector.GetShape(geometry_id).Reify(this);
  }

private:
  using drake::geometry::ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const drake::geometry::Sphere & sphere, void *) override
  {
    prototype_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    prototype_marker_.scale.x = sphere.radius();
    prototype_marker_.scale.y = sphere.radius();
    prototype_marker_.scale.z = sphere.radius();
    prototype_marker_.pose = utilities::ToPoseMsg(X_FG_);
    marker_array_->markers.push_back(prototype_marker_);
  }

  void ImplementGeometry(const drake::geometry::Ellipsoid & ellipsoid, void *) override
  {
    prototype_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    prototype_marker_.scale.x = ellipsoid.a();
    prototype_marker_.scale.y = ellipsoid.b();
    prototype_marker_.scale.z = ellipsoid.c();
    prototype_marker_.pose = utilities::ToPoseMsg(X_FG_);

    marker_array_->markers.push_back(prototype_marker_);
  }

  void ImplementGeometry(const drake::geometry::Cylinder & cylinder, void *) override
  {
    prototype_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
    prototype_marker_.scale.x = cylinder.radius();
    prototype_marker_.scale.y = cylinder.radius();
    prototype_marker_.scale.z = cylinder.length();
    prototype_marker_.pose = utilities::ToPoseMsg(X_FG_);

    marker_array_->markers.push_back(prototype_marker_);
  }

  void ImplementGeometry(const drake::geometry::HalfSpace &, void *) override
  {
    constexpr double kHalfSpaceLength = 50.;
    constexpr double kHalfSpaceThickness = 1.;

    prototype_marker_.type = visualization_msgs::msg::Marker::CUBE;
    prototype_marker_.scale.x = kHalfSpaceLength;
    prototype_marker_.scale.y = kHalfSpaceLength;
    prototype_marker_.scale.z = kHalfSpaceThickness;
    const drake::math::RigidTransform<double> X_GH{
      drake::Vector3<double>{0., 0., -kHalfSpaceThickness / 2.}};
    const drake::math::RigidTransform<double> X_FH = X_FG_ * X_GH;
    prototype_marker_.pose = utilities::ToPoseMsg(X_FH);

    marker_array_->markers.push_back(prototype_marker_);
  }

  void ImplementGeometry(const drake::geometry::Box & box, void *) override
  {
    prototype_marker_.type = visualization_msgs::msg::Marker::CUBE;
    prototype_marker_.scale.x = box.width();
    prototype_marker_.scale.y = box.depth();
    prototype_marker_.scale.z = box.height();
    prototype_marker_.pose = utilities::ToPoseMsg(X_FG_);

    marker_array_->markers.push_back(prototype_marker_);
  }

  void ImplementGeometry(const drake::geometry::Capsule & capsule, void *) override
  {
    visualization_msgs::msg::Marker body_marker = prototype_marker_;
    body_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    body_marker.scale.x = capsule.radius();
    body_marker.scale.y = capsule.radius();
    body_marker.scale.z = capsule.length();
    prototype_marker_.pose = utilities::ToPoseMsg(X_FG_);

    marker_array_->markers.push_back(body_marker);

    visualization_msgs::msg::Marker upper_cap_marker = prototype_marker_;
    upper_cap_marker.id = 1;
    upper_cap_marker.type = visualization_msgs::msg::Marker::SPHERE;
    upper_cap_marker.scale.x = capsule.radius();
    upper_cap_marker.scale.y = capsule.radius();
    upper_cap_marker.scale.z = capsule.radius();
    const drake::math::RigidTransform<double> X_GU{
      drake::Vector3<double>{0., 0., capsule.length() / 2.}};
    const drake::math::RigidTransform<double> X_FU = X_FG_ * X_GU;
    upper_cap_marker.pose = utilities::ToPoseMsg(X_FU);

    marker_array_->markers.push_back(upper_cap_marker);

    visualization_msgs::msg::Marker lower_cap_marker = upper_cap_marker;
    lower_cap_marker.id = 2;
    const drake::math::RigidTransform<double> X_GL{
      drake::Vector3<double>{0., 0., -capsule.length() / 2.}};
    const drake::math::RigidTransform<double> X_FL = X_FG_ * X_GL;
    lower_cap_marker.pose = utilities::ToPoseMsg(X_FL);

    marker_array_->markers.push_back(lower_cap_marker);
  }

  void ImplementGeometry(const drake::geometry::Convex & convex, void *) override
  {
    prototype_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    prototype_marker_.scale.x = convex.scale();
    prototype_marker_.scale.y = convex.scale();
    prototype_marker_.scale.z = convex.scale();
    // Assume it is an absolute path and turn it into a file URL.
    prototype_marker_.mesh_resource = "file://" + convex.filename();
    prototype_marker_.pose = utilities::ToPoseMsg(X_FG_);

    marker_array_->markers.push_back(prototype_marker_);
  }

  void ImplementGeometry(const drake::geometry::Mesh & mesh, void *) override
  {
    prototype_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    prototype_marker_.scale.x = mesh.scale();
    prototype_marker_.scale.y = mesh.scale();
    prototype_marker_.scale.z = mesh.scale();
    // Assume it is an absolute path and turn it into a file URL.
    prototype_marker_.mesh_resource = "file://" + mesh.filename();
    prototype_marker_.pose = utilities::ToPoseMsg(X_FG_);

    marker_array_->markers.push_back(prototype_marker_);
  }

  const drake::geometry::Role role_;
  const drake::geometry::Rgba default_color_;
  visualization_msgs::msg::MarkerArray * marker_array_{nullptr};
  visualization_msgs::msg::Marker prototype_marker_{};
  drake::math::RigidTransform<double> X_FG_{};
};

}  // namespace

class SceneMarkersSystem::SceneMarkersSystemPrivate
{
public:
  SceneMarkersSystemPrivate(
    const drake::geometry::Role & _role,
    const drake::geometry::Rgba & _default_color)
  : role(_role), default_color(_default_color)
  {
  }

  const drake::geometry::Role role;
  const drake::geometry::Rgba default_color;
  mutable drake::geometry::GeometryVersion version;
  drake::systems::CacheIndex scene_markers_cache_index;
  drake::systems::InputPortIndex graph_query_port_index;
  drake::systems::InputPortIndex clock_port_index;
};

SceneMarkersSystem::SceneMarkersSystem(
  const drake::geometry::Role & role,
  const drake::geometry::Rgba & default_color)
: impl_(new SceneMarkersSystemPrivate(role, default_color))
{
  impl_->graph_query_port_index =
    this->DeclareAbstractInputPort(
      "graph_query", drake::Value<drake::geometry::QueryObject<double>>{}).get_index();

  impl_->clock_port_index =
    this->DeclareAbstractInputPort("clock", drake::Value<double>{}).get_index();

  impl_->scene_markers_cache_index =
      this->DeclareCacheEntry("scene_markers_cache",
                              &SceneMarkersSystem::CalcSceneMarkers,
                              {nothing_ticket()}).cache_index();

  this->DeclareAbstractOutputPort(
    "scene_markers", &SceneMarkersSystem::PopulateSceneMarkersMessage);
}

SceneMarkersSystem::~SceneMarkersSystem()
{
}

void
SceneMarkersSystem::PopulateSceneMarkersMessage(
  const drake::systems::Context<double> & context,
  visualization_msgs::msg::MarkerArray * output_value) const
{
  *output_value = this->EvalSceneMarkers(context);

  rclcpp::Time current_time;
  current_time += rclcpp::Duration::from_seconds(
    get_input_port(impl_->clock_port_index).Eval<double>(context));
  builtin_interfaces::msg::Time stamp = current_time;
  for (visualization_msgs::msg::Marker & marker : output_value->markers)
  {
    marker.header.stamp = stamp;
  }
}

const visualization_msgs::msg::MarkerArray &
SceneMarkersSystem::EvalSceneMarkers(
  const drake::systems::Context<double> & context) const
{
  const drake::geometry::QueryObject<double> & query_object =
    get_input_port(impl_->graph_query_port_index)
      .Eval<drake::geometry::QueryObject<double>>(context);
  const drake::geometry::GeometryVersion & current_version =
    query_object.inspector().geometry_version();
  if (!impl_->version.IsSameAs(current_version, impl_->role)) {
    // Invalidate scene markers cache
    get_cache_entry(impl_->scene_markers_cache_index)
      .get_mutable_cache_entry_value(context)
        .mark_out_of_date();
    impl_->version = current_version;
  }
  return get_cache_entry(impl_->scene_markers_cache_index)
    .Eval<visualization_msgs::msg::MarkerArray>(context);
}

void
SceneMarkersSystem::CalcSceneMarkers(
  const drake::systems::Context<double> & context,
  visualization_msgs::msg::MarkerArray * output_value) const
{
  const drake::geometry::QueryObject<double> & query_object =
    get_input_port(impl_->graph_query_port_index)
      .Eval<drake::geometry::QueryObject<double>>(context);
  const drake::geometry::SceneGraphInspector<double> & inspector = query_object.inspector();
  output_value->markers.reserve(inspector.NumGeometriesWithRole(impl_->role));
  for (const drake::geometry::FrameId & frame_id : inspector.all_frame_ids()) {
    for (const drake::geometry::GeometryId & geometry_id :
         inspector.GetGeometries(frame_id, impl_->role)) {
      SceneGeometryToMarkers(impl_->role, impl_->default_color)
        .Populate(inspector, geometry_id, output_value);
    }
  }
}

}  // drake_ros_systems
