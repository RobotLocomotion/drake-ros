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

#include "drake_ros_tf2/scene_tf_system.h"

#include <unordered_set>

#include <drake/geometry/query_object.h>
#include <drake/geometry/scene_graph_inspector.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include "drake_ros_tf2/utilities/name_conventions.h"
#include "drake_ros_tf2/utilities/type_conversion.h"

namespace drake_ros_tf2 {
class SceneTfSystem::Impl {
 public:
  std::unordered_set<const drake::multibody::MultibodyPlant<double>*> plants;
  drake::systems::InputPortIndex graph_query_port_index;
  drake::systems::InputPortIndex body_poses_port_index;
  drake::systems::OutputPortIndex scene_tf_port_index;

  bool precomputed = false;
  // Pre-computed TF information
  struct Frame {
    drake::geometry::FrameId id;
    drake::multibody::BodyIndex body_index;
    std::string name;
    geometry_msgs::msg::TransformStamped transform;
  };
  using ParentFrameMap = std::unordered_map<drake::geometry::FrameId, Frame>;
  ParentFrameMap parent_frames_map;
};

SceneTfSystem::SceneTfSystem() : impl_(new Impl()) {
  impl_->graph_query_port_index =
      this->DeclareAbstractInputPort(
              "graph_query",
              drake::Value<drake::geometry::QueryObject<double>>{})
          .get_index();

  impl_->scene_tf_port_index =
      this->DeclareAbstractOutputPort("scene_tf", &SceneTfSystem::CalcSceneTf)
          .get_index();
}

SceneTfSystem::~SceneTfSystem() {}

void SceneTfSystem::RegisterMultibodyPlant(
    const drake::multibody::MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  impl_->plants.insert(plant);
}

const drake::systems::InputPort<double>&
SceneTfSystem::get_graph_query_input_port() const {
  return get_input_port(impl_->graph_query_port_index);
}

const drake::systems::OutputPort<double>&
SceneTfSystem::get_scene_tf_output_port() const {
  return get_output_port(impl_->scene_tf_port_index);
}

void SceneTfSystem::ComputeFrameHierarchy() {
  impl_->precomputed = true;
  // Clear out the frame hierarchy so we can re-compute it from scratch,
  // in case of connections between MbPs
  impl_->parent_frames_map.clear();

  // Iterate over the registered plants
  for (auto* plant : impl_->plants) {
    DRAKE_THROW_UNLESS(plant->is_finalized());

    for (auto ii = 0; ii < plant->num_joints(); ++ii) {
      auto& joint = plant->get_joint(drake::multibody::JointIndex(ii));

      auto& parent_body = joint.parent_body();
      auto parent_body_index = plant->GetIndexOfBodyIfExists(parent_body);
      auto parent_body_frame_id = plant->GetBodyFrameIdOrThrow(parent_body_index.value());

      auto& child_body = joint.child_body();
      auto child_body_index = plant->GetIndexOfBodyIfExists(child_body);
      auto child_body_frame_id = plant->GetBodyFrameIdOrThrow(child_body_index.value());

      geometry_msgs::msg::TransformStamped transform;
      if (joint.parent_body().name() == plant->world_body().name()) {
        transform.header.frame_id = "world";
      } else {
        transform.header.frame_id = GetTfFrameName(
          joint.parent_body(),
          plant,
          parent_body_frame_id);
      }
      transform.child_frame_id = GetTfFrameName(
        joint.child_body(),
        plant,
        child_body_frame_id);

      impl_->parent_frames_map.insert(
        {child_body_frame_id,
        SceneTfSystem::Impl::Frame({parent_body_frame_id,
           parent_body_index.value(),
           joint.parent_body().name(),
           transform})
        });
    }
  }
}

void SceneTfSystem::CalcSceneTf(const drake::systems::Context<double>& context,
                                tf2_msgs::msg::TFMessage* output_value) const {
  const drake::geometry::QueryObject<double>& query_object =
      get_graph_query_input_port().Eval<drake::geometry::QueryObject<double>>(
          context);
  const drake::geometry::SceneGraphInspector<double>& inspector =
      query_object.inspector();
  if (inspector.num_frames() > 1) {
    output_value->transforms.clear();
    output_value->transforms.reserve(inspector.num_frames() - 1);

    for (const drake::geometry::FrameId& frame_id :
         inspector.GetAllFrameIds()) {
      if (frame_id == inspector.world_frame_id()) {
        continue;
      }
      if (impl_->parent_frames_map.find(frame_id) != impl_->parent_frames_map.end()) {
        auto& parent_frame = impl_->parent_frames_map[frame_id];
        parent_frame.transform.header.stamp =
          rclcpp::Time() + rclcpp::Duration::from_seconds(context.get_time());
        auto parent_transform = query_object.GetPoseInParent(parent_frame.id);
        auto child_transform = query_object.GetPoseInParent(frame_id);
        parent_frame.transform.transform =
            ToTransformMsg(parent_transform.inverse() * child_transform);
        output_value->transforms.push_back(parent_frame.transform);
      } else {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp =
          rclcpp::Time() + rclcpp::Duration::from_seconds(context.get_time());

        transform.header.frame_id = GetTfFrameName(
            inspector, impl_->plants, inspector.GetParentFrame(frame_id));
        transform.child_frame_id =
            GetTfFrameName(inspector, impl_->plants, frame_id);
        transform.transform =
            ToTransformMsg(query_object.GetPoseInParent(frame_id));
        output_value->transforms.push_back(transform);
      }
    }
  }
}

}  // namespace drake_ros_tf2
