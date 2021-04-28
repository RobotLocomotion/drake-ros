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

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include <drake/common/eigen_types.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/scene_graph_inspector.h>

#include <rclcpp/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "drake_ros_systems/tf_broadcaster_system.hpp"
#include "drake_ros_systems/utilities/type_conversion.hpp"


namespace drake_ros_systems
{
class TfBroadcasterSystem::TfBroadcasterSystemPrivate
{
public:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  drake::systems::InputPortIndex graph_query_port_index;
};

TfBroadcasterSystem::TfBroadcasterSystem(
  DrakeRosInterface * ros,
  const std::unordered_set<drake::systems::TriggerType> & publish_triggers,
  double publish_period)
: impl_(new TfBroadcasterSystemPrivate())
{
  impl_->tf_broadcaster_ = ros->create_tf_broadcaster();

  impl_->graph_query_port_index =
    this->DeclareAbstractInputPort(
      "graph_query", drake::Value<drake::geometry::QueryObject<double>>{}).get_index();

  // vvv Mostly copied from LcmPublisherSystem vvv
  using TriggerType = drake::systems::TriggerType;
  // Check that publish_triggers does not contain an unsupported trigger.
  for (const auto & trigger : publish_triggers) {
    if (
      (trigger != TriggerType::kForced) &&
      (trigger != TriggerType::kPeriodic) &&
      (trigger != TriggerType::kPerStep))
    {
      throw std::invalid_argument(
        "Only kForced, kPeriodic, or kPerStep are supported");
    }
  }

  // Declare a forced publish so that any time Publish(.) is called on this
  // system (or a Diagram containing it), a message is emitted.
  if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
    this->DeclareForcedPublishEvent(&TfBroadcasterSystem::PublishFrames);
  }

  if (publish_triggers.find(TriggerType::kPeriodic) != publish_triggers.end()) {
    if (publish_period <= 0.0) {
      throw std::invalid_argument("kPeriodic requires publish_period > 0");
    }
    const double offset = 0.0;
    this->DeclarePeriodicPublishEvent(
      publish_period, offset,
      &TfBroadcasterSystem::PublishFrames);
  } else if (publish_period > 0) {
    // publish_period > 0 without drake::systems::TriggerType::kPeriodic has no meaning and is
    // likely a mistake.
    throw std::invalid_argument("publish_period > 0 requires kPeriodic");
  }

  if (publish_triggers.find(TriggerType::kPerStep) != publish_triggers.end()) {
    DeclarePerStepEvent(
      drake::systems::PublishEvent<double>(
        [this](
          const drake::systems::Context<double> & context,
          const drake::systems::PublishEvent<double> &) {
          PublishFrames(context);
        }));
  }
  // ^^^ Mostly copied from LcmPublisherSystem ^^^
}

TfBroadcasterSystem::~TfBroadcasterSystem()
{
}

drake::systems::EventStatus
TfBroadcasterSystem::PublishFrames(
  const drake::systems::Context<double> & context) const
{
  const drake::geometry::QueryObject<double> & query_object =
    get_input_port(impl_->graph_query_port_index)
      .Eval<drake::geometry::QueryObject<double>>(context);
  const drake::geometry::SceneGraphInspector<double> & inspector = query_object.inspector();
  // TODO(hidmic): publish frame transforms w.r.t. to their parent frame
  //               instead of the world frame when an API is made available.
  if (inspector.num_frames() > 1) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(inspector.num_frames() - 1);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp =
      rclcpp::Time() + rclcpp::Duration::from_seconds(context.get_time());
    transform.header.frame_id = inspector.GetName(inspector.world_frame_id());
    for (const drake::geometry::FrameId & frame_id : inspector.all_frame_ids()) {
      if (frame_id == inspector.world_frame_id()) {
        continue;
      }
      transform.child_frame_id = inspector.GetName(frame_id);
      transform.transform =
        utilities::ToTransformMsg(query_object.GetPoseInWorld(frame_id));
      transforms.push_back(transform);
    }
    impl_->tf_broadcaster_->sendTransform(transforms);
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace drake_ros_systems
