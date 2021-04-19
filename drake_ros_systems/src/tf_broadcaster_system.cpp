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

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include <rclcpp/clock.hpp>
#include <tf2_ros/transform_broadcaster.h> // NOLINT

#include "drake_ros_systems/tf_broadcaster_system.hpp"


namespace drake_ros_systems
{
class TfBroadcasterSystemPrivate
{
public:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

TfBroadcasterSystem::TfBroadcasterSystem(
  DrakeRosInterface * ros,
  const std::unordered_set<drake::systems::TriggerType> & publish_triggers,
  double publish_period)
: impl_(new TFBroadcasterSystemPrivate())
{
  impl_->clock_ = ros->get_clock();
  impl_->tf_broadcaster_ = ros->create_tf_broadcaster();

  DeclareAbstractInputPort(
    "graph_query", drake::Value<drake::geometry::QueryObject<double>>{});

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
    this->DeclareForcedPublishEvent(&TFBroadcasterSystem::DoPublishFrames);
  }

  if (publish_triggers.find(TriggerType::kPeriodic) != publish_triggers.end()) {
    if (publish_period <= 0.0) {
      throw std::invalid_argument("kPeriodic requires publish_period > 0");
    }
    const double offset = 0.0;
    this->DeclarePeriodicPublishEvent(
      publish_period, offset,
      &TfBroadcasterSystem::DoPublishFrames);
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
          DoPublishFrames(context);
        }));
  }
  // ^^^ Mostly copied from LcmPublisherSystem ^^^
}

drake::systems::EventStatus
TfBroadcasterSystem::DoPublishFrames(
  const drake::systems::Context<double> & context) const
{
  const drake::geometry::QueryObject<double> & query_object =
    get_input_port().Eval<drake::geometry::QueryObject<double>>(context);
  const drake::geometry::SceneGraphInspector<double> & inspector =
    query_object.inspector();
  // TODO(hidmic): publish frame transforms w.r.t. to their parent frame
  //               instead of the world frame when an API is made available.
  std::vector<geometry_msgs::msg::TransformStamped> all_messages;
  all_messages.reserve(inspector.num_frames() - 1);
  const std::string & world_frame_name =
    inspector.Getname(inspector.world_frame_id());
  geometry_msgs::msg::TransformStamped message;
  message.header.frame_id = world_frame_name;
  message.header.stamp = impl_->clock_->now();
  for (const drake::geometry::FrameId & frame_id : inspector.all_frame_ids()) {
    if (frame_id != inspector.world_frame_id()) {
      continue;
    }
    const drake::math::RigidTransform<double> & X_FW =
      query_object.GetPoseInWord(frame_id);
    message.child_frame_id = inspector.GetName(frame_id);
    const drake::math::Vector3<double> & p_FW = X_FW.translation();
    message.transform.translation.x = p_FW.x();
    message.transform.translation.y = p_FW.y();
    message.transform.translation.z = p_FW.z();
    const Eigen::Quaternion<double> R_FW = X_FW.rotation().ToQuaternion();
    message.transform.rotation.x = R_FW.x();
    message.transform.rotation.y = R_FW.y();
    message.transform.rotation.z = R_FW.z();
    message.transform.rotation.w = R_FW.w();
    all_messages.push_back(message);
  }
  impl_->tf_broadcaster_->sendTransform(all_messages);
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace drake_ros_systems
