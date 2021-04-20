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
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/parsing/scoped_names.h>

#include <rclcpp/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "drake_ros_systems/tf_broadcaster_system.hpp"


namespace drake_ros_systems
{
class TfBroadcasterSystemPrivate
{
public:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unordered_map<std::string, std::string> remappings_;
  const drake::systems::InputPort<double> * graph_query_port_{nullptr};
  const drake::systems::InputPort<double> * clock_port_{nullptr};
};

TfBroadcasterSystem::TfBroadcasterSystem(
  DrakeRosInterface * ros,
  const std::unordered_map<std::string, std::string> & remappings,
  const std::unordered_set<drake::systems::TriggerType> & publish_triggers,
  double publish_period)
: impl_(new TfBroadcasterSystemPrivate())
{
  impl_->tf_broadcaster_ = ros->create_tf_broadcaster();
  impl_->remappings_ = remappings;

  impl_->graph_query_port_ = &DeclareAbstractInputPort(
    "graph_query", drake::Value<drake::geometry::QueryObject<double>>{});

  impl_->clock_port_ = &DeclareAbstractInputPort("clock", drake::Value<double>{});

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
    this->DeclareForcedPublishEvent(&TfBroadcasterSystem::DoPublishFrames);
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

TfBroadcasterSystem::~TfBroadcasterSystem()
{
}

drake::systems::EventStatus
TfBroadcasterSystem::DoPublishFrames(
  const drake::systems::Context<double> & context) const
{
  const drake::geometry::QueryObject<double> & query_object =
    impl_->graph_query_port_->Eval<drake::geometry::QueryObject<double>>(context);
  const drake::geometry::SceneGraphInspector<double> & inspector = query_object.inspector();
  // TODO(hidmic): publish frame transforms w.r.t. to their parent frame
  //               instead of the world frame when an API is made available.
  if (inspector.num_frames() > 1) {
    std::vector<geometry_msgs::msg::TransformStamped> all_messages;
    all_messages.reserve(inspector.num_frames() - 1);
    const std::string & world_frame_name =
      inspector.GetName(inspector.world_frame_id());
    geometry_msgs::msg::TransformStamped message;
    message.header.frame_id = world_frame_name;
    const double time = impl_->clock_port_->Eval<double>(context);
    message.header.stamp = rclcpp::Time() + rclcpp::Duration::from_seconds(time);
    for (const drake::geometry::FrameId & frame_id : inspector.all_frame_ids()) {
      if (frame_id == inspector.world_frame_id()) {
        continue;
      }
      const drake::math::RigidTransform<double> & X_FW =
        query_object.GetPoseInWorld(frame_id);
      std::string frame_name = inspector.GetName(frame_id);
      if (impl_->remappings_.count(frame_name) > 0) {
        frame_name = impl_->remappings_[frame_name];
      }
      drake::multibody::parsing::ScopedName scoped_frame_name =
        drake::multibody::parsing::ParseScopedName(frame_name);
      if (impl_->remappings_.count(scoped_frame_name.instance_name) > 0) {
        scoped_frame_name.instance_name =
          impl_->remappings_[scoped_frame_name.instance_name];
      }
      message.child_frame_id = drake::multibody::parsing::PrefixName(
        scoped_frame_name.instance_name, scoped_frame_name.name);
      const drake::Vector3<double> & p_FW = X_FW.translation();
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
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace drake_ros_systems
