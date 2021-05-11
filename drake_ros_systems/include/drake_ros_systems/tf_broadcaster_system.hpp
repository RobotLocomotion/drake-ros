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
#ifndef DRAKE_ROS_SYSTEMS__TF_BROADCASTER_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__TF_BROADCASTER_SYSTEM_HPP_

#include <drake/systems/framework/leaf_system.h>

#include <memory>
#include <unordered_set>

#include "drake_ros_systems/drake_ros_interface.hpp"


namespace drake_ros_systems
{
/// System for tf2 transform broadcasting.
///
/// This system is a subdiagram aggregating a SceneTFSystem and a
/// RosPublisherSystem to broadcast SceneGraph frame transforms.
/// Messages are published to the `/tf` ROS topic.
///
/// It exports one input port:
/// - *graph_query* (abstract): expects a QueryObject from the SceneGraph.
class TfBroadcasterSystem : public drake::systems::Diagram<double>
{
public:
  TfBroadcasterSystem(
    std::shared_ptr<DrakeRosInterface> ros_interface,
    const std::unordered_set<drake::systems::TriggerType> & publish_triggers = {
      drake::systems::TriggerType::kPerStep, drake::systems::TriggerType::kForced},
    double publish_period = 0.0);

  const drake::systems::InputPort<double> & get_graph_query_port() const;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__TF_BROADCASTER_SYSTEM_HPP_
