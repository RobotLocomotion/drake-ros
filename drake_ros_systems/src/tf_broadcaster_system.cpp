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

#include <drake/systems/framework/diagram_builder.h>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>

#include <memory>
#include <unordered_set>

#include "drake_ros_systems/drake_ros_interface.hpp"
#include "drake_ros_systems/ros_publisher_system.hpp"
#include "drake_ros_systems/scene_tf_system.hpp"
#include "drake_ros_systems/tf_broadcaster_system.hpp"


namespace drake_ros_systems
{

TfBroadcasterSystem::TfBroadcasterSystem(
  std::shared_ptr<DrakeRosInterface> ros_interface,
  const std::unordered_set<drake::systems::TriggerType> & publish_triggers,
  double publish_period)
{
  drake::systems::DiagramBuilder<double> builder;

  auto scene_tf = builder.AddSystem<SceneTfSystem>();

  auto scene_tf_publisher = builder.AddSystem(
    RosPublisherSystem::Make<tf2_msgs::msg::TFMessage>(
      "/tf", tf2_ros::DynamicBroadcasterQoS(),
      ros_interface, publish_triggers, publish_period));

  builder.Connect(
    scene_tf->get_scene_tf_output_port(),
    scene_tf_publisher->get_input_port());

  builder.ExportInput(scene_tf->get_graph_query_port(), "graph_query");

  builder.BuildInto(this);
}

const drake::systems::InputPort<double> &
TfBroadcasterSystem::get_graph_query_port() const
{
  return get_input_port();
}

}  // namespace drake_ros_systems
