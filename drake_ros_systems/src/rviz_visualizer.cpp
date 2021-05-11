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

#include <rclcpp/qos.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <unordered_set>

#include "drake_ros_systems/ros_publisher_system.hpp"
#include "drake_ros_systems/scene_markers_system.hpp"
#include "drake_ros_systems/tf_broadcaster_system.hpp"

#include "drake_ros_systems/rviz_visualizer.hpp"


namespace drake_ros_systems
{

RvizVisualizer::RvizVisualizer(
  std::shared_ptr<DrakeRosInterface> ros_interface,
  const std::unordered_set<drake::systems::TriggerType> & publish_triggers,
  double publish_period, bool publish_tf)
{
  drake::systems::DiagramBuilder<double> builder;

  auto scene_markers = builder.AddSystem<SceneMarkersSystem>();

  auto scene_markers_publisher = builder.AddSystem(
    RosPublisherSystem::Make<visualization_msgs::msg::MarkerArray>(
      "/scene_markers", rclcpp::QoS(1), ros_interface, publish_triggers, publish_period));

  builder.Connect(
    scene_markers->get_markers_output_port(),
    scene_markers_publisher->get_input_port());

  builder.ExportInput(scene_markers->get_graph_query_port(), "graph_query");

  if (publish_tf) {
    auto tf_broadcaster = builder.AddSystem<TfBroadcasterSystem>(
      ros_interface, publish_triggers, publish_period);

    builder.ConnectInput("graph_query", tf_broadcaster->get_graph_query_port());
  }

  builder.BuildInto(this);
}

const drake::systems::InputPort<double> &
RvizVisualizer::get_graph_query_port() const
{
  return get_input_port();
}

}  // namespace drake_ros_systems
