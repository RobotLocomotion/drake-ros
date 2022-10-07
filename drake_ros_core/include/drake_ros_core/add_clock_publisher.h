// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#pragma once

#include <drake/systems/framework/diagram_builder.h>
#include <drake_ros_core/clock_system.h>
#include <drake_ros_core/ros_publisher_system.h>
#include <rclcpp/qos.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace drake_ros_core {
void AddClockPublisher(
    drake::systems::DiagramBuilder<double>* builder, DrakeRos* ros,
    const std::string& topic_name = "/clock",
    const rclcpp::QoS& qos = rclcpp::ClockQoS(),
    const std::unordered_set<drake::systems::TriggerType>& publish_triggers =
        RosPublisherSystem::kDefaultTriggerTypes,
    double publish_period = 0.0) {
  auto* clock_system = builder->AddSystem<ClockSystem>();

  auto* pub_system =
      builder->AddSystem(RosPublisherSystem::Make<rosgraph_msgs::msg::Clock>(
          topic_name, qos, ros, publish_triggers, publish_period));

  builder->Connect(clock_system->get_output_port(),
                   pub_system->get_input_port());
}
}  // namespace drake_ros_core
