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

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/basic_types.hpp>

#include <memory>
#include <utility>
#include <vector>

#include "drake_ros_systems/drake_ros.hpp"
#include "drake_ros_systems/ros_interface_system.hpp"
#include "drake_ros_systems/ros_publisher_system.hpp"
#include "drake_ros_systems/ros_subscriber_system.hpp"

using drake_ros_systems::DrakeRos;
using drake_ros_systems::RosInterfaceSystem;
using drake_ros_systems::RosPublisherSystem;
using drake_ros_systems::RosSubscriberSystem;


TEST(Integration, sub_to_pub) {
  drake::systems::DiagramBuilder<double> builder;

  const size_t num_msgs = 5;

  rclcpp::QoS qos{rclcpp::KeepLast(num_msgs)};
  qos.transient_local().reliable();

  auto sys_ros_interface = builder.AddSystem<RosInterfaceSystem>(std::make_unique<DrakeRos>());
  auto sys_sub = builder.AddSystem(
    RosSubscriberSystem::Make<test_msgs::msg::BasicTypes>(
      "in", qos, sys_ros_interface->get_ros_interface()));
  auto sys_pub = builder.AddSystem(
    RosPublisherSystem::Make<test_msgs::msg::BasicTypes>(
      "out", qos, sys_ros_interface->get_ros_interface()));

  builder.Connect(sys_sub->get_output_port(0), sys_pub->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto simulator =
    std::make_unique<drake::systems::Simulator<double>>(*diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  auto & simulator_context = simulator->get_mutable_context();

  // Don't need to rclcpp::init because DrakeRos uses global rclcpp::Context by default
  auto node = rclcpp::Node::make_shared("sub_to_pub");

  // Create publisher talking to subscriber system.
  auto publisher = node->create_publisher<test_msgs::msg::BasicTypes>("in", qos);

  // Create subscription listening to publisher system
  std::vector<std::unique_ptr<test_msgs::msg::BasicTypes>> rx_msgs;
  auto rx_callback = [&](std::unique_ptr<test_msgs::msg::BasicTypes> msg)
    {
      rx_msgs.push_back(std::move(msg));
    };
  auto subscription =
    node->create_subscription<test_msgs::msg::BasicTypes>("out", qos, rx_callback);

  // Send messages into the drake system
  for (size_t p = 0; p < num_msgs; ++p) {
    publisher->publish(std::make_unique<test_msgs::msg::BasicTypes>());
  }

  const int timeout_sec = 5;
  const int spins_per_sec = 10;
  const float time_delta = 1.0f / spins_per_sec;
  for (int i = 0; i < timeout_sec * spins_per_sec && rx_msgs.size() < num_msgs; ++i) {
    rclcpp::spin_some(node);
    simulator->AdvanceTo(simulator_context.get_time() + time_delta);
  }

  // Make sure same number of messages got out
  ASSERT_EQ(num_msgs, rx_msgs.size());
}
