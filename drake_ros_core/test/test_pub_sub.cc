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
#include <utility>
#include <vector>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/basic_types.hpp>

#include "drake_ros_core/drake_ros.h"
#include "drake_ros_core/ros_interface_system.h"
#include "drake_ros_core/ros_publisher_system.h"
#include "drake_ros_core/ros_subscriber_system.h"

using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_core::RosPublisherSystem;
using drake_ros_core::RosSubscriberSystem;

TEST(Integration, sub_to_pub) {
  drake::systems::DiagramBuilder<double> builder;

  const size_t num_msgs = 5;

  rclcpp::QoS qos{rclcpp::KeepLast(num_msgs)};
  qos.transient_local().reliable();

  auto ros_interface_system =
      builder.AddSystem<RosInterfaceSystem>(std::make_unique<DrakeRos>());
  auto ros_subscriber_system =
      builder.AddSystem(RosSubscriberSystem::Make<test_msgs::msg::BasicTypes>(
          "in", qos, ros_interface_system->get_ros_interface()));
  auto ros_publisher_system =
      builder.AddSystem(RosPublisherSystem::Make<test_msgs::msg::BasicTypes>(
          "out", qos, ros_interface_system->get_ros_interface()));

  builder.Connect(ros_subscriber_system->get_output_port(0),
                  ros_publisher_system->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  auto& simulator_context = simulator->get_mutable_context();

  // Don't need to rclcpp::init because DrakeRos uses global rclcpp::Context by
  // default
  auto node = rclcpp::Node::make_shared("sub_to_pub");

  // Create publisher talking to subscriber system.
  auto publisher =
      node->create_publisher<test_msgs::msg::BasicTypes>("in", qos);

  // Create subscription listening to publisher system
  std::vector<std::unique_ptr<test_msgs::msg::BasicTypes>> rx_msgs;
  auto rx_callback = [&](std::unique_ptr<test_msgs::msg::BasicTypes> msg) {
    // Cope with lack of synchronization between subscriber
    // and publisher systems by ignoring duplicate messages.
    if (rx_msgs.empty() || rx_msgs.back()->uint64_value != msg->uint64_value) {
      rx_msgs.push_back(std::move(msg));
    }
  };
  auto subscription = node->create_subscription<test_msgs::msg::BasicTypes>(
      "out", qos, rx_callback);

  size_t num_msgs_sent = 0;
  const int timeout_sec = 5;
  const int spins_per_sec = 10;
  const float time_delta = 1.0f / spins_per_sec;
  for (int i = 0; i < timeout_sec * spins_per_sec; ++i) {
    if (rx_msgs.size() >= num_msgs) {
      break;
    }
    // Cope with lack of synchronization between subscriber
    // and publisher systems by sending one message at a time.
    if (rx_msgs.size() == num_msgs_sent) {
      // Send message into the drake system
      auto message = std::make_unique<test_msgs::msg::BasicTypes>();
      message->uint64_value = num_msgs_sent++;
      publisher->publish(std::move(message));
    }
    rclcpp::spin_some(node);
    simulator->AdvanceTo(simulator_context.get_time() + time_delta);
  }

  // Make sure same number of messages got out
  ASSERT_EQ(num_msgs, rx_msgs.size());
  // Make sure all messages got out and in the right order
  for (size_t p = 0; p < num_msgs; ++p) {
    EXPECT_EQ(rx_msgs[p]->uint64_value, p);
  }
}
