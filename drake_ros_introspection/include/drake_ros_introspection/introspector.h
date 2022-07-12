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

#include <drake/systems/framework/diagram.h>

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/empty.hpp>

namespace drake_ros_introspection {

template<typename T>
class Introspector {
public:
  Introspector(
    std::vector<rclcpp::Node::SharedPtr> & nodes,
    std::vector<rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr> & publishers,
    std::vector<rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr> & subscriptions)
  : nodes_(nodes),
    publishers_(publishers),
    subscriptions_(subscriptions)
  {
  }

private:
  std::vector<rclcpp::Node::SharedPtr> nodes_;
  std::vector<rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr> publishers_;
  std::vector<rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr> subscriptions_;
};

}  // namespace drake_ros_introspection
