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
#pragma once

#include <memory>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "drake_ros_core/drake_ros_interface.h"

namespace drake_ros_core {
/// System that abstracts working with ROS
class DrakeRos final : public DrakeRosInterface {
 public:
  DrakeRos() : DrakeRos("DrakeRosSystems", rclcpp::NodeOptions{}) {}

  DrakeRos(const std::string& node_name, rclcpp::NodeOptions node_options);

  ~DrakeRos() override;

  const rclcpp::Node& get_node() const final;

  rclcpp::Node* get_mutable_node() const final;

  void Spin(int timeout_millis) final;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace drake_ros_core
