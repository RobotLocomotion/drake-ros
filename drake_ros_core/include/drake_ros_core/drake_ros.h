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

namespace drake_ros_core {

/** A Drake ROS interface that wraps a live ROS2 node.
 This interface manages both ROS2 node construction and scheduling.
 */
class DrakeRos final {
 public:
  /** A constructor that wraps a "drake_ros" ROS2 node with default options. */
  DrakeRos() : DrakeRos("drake_ros", rclcpp::NodeOptions{}) {}

  /** A constructor that wraps a `node_name` ROS2 node with `node_options`.
   See `rclcpp::Node` documentation for further reference on arguments.
   */
  DrakeRos(const std::string& node_name, rclcpp::NodeOptions node_options);

  ~DrakeRos();

  /** Returns a constant reference to the underlying ROS2 node. */
  const rclcpp::Node& get_node() const;

  /** Returns a mutable reference to the underlying ROS2 node. */
  rclcpp::Node* get_mutable_node() const;

  /** Spins the underlying ROS2 node, dispatching all available work.
   @param[in] timeout_millis Timeout, in milliseconds.
     If timeout is less than 0, the call will block indefinitely
     until some work has been dispatched. If timeout is 0, the call
     will dispatch available work without blocking. If timeout is
     larger than 0, the call will wait up to the given timeout for
     work to dispatch.
   */
  void Spin(int timeout_millis = 0);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace drake_ros_core
