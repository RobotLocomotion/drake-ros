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

#include <memory>

#include <drake/systems/framework/context.h>
#include <drake/systems/framework/event_status.h>
#include <rclcpp/rclcpp.hpp>

namespace drake_ros_introspection {
namespace probes {

// The interface for a probe.
// Instatiations of this interface must implement the DoProbe() private member function. This will
// be called by the Probe() public member function when Probe() is called by Drake during each
// simulation step.
// Instatiations should also implement the DoConfigure() private function if necessary. This is
// used to set up the probe's ROS aspects, such as declaring a publisher.
template <typename T>
class ProbeInterface {
 public:
  // Subclassable (in fact, must be subclassed)
  virtual ~ProbeInterface() = default;

  // Configure the ROS aspects of the probe. For example, create a publisher.
  void Configure(const std::shared_ptr<rclcpp::Node>& node) {
    return this->DoConfigure(node);
  }

  // Called by Drake during simulation after each simulation step.
  // This is used to give the probe a chance to do whatever it is that it does, such as publishing
  // an output port's value on a ROS topic, or inputting a value received from a ROS topic.
  drake::systems::EventStatus Probe(const drake::systems::Context<T>& context) {
    return this->DoProbe(context);
  }

 private:
  // Implementation of the Probe() function should be put here.
  // This is required
  virtual drake::systems::EventStatus DoProbe(
      const drake::systems::Context<T>& context) = 0;

  // Implementation of the Configure() function should be put here.
  // Not essential.
  virtual void DoConfigure(const std::shared_ptr<rclcpp::Node>&) {}
};

}  // namespace probes
}  // namespace drake_ros_introspection
