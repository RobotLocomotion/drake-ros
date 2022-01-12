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

template <typename T>
class ProbeInterface {
 public:
  virtual ~ProbeInterface() = default;

  void Configure(const std::shared_ptr<rclcpp::Node>& node) {
    return this->DoConfigure(node);
  }

  drake::systems::EventStatus Probe(const drake::systems::Context<T>& context) {
    return this->DoProbe(context);
  }

 private:
  virtual drake::systems::EventStatus DoProbe(
      const drake::systems::Context<T>& context) = 0;

  virtual void DoConfigure(const std::shared_ptr<rclcpp::Node>&) {}
};

}  // namespace probes
}  // namespace drake_ros_introspection
