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
#include <utility>
#include <vector>

#include <drake/systems/framework/context.h>
#include <drake/systems/framework/event_status.h>
#include <drake_ros_introspection/probes/probe_interface.h>

namespace drake_ros_introspection {

template <typename T>
class SimulatorMonitor {
 public:
  explicit SimulatorMonitor(
      std::vector<std::unique_ptr<probes::ProbeInterface<T>>> probes)
      : probes_(std::move(probes)) {}

  void Configure(const std::shared_ptr<rclcpp::Node>& node) {
    for (auto& probe : probes_) {
      probe->Configure(node);
    }
  }

  operator std::function<
      drake::systems::EventStatus(const drake::systems::Context<T>&)>() {
    return [this](const drake::systems::Context<T>& context) {
      auto status = drake::systems::EventStatus::DidNothing();
      for (auto& probe : probes_) {
        status.KeepMoreSevere(probe->Probe(context));
      }
      return status;
    };
  }

 private:
  std::vector<std::unique_ptr<probes::ProbeInterface<T>>> probes_;
};

}  // namespace drake_ros_introspection
