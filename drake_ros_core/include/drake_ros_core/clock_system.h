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

#include <drake/systems/framework/leaf_system.h>
#include <rosgraph_msgs/msg/clock.hpp>

namespace drake_ros_core {
/** A system that convert's drake's time to a rosgraph_msgs/msg/Clock message.
 */
class ClockSystem : public drake::systems::LeafSystem<double> {
 public:
  /** A constructor for the clock system.
   */
  ClockSystem();

  ~ClockSystem() override;

 protected:
  void CalcClock(const drake::systems::Context<double>& context,
                 rosgraph_msgs::msg::Clock* output_value) const;
};
}  // namespace drake_ros_core
