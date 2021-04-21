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
#ifndef DRAKE_ROS_SYSTEMS__RVIZ_VISUALIZER_HPP_
#define DRAKE_ROS_SYSTEMS__RVIZ_VISUALIZER_HPP_

#include <drake/systems/framework/diagram.h>

#include <memory>
#include <unordered_set>

#include "drake_ros_systems/drake_ros_interface.hpp"


namespace drake_ros_systems {

class RvizVisualizer : public drake::systems::Diagram<double>
{
public:
  RvizVisualizer(
    std::shared_ptr<DrakeRosInterface> ros_interface,
    const std::unordered_set<drake::systems::TriggerType> & publish_triggers = {
      drake::systems::TriggerType::kForced, drake::systems::TriggerType::kPeriodic},
    double publish_period = 0.05,
    bool publish_tf = true);
};

}  // namespace drake_ros_systems

#endif  // DRAKE_ROS_SYSTEMS__RVIZ_VISUALIZER_HPP_
