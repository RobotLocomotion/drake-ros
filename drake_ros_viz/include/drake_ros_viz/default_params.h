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

#include <unordered_set>

#include <drake/systems/framework/diagram.h>
#include <rclcpp/qos.hpp>

namespace drake_ros_viz {

/// Publish triggers for scene markers
const std::unordered_set<drake::systems::TriggerType> kDefaultPublishTriggers{
      drake::systems::TriggerType::kForced,
      drake::systems::TriggerType::kPeriodic};

/// Period for periodic markers and tf broadcasting.
/// To help avoid small timesteps, use a default period that has an exact
/// representation in binary floating point (see drake#15021).
const double kDefaultPublishPeriod{1.0 / 32.0};

/// Quality of service settings for markers topics
const rclcpp::QoS kDefaultMarkersQos(1);
}  // namespace drake_ros_viz
