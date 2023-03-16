#pragma once

#include <unordered_set>

#include <drake/systems/framework/diagram.h>
#include <rclcpp/qos.hpp>

namespace drake_ros {
namespace viz {

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

/// Quality of service settings for markers topics
const rclcpp::Duration kMarkerLifetime{rclcpp::Duration::from_nanoseconds(1e9)};
}  // namespace viz
}  // namespace drake_ros
