#include "drake_ros_core/clock_system.h"

#include <rclcpp/time.hpp>

using drake_ros_core::ClockSystem;

ClockSystem::ClockSystem() {
  DeclareAbstractOutputPort("clock", &ClockSystem::CalcClock);
}

ClockSystem::~ClockSystem() {}

void ClockSystem::CalcClock(const drake::systems::Context<double>& context,
                            rosgraph_msgs::msg::Clock* output_value) const {
  rclcpp::Time now;
  now += rclcpp::Duration::from_seconds(context.get_time());
  output_value->clock = now;
};
