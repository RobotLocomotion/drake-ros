#include "drake_ros/core/clock_system.h"

#include <rclcpp/time.hpp>

using drake_ros::core::ClockSystem;
using drake_ros::core::RosPublisherSystem;

ClockSystem::ClockSystem() {
  DeclareAbstractOutputPort("clock", &ClockSystem::CalcClock);
}

ClockSystem::~ClockSystem() {}

void ClockSystem::CalcClock(const drake::systems::Context<double>& context,
                            rosgraph_msgs::msg::Clock* output_value) const {
  rclcpp::Time now{0, 0, RCL_ROS_TIME};
  now += rclcpp::Duration::from_seconds(context.get_time());
  output_value->clock = now;
}

std::tuple<ClockSystem*, RosPublisherSystem*> ClockSystem::AddToBuilder(
    drake::systems::DiagramBuilder<double>* builder, DrakeRos* ros,
    const std::string& topic_name, const rclcpp::QoS& qos,
    const std::unordered_set<drake::systems::TriggerType>& publish_triggers,
    double publish_period) {
  auto* clock_system = builder->AddSystem<ClockSystem>();

  auto* pub_system =
      builder->AddSystem(RosPublisherSystem::Make<rosgraph_msgs::msg::Clock>(
          topic_name, qos, ros, publish_triggers, publish_period));

  builder->Connect(clock_system->get_output_port(),
                   pub_system->get_input_port());

  return {clock_system, pub_system};
}
