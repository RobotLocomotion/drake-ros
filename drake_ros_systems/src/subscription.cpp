#include "subscription.hpp"

namespace drake_ros_systems
{
// Copied from rosbag2_transport rosbag2_get_subscription_options
rcl_subscription_options_t subscription_options(const rclcpp::QoS & qos)
{
  auto options = rcl_subscription_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}

Subscription::Subscription(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const rosidl_message_type_support_t & ts,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback)
: rclcpp::SubscriptionBase(node_base, ts, topic_name, subscription_options(qos), true),
  callback_(callback)
{
}

Subscription::~Subscription()
{
}
}  // namespace drake_ros_systems

