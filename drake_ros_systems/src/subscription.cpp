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
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
: rclcpp::SubscriptionBase(node_base, ts, topic_name, subscription_options(qos), true),
  callback_(callback)
{
}

Subscription::~Subscription()
{
}

std::shared_ptr<void>
Subscription::create_message()
{
  // Subscriber only does serialized messages
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage>
Subscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>();
}

void
Subscription::handle_message(
  std::shared_ptr<void> & message,
  const rclcpp::MessageInfo & message_info)
{
  (void) message_info;
  callback_(std::static_pointer_cast<rclcpp::SerializedMessage>(message));
}

void
Subscription::handle_loaned_message(
  void * loaned_message, const rclcpp::MessageInfo & message_info)
{
  (void)loaned_message;
  (void)message_info;
  throw std::runtime_error("handle_loaned_message() not supported by drake_ros_systems");
}

void
Subscription::return_message(std::shared_ptr<void> & message)
{
  auto serialized_msg_ptr = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  return_serialized_message(serialized_msg_ptr);
}

void
Subscription::return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message)
{
  message.reset();
}
}  // namespace drake_ros_systems

