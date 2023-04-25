#include "subscription.h"  // NOLINT(build/include)

#include <memory>
#include <string>

#include <rclcpp/version.h>

namespace drake_ros {
namespace core {
namespace internal {
namespace {
// Copied from rosbag2_transport rosbag2_get_subscription_options
rcl_subscription_options_t subscription_options(const rclcpp::QoS& qos) {
  auto options = rcl_subscription_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}
}  // namespace

Subscription::Subscription(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    const rosidl_message_type_support_t& ts, const std::string& topic_name,
    const rclcpp::QoS& qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
#if RCLCPP_VERSION_GTE(18, 0, 0)
    : rclcpp::SubscriptionBase(
          node_base, ts, topic_name, subscription_options(qos),
          /* event_callbacks */ {},
          /* use_default_callbacks */ true,
          /* delivered_message_kind */
          rclcpp::DeliveredMessageKind::SERIALIZED_MESSAGE),
#else
    : rclcpp::SubscriptionBase(node_base, ts, topic_name,
                               subscription_options(qos),
                               /* is_serialized */ true),
#endif
      callback_(callback) {
}

Subscription::~Subscription() {}

std::shared_ptr<void> Subscription::create_message() {
  // Subscriber only does serialized messages
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage>
Subscription::create_serialized_message() {
  return std::make_shared<rclcpp::SerializedMessage>();
}

void Subscription::handle_message(std::shared_ptr<void>& message,
                                  const rclcpp::MessageInfo& message_info) {
  handle_serialized_message(
      std::static_pointer_cast<rclcpp::SerializedMessage>(message),
      message_info);
}

void Subscription::handle_loaned_message(
    void* loaned_message, const rclcpp::MessageInfo& message_info) {
  (void)loaned_message;
  (void)message_info;
  throw std::runtime_error(
      "handle_loaned_message() not supported by drake_ros_core");
}

void Subscription::handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage>& message,
    const rclcpp::MessageInfo& message_info) {
  (void)message_info;
  callback_(message);
}

void Subscription::return_message(std::shared_ptr<void>& message) {
  auto serialized_msg_ptr =
      std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  return_serialized_message(serialized_msg_ptr);
}

void Subscription::return_serialized_message(
    std::shared_ptr<rclcpp::SerializedMessage>& message) {
  message.reset();
}
}  // namespace internal
}  // namespace core
}  // namespace drake_ros
