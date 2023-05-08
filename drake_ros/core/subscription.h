#pragma once

#include <memory>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/version.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

namespace drake_ros {
namespace core {
namespace internal {
// A type-erased version of rclcpp:::Subscription<Message>.
// This class conforms to the ROS 2 C++ style for consistency.
class Subscription final : public rclcpp::SubscriptionBase {
 public:
  Subscription(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      const rosidl_message_type_support_t& ts, const std::string& topic_name,
      const rclcpp::QoS& qos,
      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback);

  ~Subscription();

#if RCLCPP_VERSION_GTE(18, 0, 0)
  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
  get_shared_dynamic_message_type() override {
    throw rclcpp::exceptions::UnimplementedError(
        "get_shared_dynamic_message_type is not implemented for Subscription");
  }

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
  get_shared_dynamic_message() override {
    throw rclcpp::exceptions::UnimplementedError(
        "get_shared_dynamic_message is not implemented for Subscription");
  }

  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override {
    throw rclcpp::exceptions::UnimplementedError(
        "get_shared_dynamic_serialization_support is not implemented for "
        "Subscription");
  }

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
  create_dynamic_message() override {
    throw rclcpp::exceptions::UnimplementedError(
        "create_dynamic_message is not implemented for Subscription");
  }

  void return_dynamic_message(
      rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr& message)
      override {
    (void)message;
    throw rclcpp::exceptions::UnimplementedError(
        "return_dynamic_message is not implemented for Subscription");
  }

  void handle_dynamic_message(
      const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr& message,
      const rclcpp::MessageInfo& message_info) override {
    (void)message;
    (void)message_info;
    throw rclcpp::exceptions::UnimplementedError(
        "handle_dynamic_message is not implemented for Subscription");
  }
#endif

 protected:
  // Borrow a new message.
  /** \return Shared pointer to the fresh message. */
  std::shared_ptr<void> create_message() override;

  // Borrow a new serialized message
  /** \return Shared pointer to a rcl_message_serialized_t. */
  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message()
      override;

  // Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  void handle_message(std::shared_ptr<void>& message,
                      const rclcpp::MessageInfo& message_info) override;

  void handle_loaned_message(void* loaned_message,
                             const rclcpp::MessageInfo& message_info) override;

  void handle_serialized_message(
      const std::shared_ptr<rclcpp::SerializedMessage>& message,
      const rclcpp::MessageInfo& message_info) override;

  // Return the message borrowed in create_message.
  /** \param[in] message Shared pointer to the returned message. */
  void return_message(std::shared_ptr<void>& message) override;

  // Return the message borrowed in create_serialized_message.
  /** \param[in] message Shared pointer to the returned message. */
  void return_serialized_message(
      std::shared_ptr<rclcpp::SerializedMessage>& message) override;

 private:
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback_;
};
}  // namespace internal
}  // namespace core
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_core = drake_ros::core;
