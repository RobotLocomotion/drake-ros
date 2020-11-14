#ifndef DRAKE_ROS_SYSTEMS__SUBSCRIPTION_HPP_
#define DRAKE_ROS_SYSTEMS__SUBSCRIPTION_HPP_

#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <memory>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_base.hpp>

namespace drake_ros_systems
{
class Subscription final : public rclcpp::SubscriptionBase
{
public:
  Subscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback);

  ~Subscription();

protected:
  /// Borrow a new message.
  /** \return Shared pointer to the fresh message. */
  std::shared_ptr<void>
  create_message() override;

  /// Borrow a new serialized message
  /** \return Shared pointer to a rcl_message_serialized_t. */
  std::shared_ptr<rclcpp::SerializedMessage>
  create_serialized_message() override;

  /// Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  void
  handle_message(
    std::shared_ptr<void> & message,
    const rclcpp::MessageInfo & message_info) override;

  void
  handle_loaned_message(
    void * loaned_message, const rclcpp::MessageInfo & message_info) override;

  /// Return the message borrowed in create_message.
  /** \param[in] message Shared pointer to the returned message. */
  void
  return_message(std::shared_ptr<void> & message) override;

  /// Return the message borrowed in create_serialized_message.
  /** \param[in] message Shared pointer to the returned message. */
  void
  return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override;

private:
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__SUBSCRIPTION_HPP_
