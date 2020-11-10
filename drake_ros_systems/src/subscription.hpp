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
    std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback);

  ~Subscription();

private:
  std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__SUBSCRIPTION_HPP_


