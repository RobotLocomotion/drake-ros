#pragma once

#include <memory>
#include <string>

#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>

namespace drake_ros {
namespace core {
namespace internal {
// A type-erased version of rclcpp:::Publisher<Message>.
// This class conforms to the ROS 2 C++ style for consistency.
class Publisher final : public rclcpp::PublisherBase {
 public:
  Publisher(rclcpp::node_interfaces::NodeBaseInterface* node_base,
            const rosidl_message_type_support_t& ts,
            const std::string& topic_name, const rclcpp::QoS& qos);

  ~Publisher();

  void publish(const rclcpp::SerializedMessage& serialized_msg);
};
}  // namespace internal
}  // namespace core
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_core = drake_ros::core;
