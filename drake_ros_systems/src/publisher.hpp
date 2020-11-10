#ifndef DRAKE_ROS_SYSTEMS__PUBLISHER_HPP_
#define DRAKE_ROS_SYSTEMS__PUBLISHER_HPP_

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <memory>
#include <string>

#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>

namespace drake_ros_systems
{
class Publisher final : public rclcpp::PublisherBase
{
public:
  Publisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    const rclcpp::QoS & qos);

  ~Publisher();

  void
  publish(const rclcpp::SerializedMessage & serialized_msg);
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__PUBLISHER_HPP_
