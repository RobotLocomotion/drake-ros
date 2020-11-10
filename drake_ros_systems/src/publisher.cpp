#include "publisher.hpp"

namespace drake_ros_systems
{
// Copied from rosbag2_transport rosbag2_get_publisher_options
rcl_publisher_options_t publisher_options(const rclcpp::QoS & qos)
{
  auto options = rcl_publisher_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}

Publisher::Publisher(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const rosidl_message_type_support_t & type_support,
  const std::string & topic_name,
  const rclcpp::QoS & qos)
: rclcpp::PublisherBase(node_base, topic_name, type_support, publisher_options(qos))
{
}

Publisher::~Publisher()
{
}

void
Publisher::publish(const rclcpp::SerializedMessage & serialized_msg)
{
  // TODO(sloretz) Copied from rosbag2_transport GenericPublisher, can it be upstreamed to rclcpp?
  auto return_code = rcl_publish_serialized_message(
    get_publisher_handle().get(), &serialized_msg.get_rcl_serialized_message(), NULL);

  if (return_code != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
  }
}
}  // namespace drake_ros_systems
