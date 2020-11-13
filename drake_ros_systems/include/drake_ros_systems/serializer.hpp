#ifndef DRAKE_ROS_SYSTEMS__SERIALIZER_HPP_
#define DRAKE_ROS_SYSTEMS__SERIALIZER_HPP_

#include <drake/common/value.h>
#include <rmw/rmw.h>

#include <drake_ros_systems/serializer_interface.hpp>

#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace drake_ros_systems
{
template <typename MessageT>
class Serializer : public SerializerInterface
{
public:
  rclcpp::SerializedMessage
  serialize(const drake::AbstractValue & abstract_value) const override
  {
    rclcpp::SerializedMessage serialized_msg;
    const MessageT & message = abstract_value.get_value<MessageT>();
    const auto ret = rmw_serialize(
      &message,
      rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
      &serialized_msg.get_rcl_serialized_message());
    if (ret != RMW_RET_OK) {
      // TODO(sloretz) do something if serialization fails
      (void)ret;
    }
    return serialized_msg;
  }

  void
  deserialize(
    const rclcpp::SerializedMessage & message,
    drake::AbstractValue & abstract_value) const override
  {
    // TODO
    (void) message;
    (void) abstract_value;
  }

  std::unique_ptr<drake::AbstractValue>
  create_default_value() const override
  {
    return std::make_unique<drake::Value<MessageT>>(MessageT());
  }
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__SERIALIZER_HPP_
