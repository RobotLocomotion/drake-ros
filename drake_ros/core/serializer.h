#pragma once

#include <memory>

#include <drake/common/value.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

#include "drake_ros/core/serializer_interface.h"

namespace drake_ros {
namespace core {
/** A (de)serialization interface implementation that is
 bound to C++ ROS messages of `MessageT` type. */
template <typename MessageT>
class Serializer : public SerializerInterface {
 public:
  rclcpp::SerializedMessage Serialize(
      const drake::AbstractValue& abstract_value) const override {
    rclcpp::SerializedMessage serialized_message;
    protocol_.serialize_message(&abstract_value.get_value<MessageT>(),
                                &serialized_message);
    return serialized_message;
  }

  void Deserialize(const rclcpp::SerializedMessage& serialized_message,
                   drake::AbstractValue* abstract_value) const override {
    protocol_.deserialize_message(
        &serialized_message, &abstract_value->get_mutable_value<MessageT>());
  }

  std::unique_ptr<drake::AbstractValue> CreateDefaultValue() const override {
    return std::make_unique<drake::Value<MessageT>>(MessageT());
  }

  const rosidl_message_type_support_t* GetTypeSupport() const override {
    return rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
  }

 private:
  rclcpp::Serialization<MessageT> protocol_;
};
}  // namespace core
}  // namespace drake_ros
