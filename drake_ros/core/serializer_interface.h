#pragma once

#include <memory>

#include <drake/common/value.h>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace drake_ros {
namespace core {
/** An interface for (de)serialization of a ROS message.

 This interface deals with ROS messages of a fixed message
 type, which cannot be inferred from its API. Therefore,
 caller code must either know what the type is or otherwise
 rely on type-erased ROS APIs that can take ROS message
 typesupport.
 */
class SerializerInterface {
 public:
  virtual ~SerializerInterface() = default;

  /** Serializes a ROS message of a given type.
   @param[in] abstract_value type-erased value
     wrapping the ROS message to be serialized.
   @return the serialized ROS message.
   */
  virtual rclcpp::SerializedMessage Serialize(
      const drake::AbstractValue& abstract_value) const = 0;

  /** Deserializes a ROS message of a given type.
   @param[in] message the serialized ROS message.
   @param[inout] abstract_value type-erased value wrapping
     the ROS message to be populated upon deserialization.
   */
  virtual void Deserialize(const rclcpp::SerializedMessage& message,
                           drake::AbstractValue* abstract_value) const = 0;

  /** Creates a default ROS message wrapped in an abstract, type-erased
   value. */
  virtual std::unique_ptr<drake::AbstractValue> CreateDefaultValue() const = 0;

  /** Returns a reference to the ROS message typesupport. */
  virtual const rosidl_message_type_support_t* GetTypeSupport() const = 0;
};
}  // namespace core
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_core = drake_ros::core;
