// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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
      get_type_support(),
      &serialized_msg.get_rcl_serialized_message());
    if (ret != RMW_RET_OK) {
      // TODO(sloretz) do something if serialization fails
      (void)ret;
    }
    return serialized_msg;
  }

  bool
  deserialize(
    const rclcpp::SerializedMessage & serialized_message,
    drake::AbstractValue & abstract_value) const override
  {
    const auto ret = rmw_deserialize(
      &serialized_message.get_rcl_serialized_message(),
      get_type_support(),
      &abstract_value.get_mutable_value<MessageT>());
    return ret == RMW_RET_OK;
  }

  std::unique_ptr<drake::AbstractValue>
  create_default_value() const override
  {
    return std::make_unique<drake::Value<MessageT>>(MessageT());
  }

  const rosidl_message_type_support_t *
  get_type_support() const override
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
  }
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__SERIALIZER_HPP_
