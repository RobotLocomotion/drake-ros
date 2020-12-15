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
#ifndef DRAKE_ROS_SYSTEMS__SERIALIZER_INTERFACE_HPP_
#define DRAKE_ROS_SYSTEMS__SERIALIZER_INTERFACE_HPP_

#include <drake/common/value.h>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

#include <memory>

namespace drake_ros_systems
{
class SerializerInterface
{
public:
  virtual
  rclcpp::SerializedMessage
  serialize(const drake::AbstractValue & abstract_value) const = 0;

  virtual
  bool
  deserialize(
    const rclcpp::SerializedMessage & message,
    drake::AbstractValue & abstract_value) const = 0;

  virtual
  std::unique_ptr<drake::AbstractValue>
  create_default_value() const = 0;

  virtual
  const rosidl_message_type_support_t *
  get_type_support() const = 0;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__SERIALIZER_INTERFACE_HPP_
