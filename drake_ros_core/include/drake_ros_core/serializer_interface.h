// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#pragma once

#include <memory>

#include <drake/common/value.h>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace drake_ros_core {
class SerializerInterface {
 public:
  virtual rclcpp::SerializedMessage Serialize(
      const drake::AbstractValue& abstract_value) const = 0;

  virtual void Deserialize(const rclcpp::SerializedMessage& message,
                           drake::AbstractValue* abstract_value) const = 0;

  virtual std::unique_ptr<drake::AbstractValue> CreateDefaultValue() const = 0;

  virtual const rosidl_message_type_support_t* GetTypeSupport() const = 0;
};
}  // namespace drake_ros_core
