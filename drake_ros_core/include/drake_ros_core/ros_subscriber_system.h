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
#include <string>

#include <drake/systems/framework/leaf_system.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

#include "drake_ros_core/drake_ros_interface.h"
#include "drake_ros_core/serializer.h"
#include "drake_ros_core/serializer_interface.h"

namespace drake_ros_core {
/// System that subscribes to a ROS topic and makes it available on an output
/// port
class RosSubscriberSystem : public drake::systems::LeafSystem<double> {
 public:
  /// Convenience method to make a subscriber system given a ROS message type
  template <typename MessageT>
  static std::unique_ptr<RosSubscriberSystem> Make(
      const std::string& topic_name, const rclcpp::QoS& qos,
      DrakeRosInterface* ros_interface) {
    // Assume C++ typesupport since this is a C++ template function
    return std::make_unique<RosSubscriberSystem>(
        std::make_unique<Serializer<MessageT>>(), topic_name, qos,
        ros_interface);
  }

  RosSubscriberSystem(std::unique_ptr<SerializerInterface> serializer,
                      const std::string& topic_name, const rclcpp::QoS& qos,
                      DrakeRosInterface* ros_interface);

  virtual ~RosSubscriberSystem();

 protected:
  /// Override as a place to schedule event to move ROS message into a context
  void DoCalcNextUpdateTime(const drake::systems::Context<double>&,
                            drake::systems::CompositeEventCollection<double>*,
                            double*) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace drake_ros_core
