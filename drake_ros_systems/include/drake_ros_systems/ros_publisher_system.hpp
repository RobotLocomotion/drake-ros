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
#ifndef DRAKE_ROS_SYSTEMS__ROS_PUBLISHER_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__ROS_PUBLISHER_SYSTEM_HPP_

#include <drake/systems/framework/leaf_system.h>
#include <rmw/rmw.h>

#include <rclcpp/serialized_message.hpp>

#include <memory>
#include <string>

#include "drake_ros_systems/drake_ros_interface.hpp"
#include "drake_ros_systems/serializer.hpp"
#include "drake_ros_systems/serializer_interface.hpp"


namespace drake_ros_systems
{
/// PIMPL forward declaration
class RosPublisherSystemPrivate;

/// Accepts ROS messages on an input port and publishes them to a ROS topic
class RosPublisherSystem : public drake::systems::LeafSystem<double>
{
public:
  /// Convenience method to make a publisher system given a ROS message type
  template<typename MessageT>
  static
  std::unique_ptr<RosPublisherSystem>
  Make(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::shared_ptr<DrakeRosInterface> ros_interface)
  {
    // Assume C++ typesupport since this is a C++ template function
    std::unique_ptr<SerializerInterface> serializer = std::make_unique<Serializer<MessageT>>();
    return std::make_unique<RosPublisherSystem>(serializer, topic_name, qos, ros_interface);
  }

  RosPublisherSystem(
    std::unique_ptr<SerializerInterface> & serializer,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::shared_ptr<DrakeRosInterface> ros_interface);

  virtual ~RosPublisherSystem();

  /// Convenience method to publish a C++ ROS message
  template<typename MessageT>
  void
  publish(const MessageT & message)
  {
    static const Serializer<MessageT> serializer;
    publish(serializer->serialize(message));
  }

  /// Publish a serialized ROS message
  void
  publish(const rclcpp::SerializedMessage & serialized_msg);

protected:
  void
  publish_input(const drake::systems::Context<double> & context);

  std::unique_ptr<RosPublisherSystemPrivate> impl_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__ROS_PUBLISHER_SYSTEM_HPP_
