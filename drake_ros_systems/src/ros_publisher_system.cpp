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
#include <drake_ros_systems/ros_publisher_system.hpp>
#include <drake_ros_systems/serializer_interface.hpp>

#include "publisher.hpp"

namespace drake_ros_systems
{
class RosPublisherSystemPrivate
{
public:
  std::unique_ptr<SerializerInterface> serializer_;
  std::unique_ptr<Publisher> pub_;
};

RosPublisherSystem::RosPublisherSystem(
  std::unique_ptr<SerializerInterface> & serializer,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  std::shared_ptr<DrakeRosInterface> ros)
: impl_(new RosPublisherSystemPrivate())
{
  impl_->serializer_ = std::move(serializer);
  impl_->pub_ = ros->create_publisher(
    *impl_->serializer_->get_type_support(), topic_name, qos);

  DeclareAbstractInputPort("message", *(impl_->serializer_->create_default_value()));

  // TODO(sloretz) customizable triggers like lcm system
  DeclarePerStepEvent(
    drake::systems::PublishEvent<double>(
      [this](
        const drake::systems::Context<double> & context,
        const drake::systems::PublishEvent<double> &) {
        publish_input(context);
      }));
}

RosPublisherSystem::~RosPublisherSystem()
{
}

void
RosPublisherSystem::publish(const rclcpp::SerializedMessage & serialized_msg)
{
  impl_->pub_->publish(serialized_msg);
}

void
RosPublisherSystem::publish_input(const drake::systems::Context<double> & context)
{
  const drake::AbstractValue & input = get_input_port().Eval<drake::AbstractValue>(context);
  impl_->pub_->publish(impl_->serializer_->serialize(input));
}
}  // namespace drake_ros_systems
