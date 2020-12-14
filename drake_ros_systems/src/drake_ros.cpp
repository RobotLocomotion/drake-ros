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
#include <drake_ros_systems/drake_ros.hpp>
#include "publisher.hpp"
#include "subscription.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

namespace drake_ros_systems
{
class DrakeRosPrivate
{
public:
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::UniquePtr node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
};

DrakeRos::DrakeRos()
  : impl_(new DrakeRosPrivate())
{
  impl_->context_ = std::make_shared<rclcpp::Context>();

  // TODO(sloretz) allow passing args and init options in constructor
  impl_->context_->init(0, nullptr);

  // TODO(sloretz) allow passing in node name and node options
  rclcpp::NodeOptions no;
  no.context(impl_->context_);
  impl_->node_.reset(new rclcpp::Node("DrakeRosSystems", no));

  // TODO(sloretz) allow passing in executor options
  rclcpp::ExecutorOptions eo;
  eo.context = impl_->context_;
  impl_->executor_.reset(new rclcpp::executors::SingleThreadedExecutor(eo));

  impl_->executor_->add_node(impl_->node_->get_node_base_interface());
}

DrakeRos::~DrakeRos()
{
  impl_->context_->shutdown("~DrakeRos()");
}

std::unique_ptr<Publisher>
DrakeRos::create_publisher(
  const rosidl_message_type_support_t & ts,
  const std::string & topic_name,
  const rclcpp::QoS & qos)
{
  return std::make_unique<Publisher>(
    impl_->node_->get_node_base_interface().get(), ts, topic_name, qos);
}

std::shared_ptr<Subscription>
DrakeRos::create_subscription(
  const rosidl_message_type_support_t & ts,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
{
  auto sub = std::make_shared<Subscription>(
    impl_->node_->get_node_base_interface().get(), ts, topic_name, qos, callback);
  impl_->node_->get_node_topics_interface()->add_subscription(sub, nullptr);

  // TODO(sloretz) return unique pointer to subscription and make subscription
  // automatically unsubscribe when it's deleted

  return sub;
}

void
DrakeRos::spin(
  int timeout_millis)
{
  impl_->executor_->spin_some(std::chrono::milliseconds(timeout_millis));
}
}  // namespace drake_ros_systems
