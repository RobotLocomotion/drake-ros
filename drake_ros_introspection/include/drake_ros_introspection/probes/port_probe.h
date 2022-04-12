// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#include <regex>
#include <sstream>
#include <string>

#include <drake/common/drake_throw.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/event_status.h>
#include <drake/systems/framework/framework_common.h>
#include <drake_ros_introspection/probes/probe_interface.h>
#include <drake_ros_introspection/utilities/port_traits.h>
#include <drake_ros_introspection/utilities/type_conversion.h>
#include <rclcpp/rclcpp.hpp>

namespace drake_ros_introspection {
namespace probes {

// The base interface for a PortProbe.
// A PortProbe is a specialised Probe that probes Drake system ports.
template <typename PortT>
class PortProbeInterface
    : public ProbeInterface<typename utilities::port_traits<PortT>::ScalarT> {
 public:
  // Construct a PortProbe for the provided Drake port.
  explicit PortProbeInterface(const PortT& port) : port_(port) {}

  // Subclassable
  virtual ~PortProbeInterface() = default;

  // Get the Drake port object probed by this PortProbe.
  const PortT& get_port() { return port_; }

 private:
  const PortT& port_;
};

// An object for probing Drake system ports.
// When configured, it will create a publisher to a topic named after the Drake port name. The
// publisher will publish messages of type MessageT, doing so each time the probe is called by
// Drake  (typically every simulation step).
template <typename PortT, typename ValueT, auto ValueConvention,
          typename MessageT>
class PortProbe : public PortProbeInterface<PortT> {
 public:
  using PortProbeInterface<PortT>::PortProbeInterface;

 private:
  using ScalarT = typename utilities::port_traits<PortT>::ScalarT;

  // Called (indirectly, due to the PortProbInterface) by Drake for each simulation step.
  // This function will get the value from the context provided by drake for the port that this
  // probe is for. It will then convert it to the ROS type using a conversion convention, and
  // publish that ROS value.
  drake::systems::EventStatus DoProbe(
      const drake::systems::Context<ScalarT>& context) override {
    DRAKE_THROW_UNLESS(publisher_ != nullptr);
    if (publisher_->get_subscription_count() > 0) {
      const PortT& port = this->get_port();
      const drake::systems::Context<ScalarT>& subcontext =
          port.get_system().GetMyContextFromRoot(context);
      using convert = utilities::convert<ValueT, ValueConvention, MessageT>;
      publisher_->publish(
          convert::to_message(port.template Eval<ValueT>(subcontext)));
      return drake::systems::EventStatus::Succeeded();
    }
    return drake::systems::EventStatus::DidNothing();
  }

  // Configures the PortProbe.
  // This function will construct a ROS publisher to match the port. The topic is named based on
  // the name of the Drake port.
  void DoConfigure(const std::shared_ptr<rclcpp::Node>& node) override {
    static std::regex path_separator_regex{
        drake::systems::internal::SystemMessageInterface::path_separator()};
    static std::regex invalid_characters_regex{R"-([^\w\d_~{}/])-"};
    const PortT& port = this->get_port();
    std::stringstream ss;
    ss << std::regex_replace(port.get_system().GetSystemPathname(),
                             path_separator_regex, "/");
    ss << "/" << port.get_name();
    const std::string topic_name =
        std::regex_replace(ss.str(), invalid_characters_regex, "_");
    publisher_ = node->create_publisher<MessageT>(topic_name, 1);
  }

  // The ROS publisher used to publish data recieved from the port.
  std::shared_ptr<rclcpp::Publisher<MessageT>> publisher_{nullptr};
};

}  // namespace probes
}  // namespace drake_ros_introspection
