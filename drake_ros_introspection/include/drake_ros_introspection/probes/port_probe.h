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

template <typename PortT>
class PortProbeInterface
    : public ProbeInterface<typename utilities::port_traits<PortT>::ScalarT> {
 public:
  explicit PortProbeInterface(const PortT& port) : port_(port) {}

  virtual ~PortProbeInterface() = default;

  const PortT& get_port() { return port_; }

 private:
  const PortT& port_;
};

template <typename PortT, typename ValueT, auto ValueConvention,
          typename MessageT>
class PortProbe : public PortProbeInterface<PortT> {
 public:
  using PortProbeInterface<PortT>::PortProbeInterface;

 private:
  using ScalarT = typename utilities::port_traits<PortT>::ScalarT;

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

  std::shared_ptr<rclcpp::Publisher<MessageT>> publisher_{nullptr};
};

}  // namespace probes
}  // namespace drake_ros_introspection
