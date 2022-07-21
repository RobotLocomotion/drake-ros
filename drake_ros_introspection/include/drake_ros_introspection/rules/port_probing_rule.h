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

#include <functional>
#include <memory>
#include <utility>

#include <drake/common/value.h>
#include <drake_ros_introspection/probes/port_probe_builder.h>
#include <drake_ros_introspection/utilities/port_traits.h>
#include <drake_ros_introspection/utilities/type_conversion.h>

namespace drake_ros_introspection {
namespace rules {

// A rule for matching a Drake port.
// Contains the predicate function object that is called (and passed an instance of the port type,
// PortT) to check if a port matches this rule.
// Contains an instance of a PortProbeBuilderInterface that is used to construct a port probe for
// matching ports. This enables the use of different builders for different port types.
template <typename PortT>
struct PortProbingRule {
  std::function<bool(const PortT&)> matches;
  std::unique_ptr<probes::PortProbeBuilderInterface<PortT>> builder{};
};

// This object is used to convert a port probing rule into a publishing-type rule for Drake ports
// that have the BasicVector type.
// In other words, it adds to a PortProbingRule a port probe builder that constructs a ROS topic
// publisher for the Drake port.
template <typename PortT, typename ValueT, auto ValueTConvention>
class PortProbingRuleWriter {
 public:
  // Construct the writer by passing in a builder-less PortProbingRule that already has a predicate
  // in it.
  explicit PortProbingRuleWriter(PortProbingRule<PortT>& rule) : rule_(rule) {}

  // Call this function to add a publishing port probe to the PortProbingRule.
  template <typename MessageT>
  void PublishRosType() {
    using PortProbeBuilderT =
        probes::PortProbeBuilder<PortT, ValueT, ValueTConvention, MessageT>;
    rule_.builder = std::make_unique<PortProbeBuilderT>();
  }

 private:
  // The PortProbingRule to which the builder is added.
  PortProbingRule<PortT>& rule_;
};

// This object is used to convert a port probing rule into a publishing-type rule for Drake ports
// that have the AbstractValue type.
// In other words, it adds to a PortProbingRule a port probe builder that constructs a ROS topic
// publisher for the Drake port.
// It can be converted into a PortProbingRuleWriter for BasicVector type Drake ports by using the
// ReceiveDrakeType() member function.
template <typename PortT>
class AbstractPortProbingRuleWriter {
 public:
  // Construct the writer by passing in a builder-less PortProbingRule.
  explicit AbstractPortProbingRuleWriter(PortProbingRule<PortT>& rule)
      : rule_(rule) {}

  // Convert the AbstractPortProbingRuleWriter into a PortProbingRuleWriter by expecting the Drake
  // port to use a particular port type (typically, BasicVector).
  // ValueT is the type of the Drake port.
  template <typename ValueT,
            auto ValueTConvention = utilities::BuiltinTypeConventions::Default>
  PortProbingRuleWriter<PortT, ValueT, ValueTConvention>
  ReceiveDrakeType() {
    return PortProbingRuleWriter<PortT, ValueT, ValueTConvention>(rule_);
  }

  // Convert the AbstractPortProbingRuleWriter into a PortProbingRuleWriter by expecting the Drake
  // port to use a particular port type.
  // Value is the Drake port type (not BasicVector or similar, but drake::systems::OutputPort or
  // similar). The data type of that port is extracted from it using the port_traits.
  template <template <typename> typename Value,
            auto ValueConvention = utilities::BuiltinTypeConventions::Default,
            typename ScalarT = typename utilities::port_traits<PortT>::ScalarT>
  PortProbingRuleWriter<PortT, Value<ScalarT>, ValueConvention>
  ReceiveDrakeType() {
    return ReceiveDrakeType<Value<ScalarT>, ValueConvention>();
  }

  // Call this function to add a publishing port probe builder to the PortProbingRule. The port
  // probe builder constructs a port probe that publishes a Drake AbstractValue.
  template <typename MessageT>
  void PublishRosType() {
    using ValueT = drake::AbstractValue;
    constexpr auto ValueTConvention =
        utilities::BuiltinTypeConventions::Default;
    using PortProbeBuilderT =
        probes::PortProbeBuilder<PortT, ValueT, ValueTConvention, MessageT>;
    rule_.builder = std::make_unique<PortProbeBuilderT>();
  }

 private:
  PortProbingRule<PortT>& rule_;
};

}  // namespace rules
}  // namespace drake_ros_introspection
