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

template <typename PortT>
struct PortProbingRule {
  std::function<bool(const PortT&)> matches;
  std::unique_ptr<probes::PortProbeBuilderInterface<PortT>> builder{};
};

template <typename PortT, typename ValueT, auto ValueTConvention>
class PortProbingRuleWriter {
 public:
  explicit PortProbingRuleWriter(PortProbingRule<PortT>& rule) : rule_(rule) {}

  template <typename MessageT>
  void Publish() {
    using PortProbeBuilderT =
        probes::PortProbeBuilder<PortT, ValueT, ValueTConvention, MessageT>;
    rule_.builder = std::make_unique<PortProbeBuilderT>();
  }

 private:
  PortProbingRule<PortT>& rule_;
};

template <typename PortT>
class AbstractPortProbingRuleWriter {
 public:
  explicit AbstractPortProbingRuleWriter(PortProbingRule<PortT>& rule)
      : rule_(rule) {}

  template <typename ValueT,
            auto ValueTConvention = utilities::BuiltinTypeConventions::Default>
  PortProbingRuleWriter<PortT, ValueT, ValueTConvention> Expect() {
    return PortProbingRuleWriter<PortT, ValueT, ValueTConvention>(rule_);
  }

  template <template <typename> typename Value,
            auto ValueConvention = utilities::BuiltinTypeConventions::Default,
            typename ScalarT = typename utilities::port_traits<PortT>::ScalarT>
  PortProbingRuleWriter<PortT, Value<ScalarT>, ValueConvention> Expect() {
    return Expect<Value<ScalarT>, ValueConvention>();
  }

  template <typename MessageT>
  void Publish() {
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
