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
#include <utility>

#include <drake/common/value.h>

namespace drake_ros_introspection {
namespace utilities {

template <typename ValueT, auto ValueTConvention, typename MessageT>
struct convert;

template <typename ValueT, typename MessageT>
struct conventional_convert;

enum class BuiltinTypeConventions { Default };

template <typename ValueT, typename MessageT>
struct convert<ValueT, BuiltinTypeConventions::Default, MessageT>
    : public conventional_convert<ValueT, MessageT> {};

template <typename MessageT>
struct conventional_convert<drake::AbstractValue, MessageT> {
  static std::unique_ptr<MessageT> to_message(
      const drake::AbstractValue& value) {
    return std::make_unique<MessageT>(value.get_value<MessageT>());
  }
};

}  // namespace utilities
}  // namespace drake_ros_introspection
