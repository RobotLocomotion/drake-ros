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

#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/output_port.h>

namespace drake_ros_introspection {
namespace utilities {
template <typename PortT>
struct port_traits;

// Provides the type of an input port as an accessible member, ScalarT.
// For example, if the port is InputPort<double>, ScalarT will be double.
template <typename T>
struct port_traits<drake::systems::InputPort<T>> {
  using ScalarT = T;
};

// Provides the type of an output port as an accessible member, ScalarT.
// For example, if the port is OutputPort<double>, ScalarT will be double.
template <typename T>
struct port_traits<drake::systems::OutputPort<T>> {
  using ScalarT = T;
};

}  // namespace utilities
}  // namespace drake_ros_introspection
