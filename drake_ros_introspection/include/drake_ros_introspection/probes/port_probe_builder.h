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

#include <drake_ros_introspection/probes/port_probe.h>

namespace drake_ros_introspection {
namespace probes {

// The interface to a PortProbe builder object.
// Instantiators of this interface must implement the DoBuild() private member function. This is
// where the functionality of constructing a port probe of the necessary type should go.
template <typename PortT>
class PortProbeBuilderInterface {
 public:
  // Subclassable
  virtual ~PortProbeBuilderInterface() = default;

  // Do the port probe construction, returning an object matching PortProbeInterface
  std::unique_ptr<PortProbeInterface<PortT>> Build(const PortT& port) {
    return DoBuild(port);
  }

 private:
  // Put the implementation of constructing the port probe here.
  virtual std::unique_ptr<PortProbeInterface<PortT>> DoBuild(
      const PortT& port) = 0;
};

// A basic port probe builder that builds objects of the PortProbe type.
template <typename PortT, typename ValueT, auto ValueConvention,
          typename MessageT>
class PortProbeBuilder : public PortProbeBuilderInterface<PortT> {
 private:
  // Construct a PortProbe object templated on the port type, the Drake value type, the matching
  // ROS message type, and the convention for value conversion.
  std::unique_ptr<PortProbeInterface<PortT>> DoBuild(
      const PortT& port) override {
    using PortProbeT = PortProbe<PortT, ValueT, ValueConvention, MessageT>;
    return std::make_unique<PortProbeT>(port);
  };
};

}  // namespace probes
}  // namespace drake_ros_introspection
