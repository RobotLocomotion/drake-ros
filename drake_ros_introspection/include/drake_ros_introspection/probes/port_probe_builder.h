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

template <typename PortT>
class PortProbeBuilderInterface {
 public:
  virtual ~PortProbeBuilderInterface() = default;

  std::unique_ptr<PortProbeInterface<PortT>> Build(const PortT& port) {
    return DoBuild(port);
  }

 private:
  virtual std::unique_ptr<PortProbeInterface<PortT>> DoBuild(
      const PortT& port) = 0;
};

template <typename PortT, typename ValueT, auto ValueConvention,
          typename MessageT>
class PortProbeBuilder : public PortProbeBuilderInterface<PortT> {
 private:
  std::unique_ptr<PortProbeInterface<PortT>> DoBuild(
      const PortT& port) override {
    using PortProbeT = PortProbe<PortT, ValueT, ValueConvention, MessageT>;
    return std::make_unique<PortProbeT>(port);
  };
};

}  // namespace probes
}  // namespace drake_ros_introspection
