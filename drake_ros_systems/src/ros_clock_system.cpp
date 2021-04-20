// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include <rclcpp/clock.hpp>

#include "drake_ros_systems/ros_clock_system.hpp"


namespace drake_ros_systems
{
class RosClockSystemPrivate
{
public:
  std::shared_ptr<rclcpp::Clock> clock_;
};

RosClockSystem::RosClockSystem(DrakeRosInterface * ros)
: impl_(new RosClockSystemPrivate())
{
  impl_->clock_ = ros->get_clock();

  DeclareAbstractOutputPort("clock", &RosClockSystem::DoCalcOutput);
}

RosClockSystem::~RosClockSystem()
{
}

void
RosClockSystem::DoCalcOutput(
  const drake::systems::Context<double> &,
  double * output_value) const
{
  // Transfer message from state to output port
  *output_value = impl_->clock_->now().seconds();
}

}  // namespace drake_ros_systems
