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

#include <limits>
#include <memory>
#include <utility>

#include "drake_ros_systems/ros_interface_system.hpp"

namespace drake_ros_systems
{
class RosInterfaceSystemPrivate
{
public:
  std::shared_ptr<DrakeRosInterface> ros_;
};


RosInterfaceSystem::RosInterfaceSystem(std::unique_ptr<DrakeRosInterface> ros)
: impl_(new RosInterfaceSystemPrivate())
{
  impl_->ros_ = std::move(ros);
}

RosInterfaceSystem::~RosInterfaceSystem()
{
}

/// Return a handle for interacting with ROS
std::shared_ptr<DrakeRosInterface>
RosInterfaceSystem::get_ros_interface() const
{
  return impl_->ros_;
}

/// Override as a place to call rclcpp::spin()
void
RosInterfaceSystem::DoCalcNextUpdateTime(
  const drake::systems::Context<double> &,
  drake::systems::CompositeEventCollection<double> *,
  double * time) const
{
  // Do work for at most 1ms so system doesn't get blocked if there's more work than it can handle
  const int max_work_time_millis = 1;
  impl_->ros_->spin(max_work_time_millis);
  // TODO(sloretz) Lcm system pauses time if some work was done, but ROS 2 API doesn't say if
  // any work was done. How to reconcile that?
  *time = std::numeric_limits<double>::infinity();
}
}  // namespace drake_ros_systems
