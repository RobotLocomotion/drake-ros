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
#ifndef DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_

#include <drake/systems/framework/leaf_system.h>

#include <memory>

#include "drake_ros_systems/drake_ros_interface.hpp"

namespace drake_ros_systems
{
// PIMPL forward declaration
class RosInterfaceSystemPrivate;

/// System that takes care of calling spin() in Drake's systems framework
class RosInterfaceSystem : public drake::systems::LeafSystem<double>
{
public:
  explicit RosInterfaceSystem(std::unique_ptr<DrakeRosInterface> ros);
  virtual ~RosInterfaceSystem();

  /// Return a handle for interacting with ROS
  std::shared_ptr<DrakeRosInterface>
  get_ros_interface() const;

protected:
  /// Override as a place to call rclcpp::spin()
  void DoCalcNextUpdateTime(
    const drake::systems::Context<double> &,
    drake::systems::CompositeEventCollection<double> *,
    double *) const override;

  std::unique_ptr<RosInterfaceSystemPrivate> impl_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_
