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
#ifndef DRAKE_ROS_VIZ__UTILITIES__TYPE_CONVERSION_HPP_
#define DRAKE_ROS_VIZ__UTILITIES__TYPE_CONVERSION_HPP_

#include <drake/math/rigid_transform.h>
#include <geometry_msgs/msg/pose.hpp>

namespace drake_ros_viz {
namespace utilities {

geometry_msgs::msg::Pose ToPoseMsg(
    const drake::math::RigidTransform<double> X_AB);

}  // namespace utilities
}  // namespace drake_ros_viz

#endif  // DRAKE_ROS_VIZ__UTILITIES__TYPE_CONVERSION_HPP_
