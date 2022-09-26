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

#include <Eigen/Geometry>
#include <drake/common/eigen_types.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/math/spatial_algebra.h>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace drake_ros_core {

Eigen::Vector3d RosPointToVector3d(const geometry_msgs::msg::Point& point);

geometry_msgs::msg::Point Vector3dToRosPoint(const Eigen::Vector3d& point);

Eigen::Quaternion<double> RosQuatToQuat(
    const geometry_msgs::msg::Quaternion& quat);

geometry_msgs::msg::Quaternion QuatToRosQuat(
    const Eigen::Quaternion<double>& quat);

Eigen::Isometry3d RosPoseToIsometry3d(const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose Isometry3dToRosPose(const Eigen::Isometry3d& isometry);

drake::math::RigidTransformd RosPoseToRigidTransform(
    const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose RigidTransformToRosPose(
    const drake::math::RigidTransformd& transform);

Eigen::Isometry3d RosTransformToIsometry3d(
    const geometry_msgs::msg::Transform& transform);

geometry_msgs::msg::Transform Isometry3dToRosTransform(
    const Eigen::Isometry3d& isometry);

drake::math::RigidTransformd RosTransformToRigidTransform(
    const geometry_msgs::msg::Transform& transform);

geometry_msgs::msg::Transform RigidTransformToRosTransform(
    const drake::math::RigidTransformd& transform);

drake::Vector6d RosTwistToVector6d(const geometry_msgs::msg::Twist& twist);

geometry_msgs::msg::Twist Vector6dToRosTwist(const drake::Vector6d& vector);

drake::multibody::SpatialVelocity<double> RosTwistToVelocity(
    const geometry_msgs::msg::Twist& twist);

geometry_msgs::msg::Twist VelocityToRosTwist(
    const drake::multibody::SpatialVelocity<double>& velocity);

drake::Vector6d RosAccelToVector6d(const geometry_msgs::msg::Accel& accel);

geometry_msgs::msg::Accel Vector6dToRosAccel(const drake::Vector6d& vector);

drake::multibody::SpatialAcceleration<double> RosAccelToAccel(
    const geometry_msgs::msg::Accel& accel);

geometry_msgs::msg::Accel AccelToRosAccel(
    const drake::multibody::SpatialAcceleration<double>& accel);

drake::Vector6d RosWrenchToVector6d(const geometry_msgs::msg::Wrench& wrench);

geometry_msgs::msg::Wrench Vector6dToRosWrench(const drake::Vector6d& vector);

drake::multibody::SpatialForce<double> RosWrenchToForce(
    const geometry_msgs::msg::Wrench& wrench);

geometry_msgs::msg::Wrench ForceToRosWrench(
    const drake::multibody::SpatialForce<double>& force);

}  // namespace drake_ros_core
