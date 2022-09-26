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

Eigen::Vector3d RosPointToEigenVector3d(const geometry_msgs::msg::Point& point);

geometry_msgs::msg::Point EigenVector3dToRosPoint(const Eigen::Vector3d& point);

Eigen::Quaternion<double> RosQuatToEigenQuat(
    const geometry_msgs::msg::Quaternion& quat);

geometry_msgs::msg::Quaternion EigenQuatToRosQuat(
    const Eigen::Quaternion<double>& quat);

Eigen::Isometry3d RosPoseToEigenIsometry3d(
    const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose EigenIsometry3dToRosPose(
    const Eigen::Isometry3d& isometry);

drake::math::RigidTransformd RosPoseToDrakeTransform(
    const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose DrakeTransformToRosPose(
    const drake::math::RigidTransformd& transform);

Eigen::Isometry3d RosTransformToEigenIsometry3d(
    const geometry_msgs::msg::Transform& transform);

geometry_msgs::msg::Transform EigenIsometry3dToRosTransform(
    const Eigen::Isometry3d& isometry);

drake::math::RigidTransformd RosTransformToDrakeTransform(
    const geometry_msgs::msg::Transform& transform);

geometry_msgs::msg::Transform DrakeTransformToRosTransform(
    const drake::math::RigidTransformd& transform);

drake::Vector6d RosTwistToEigenVector6d(const geometry_msgs::msg::Twist& twist);

geometry_msgs::msg::Twist EigenVector6dToRosTwist(
    const drake::Vector6d& vector);

drake::multibody::SpatialVelocity<double> RosTwistToDrakeVelocity(
    const geometry_msgs::msg::Twist& twist);

geometry_msgs::msg::Twist DrakeVelocityToRosTwist(
    const drake::multibody::SpatialVelocity<double>& velocity);

drake::Vector6d RosAccelToEigenVector6d(const geometry_msgs::msg::Accel& accel);

geometry_msgs::msg::Accel EigenVector6dToRosAccel(
    const drake::Vector6d& vector);

drake::multibody::SpatialAcceleration<double> RosAccelToDrakeAccel(
    const geometry_msgs::msg::Accel& accel);

geometry_msgs::msg::Accel DrakeAccelToRosAccel(
    const drake::multibody::SpatialAcceleration<double>& accel);

drake::Vector6d RosWrenchToEigenVector6d(
    const geometry_msgs::msg::Wrench& wrench);

geometry_msgs::msg::Wrench EigenVector6dToRosWrench(
    const drake::Vector6d& vector);

drake::multibody::SpatialForce<double> RosWrenchToDrakeForce(
    const geometry_msgs::msg::Wrench& wrench);

geometry_msgs::msg::Wrench DrakeForceToRosWrench(
    const drake::multibody::SpatialForce<double>& force);

}  // namespace drake_ros_core
