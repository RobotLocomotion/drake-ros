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
namespace conversions {

Eigen::Vector3d ros_point_to_eigen_vector3d(
    const geometry_msgs::msg::Point& point);

geometry_msgs::msg::Point eigen_vector3d_to_ros_point(
    const Eigen::Vector3d& point);

Eigen::Quaternion<double> ros_quat_to_eigen_quat(
    const geometry_msgs::msg::Quaternion& quat);

geometry_msgs::msg::Quaternion eigen_quat_to_ros_quat(
    const Eigen::Quaternion<double>& quat);

Eigen::Isometry3d ros_pose_to_eigen_isometry3d(
    const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose eigen_isometry3d_to_ros_pose(
    const Eigen::Isometry3d& isometry);

drake::math::RigidTransformd ros_pose_to_drake_transform(
    const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose drake_transform_to_ros_pose(
    const drake::math::RigidTransformd& transform);

Eigen::Isometry3d ros_transform_to_eigen_isometry3d(
    const geometry_msgs::msg::Transform& transform);

geometry_msgs::msg::Transform eigen_isometry3d_to_ros_transform(
    const Eigen::Isometry3d& isometry);

drake::math::RigidTransformd ros_transform_to_drake_transform(
    const geometry_msgs::msg::Transform& transform);

geometry_msgs::msg::Transform drake_transform_to_ros_transform(
    const drake::math::RigidTransformd& transform);

drake::Vector6d ros_twist_to_eigen_vector6d(
    const geometry_msgs::msg::Twist& twist);

geometry_msgs::msg::Twist eigen_vector6d_to_ros_twist(
    const drake::Vector6d& vector);

drake::multibody::SpatialVelocity<double> ros_twist_to_drake_velocity(
    const geometry_msgs::msg::Twist& twist);

geometry_msgs::msg::Twist drake_velocity_to_ros_twist(
    const drake::multibody::SpatialVelocity<double>& velocity);

drake::Vector6d ros_accel_to_eigen_vector6d(
    const geometry_msgs::msg::Accel& accel);

geometry_msgs::msg::Accel eigen_vector6d_to_ros_accel(
    const drake::Vector6d& vector);

drake::multibody::SpatialAcceleration<double> ros_accel_to_drake_accel(
    const geometry_msgs::msg::Accel& accel);

geometry_msgs::msg::Accel drake_accel_to_ros_accel(
    const drake::multibody::SpatialAcceleration<double>& accel);

drake::Vector6d ros_wrench_to_eigen_vector6d(
    const geometry_msgs::msg::Wrench& wrench);

geometry_msgs::msg::Wrench eigen_vector6d_to_ros_wrench(
    const drake::Vector6d& vector);

drake::multibody::SpatialForce<double> ros_wrench_to_drake_force(
    const geometry_msgs::msg::Wrench& wrench);

geometry_msgs::msg::Wrench drake_force_to_ros_wrench(
    const drake::multibody::SpatialForce<double>& force);

}  // namespace conversions
}  // namespace drake_ros_core
