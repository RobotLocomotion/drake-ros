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

#include <gtest/gtest.h>

#include "drake_ros_core/conversions.h"

using drake_ros_core::conversions;

TEST(DrakeRosConversions, point) {
  geometry_msgs::msg::Point point{.x = 1.0f, .y = 2.0f, .z = 3.0f};

  EXPECT_EQ(point,
            eigen_vector3d_to_ros_point(ros_point_to_eigen_vector3d(point)));
}

TEST(DrakeRosConversions, Quaternion) {
  geometry_msgs::msg::Quaternion quat{
      .x = 0.1f, .y = 0.2f, .z = 0.3f, .w = 0.4f};

  EXPECT_EQ(quat, eigen_quat_to_ros_quat(ros_quat_to_eigen_quat(quat)));
}

TEST(DrakeRosConversions, Pose) {
  geometry_msgs::msg::Pose pose{
      .position = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
      .orientation = {.x = 0.1f, .y = 0.2f, .z = 0.3f, .w = 0.4f}};

  EXPECT_EQ(pose,
            eigen_isometry3d_to_ros_pose(ros_pose_to_eigen_isometry3d(pose)));
  EXPECT_EQ(pose,
            drake_transform_to_ros_pose(ros_post_to_drake_transform(pose)));
}

TEST(DrakeRosConversions, Transform) {
  geometry_msgs::msg::Transform transform{
      .translation = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
      .rotation = {.x = 0.1f, .y = 0.2f, .z = 0.3f, .w = 0.4f}};

  EXPECT_EQ(transform, eigen_isometry3d_to_ros_transform(
                           ros_transform_to_eigen_isometry3d(transform)));
  EXPECT_EQ(transform, drake_transform_to_ros_transform(
                           ros_transform_to_drake_transform(transform)));
}

TEST(DrakeRosConversions, Twist) {
  geometry_msgs::msg::Twist twist = {
      .linear = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
      .angular = {.x = 4.0f, .y = 5.0f, .z = 6.0f}};

  EXPECT_EQ(twist,
            eigen_vector6d_to_ros_twist(ros_twist_to_eigen_vector6d(twist)));
  EXPECT_EQ(twist,
            drake_velocity_to_ros_twist(ros_twist_to_drake_velocity(twist)));
}

TEST(DrakeRosConversions, Acceleration) {
  geometry_msgs::msg::Accel accel = {
      .linear = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
      .angular = {.x = 4.0f, .y = 5.0f, .z = 6.0f}};

  EXPECT_EQ(accel,
            eigen_vector6d_to_ros_accel(ros_accel_to_eigen_vector6d(accel)));
  EXPECT_EQ(accel, drake_accel_to_ros_accel(ros_accel_to_drake_accel(accel)));
}

TEST(DrakeRosConversions, Wrench) {
  geometry_msgs::msg::Wrench wrench = {
      .force = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
      .torque = {.x = 4.0f, .y = 5.0f, .z = 6.0f}};

  EXPECT_EQ(wrench,
            eigen_vector6d_to_ros_wrench(ros_wrench_to_eigen_vector6d(wrench)));
  EXPECT_EQ(wrench,
            drake_force_to_ros_wrench(ros_wrench_to_drake_force(wrench)));
}
