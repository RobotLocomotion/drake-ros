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

#include "drake_ros_core/geometry_conversions.h"

TEST(DrakeRosGeometryConversions, point) {
  geometry_msgs::msg::Point point;
  point.x = 1.0f;
  point.y = 2.0f;
  point.z = 3.0f;

  EXPECT_EQ(point, drake_ros_core::EigenVector3dToRosPoint(
                       drake_ros_core::RosPointToEigenVector3d(point)));
}

TEST(DrakeRosGeometryConversions, Quaternion) {
  geometry_msgs::msg::Quaternion quat;
  quat.x = 0.1f;
  quat.y = 0.2f;
  quat.z = 0.3f;
  quat.w = 0.4f;

  EXPECT_EQ(quat, drake_ros_core::EigenQuatToRosQuat(
                      drake_ros_core::RosQuatToEigenQuat(quat)));
}

TEST(DrakeRosGeometryConversions, Pose) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0f;
  pose.position.y = 2.0f;
  pose.position.z = 3.0f;
  pose.orientation.x = 0.1f;
  pose.orientation.y = 0.2f;
  pose.orientation.z = 0.3f;
  pose.orientation.w = 0.4f;

  EXPECT_EQ(pose, drake_ros_core::EigenIsometry3dToRosPose(
                      drake_ros_core::RosPoseToEigenIsometry3d(pose)));
  EXPECT_EQ(pose, drake_ros_core::DrakeTransformToRosPose(
                      drake_ros_core::RosPoseToDrakeTransform(pose)));
}

TEST(DrakeRosGeometryConversions, Transform) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.0f;
  transform.translation.y = 2.0f;
  transform.translation.z = 3.0f;
  transform.rotation.x = 0.1f;
  transform.rotation.y = 0.2f;
  transform.rotation.z = 0.3f;
  transform.rotation.w = 0.4f;

  EXPECT_EQ(transform,
            drake_ros_core::EigenIsometry3dToRosTransform(
                drake_ros_core::RosTransformToEigenIsometry3d(transform)));
  EXPECT_EQ(transform,
            drake_ros_core::DrakeTransformToRosTransform(
                drake_ros_core::RosTransformToDrakeTransform(transform)));
}

TEST(DrakeRosGeometryConversions, Twist) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0f;
  twist.linear.y = 2.0f;
  twist.linear.z = 3.0f;
  twist.angular.x = 4.0f;
  twist.angular.y = 5.0f;
  twist.angular.z = 6.0f;

  EXPECT_EQ(twist, drake_ros_core::EigenVector6dToRosTwist(
                       drake_ros_core::RosTwistToEigenVector6d(twist)));
  EXPECT_EQ(twist, drake_ros_core::DrakeVelocityToRosTwist(
                       drake_ros_core::RosTwistToDrakeVelocity(twist)));
}

TEST(DrakeRosGeometryConversions, Acceleration) {
  geometry_msgs::msg::Accel accel;
  accel.linear.x = 1.0f;
  accel.linear.y = 2.0f;
  accel.linear.z = 3.0f;
  accel.angular.x = 4.0f;
  accel.angular.y = 5.0f;
  accel.angular.z = 6.0f;

  EXPECT_EQ(accel, drake_ros_core::EigenVector6dToRosAccel(
                       drake_ros_core::RosAccelToEigenVector6d(accel)));
  EXPECT_EQ(accel, drake_ros_core::DrakeAccelToRosAccel(
                       drake_ros_core::RosAccelToDrakeAccel(accel)));
}

TEST(DrakeRosGeometryConversions, Wrench) {
  geometry_msgs::msg::Wrench wrench;
  wrench.force.x = 1.0f;
  wrench.force.y = 2.0f;
  wrench.force.z = 3.0f;
  wrench.torque.x = 4.0f;
  wrench.torque.y = 5.0f;
  wrench.torque.z = 6.0f;

  EXPECT_EQ(wrench, drake_ros_core::EigenVector6dToRosWrench(
                        drake_ros_core::RosWrenchToEigenVector6d(wrench)));
  EXPECT_EQ(wrench, drake_ros_core::DrakeForceToRosWrench(
                        drake_ros_core::RosWrenchToDrakeForce(wrench)));
}
