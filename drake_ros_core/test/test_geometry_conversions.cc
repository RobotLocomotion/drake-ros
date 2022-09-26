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

namespace drake_ros_core {
namespace {

TEST(GeometryConversions, point) {
  geometry_msgs::msg::Point point;
  point.x = 1.0f;
  point.y = 2.0f;
  point.z = 3.0f;

  EXPECT_EQ(point, drake_ros_core::Vector3dToRosPoint(
                       drake_ros_core::RosPointToVector3d(point)));
}

TEST(GeometryConversions, Quaternion) {
  geometry_msgs::msg::Quaternion quat;
  quat.x = 0.1f;
  quat.y = 0.2f;
  quat.z = 0.3f;
  quat.w = 0.4f;

  EXPECT_EQ(quat,
            drake_ros_core::QuatToRosQuat(drake_ros_core::RosQuatToQuat(quat)));
}

TEST(GeometryConversions, Pose) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0f;
  pose.position.y = 2.0f;
  pose.position.z = 3.0f;
  pose.orientation.x = 0.1f;
  pose.orientation.y = 0.2f;
  pose.orientation.z = 0.3f;
  pose.orientation.w = 0.4f;

  EXPECT_EQ(pose, drake_ros_core::Isometry3dToRosPose(
                      drake_ros_core::RosPoseToIsometry3d(pose)));
  EXPECT_EQ(pose, drake_ros_core::RigidTransformToRosPose(
                      drake_ros_core::RosPoseToRigidTransform(pose)));
}

TEST(GeometryConversions, Transform) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.0f;
  transform.translation.y = 2.0f;
  transform.translation.z = 3.0f;
  transform.rotation.x = 0.1f;
  transform.rotation.y = 0.2f;
  transform.rotation.z = 0.3f;
  transform.rotation.w = 0.4f;

  EXPECT_EQ(transform,
            drake_ros_core::Isometry3dToRosTransform(
                drake_ros_core::RosTransformToIsometry3d(transform)));
  EXPECT_EQ(transform,
            drake_ros_core::RigidTransformToRosTransform(
                drake_ros_core::RosTransformToRigidTransform(transform)));
}

TEST(GeometryConversions, Twist) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0f;
  twist.linear.y = 2.0f;
  twist.linear.z = 3.0f;
  twist.angular.x = 4.0f;
  twist.angular.y = 5.0f;
  twist.angular.z = 6.0f;

  EXPECT_EQ(twist, drake_ros_core::Vector6dToRosTwist(
                       drake_ros_core::RosTwistToVector6d(twist)));
  EXPECT_EQ(twist, drake_ros_core::VelocityToRosTwist(
                       drake_ros_core::RosTwistToVelocity(twist)));
}

TEST(GeometryConversions, Acceleration) {
  geometry_msgs::msg::Accel accel;
  accel.linear.x = 1.0f;
  accel.linear.y = 2.0f;
  accel.linear.z = 3.0f;
  accel.angular.x = 4.0f;
  accel.angular.y = 5.0f;
  accel.angular.z = 6.0f;

  EXPECT_EQ(accel, drake_ros_core::Vector6dToRosAccel(
                       drake_ros_core::RosAccelToVector6d(accel)));
  EXPECT_EQ(accel, drake_ros_core::AccelToRosAccel(
                       drake_ros_core::RosAccelToAccel(accel)));
}

TEST(GeometryConversions, Wrench) {
  geometry_msgs::msg::Wrench wrench;
  wrench.force.x = 1.0f;
  wrench.force.y = 2.0f;
  wrench.force.z = 3.0f;
  wrench.torque.x = 4.0f;
  wrench.torque.y = 5.0f;
  wrench.torque.z = 6.0f;

  EXPECT_EQ(wrench, drake_ros_core::Vector6dToRosWrench(
                        drake_ros_core::RosWrenchToVector6d(wrench)));
  EXPECT_EQ(wrench, drake_ros_core::ForceToRosWrench(
                        drake_ros_core::RosWrenchToForce(wrench)));
}

}  // namespace
}  // namespace drake_ros_core
