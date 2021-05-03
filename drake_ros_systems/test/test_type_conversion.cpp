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

#include <gtest/gtest.h>

#include <cmath>

#include "drake_ros_systems/utilities/type_conversion.hpp"


TEST(TypeConversion, ToPoseMsg)
{
  const drake::Vector3<double> position{1., 2., 3.};
  const drake::Quaternion<double> orientation{0., 0., sin(M_PI / 4.), cos(M_PI / 4.)};
  const drake::math::RigidTransform<double> pose{orientation, position};
  const geometry_msgs::msg::Pose message =
    drake_ros_systems::utilities::ToPoseMsg(pose);
  EXPECT_DOUBLE_EQ(message.position.x, position.x());
  EXPECT_DOUBLE_EQ(message.position.y, position.y());
  EXPECT_DOUBLE_EQ(message.position.z, position.z());
  EXPECT_DOUBLE_EQ(message.orientation.x, orientation.x());
  EXPECT_DOUBLE_EQ(message.orientation.y, orientation.y());
  EXPECT_DOUBLE_EQ(message.orientation.z, orientation.z());
  EXPECT_DOUBLE_EQ(message.orientation.w, orientation.w());
}

TEST(TypeConversion, ToTransformMsg)
{
  const drake::Vector3<double> translation{3., 2., 1.};
  const drake::Quaternion<double> rotation{0., 0., sin(-M_PI / 4.), cos(-M_PI / 4.)};
  const drake::math::RigidTransform<double> transform{rotation, translation};
  const geometry_msgs::msg::Transform message =
    drake_ros_systems::utilities::ToTransformMsg(transform);
  EXPECT_DOUBLE_EQ(message.translation.x, translation.x());
  EXPECT_DOUBLE_EQ(message.translation.y, translation.y());
  EXPECT_DOUBLE_EQ(message.translation.z, translation.z());
  EXPECT_DOUBLE_EQ(message.rotation.x, rotation.x());
  EXPECT_DOUBLE_EQ(message.rotation.y, rotation.y());
  EXPECT_DOUBLE_EQ(message.rotation.z, rotation.z());
  EXPECT_DOUBLE_EQ(message.rotation.w, rotation.w());
}
