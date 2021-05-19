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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>
#include <vector>

#include "drake_ros_systems/drake_ros.hpp"

using drake_ros_systems::DrakeRos;

TEST(DrakeRos, default_construct)
{
  EXPECT_NO_THROW(std::make_unique<DrakeRos>());
}

TEST(DrakeRos, local_context)
{
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::NodeOptions node_options;
  node_options.context(context);

  auto drake_ros = std::make_unique<DrakeRos>("local_ctx_node", node_options);
  (void) drake_ros;

  // Should not have initialized global context
  EXPECT_FALSE(rclcpp::contexts::get_global_default_context()->is_valid());
  EXPECT_TRUE(context->is_valid());
}
