#include <cstdlib>
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "drake_ros/core/ros_interface_system.h"

using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;

TEST(RosInterfaceSystem, default_construct) {
  drake_ros::core::init();
  auto drake_ros = std::make_unique<DrakeRos>("default_node");
  auto ros_interface_system = RosInterfaceSystem(std::move(drake_ros));
  EXPECT_TRUE(drake_ros::core::shutdown());
}

TEST(DrakeRos, external_node) {
  drake_ros::core::init();
  std::string node_name = "external_node";
  auto node = std::make_shared<rclcpp::Node>(node_name);
  auto ros_interface_system = RosInterfaceSystem(node);
  EXPECT_EQ(ros_interface_system.get_ros_interface()->get_node().get_name(),
            node_name);
  EXPECT_EQ(ros_interface_system.get_ros_interface()->get_mutable_node(),
            node.get());
  EXPECT_TRUE(drake_ros::core::shutdown());
}

// Only available in Bazel.
#ifndef _TEST_DISABLE_RMW_ISOLATION
#include "rmw_isolation/rmw_isolation.h"

int main(int argc, char* argv[]) {
  const char* TEST_TMPDIR = std::getenv("TEST_TMPDIR");
  if (TEST_TMPDIR != nullptr) {
    std::string ros_home = std::string(TEST_TMPDIR) + "/.ros";
    setenv("ROS_HOME", ros_home.c_str(), 1);
    ros2::isolate_rmw_by_path(argv[0], TEST_TMPDIR);
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#endif
