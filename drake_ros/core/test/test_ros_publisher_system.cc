#include <cstdlib>
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/basic_types.hpp>

#include "drake_ros/core/drake_ros.h"
#include "drake_ros/core/ros_interface_system.h"
#include "drake_ros/core/ros_publisher_system.h"

using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;
using drake_ros::core::RosPublisherSystem;

TEST(RosPublisherSystem, drake_ros_factory_construct) {
  drake_ros::core::init();
  constexpr double kPublishPeriod = 1.0;
  const auto qos = rclcpp::QoS{rclcpp::KeepLast(10)}.reliable();

  auto drake_ros = std::make_unique<DrakeRos>("default_node");
  auto ros_interface_system = RosInterfaceSystem(std::move(drake_ros));

  auto pub = RosPublisherSystem::Make<test_msgs::msg::BasicTypes>(
          "out", qos, ros_interface_system.get_ros_interface(),
          {drake::systems::TriggerType::kPeriodic}, kPublishPeriod);
  EXPECT_TRUE(drake_ros::core::shutdown());
}


TEST(RosPublisherSystem, external_node_factory_construct) {
  // This test is to ensure that the factory method works with a raw node pointer.
  // It does not require a DrakeRos instance.
  drake_ros::core::init();
  constexpr double kPublishPeriod = 1.0;
  const auto qos = rclcpp::QoS{rclcpp::KeepLast(10)}.reliable();

  auto ros_node = std::make_shared<rclcpp::Node>("external_node");

  auto pub = RosPublisherSystem::Make<test_msgs::msg::BasicTypes>(
          "out", qos, ros_node.get(),
          {drake::systems::TriggerType::kPeriodic}, kPublishPeriod);
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

