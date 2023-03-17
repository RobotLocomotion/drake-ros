#include <cstdlib>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "drake_ros/core/drake_ros.h"

using drake_ros::core::DrakeRos;

TEST(DrakeRos, default_construct) {
  drake_ros::core::init();
  EXPECT_NO_THROW(std::make_unique<DrakeRos>("default_node"));
  EXPECT_TRUE(drake_ros::core::shutdown());
}

TEST(DrakeRos, local_context) {
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::NodeOptions node_options;
  node_options.context(context);
  context->init(0, nullptr);
  auto drake_ros = std::make_unique<DrakeRos>("local_ctx_node", node_options);
  (void)drake_ros;
  // Should not have initialized global context
  EXPECT_FALSE(rclcpp::contexts::get_global_default_context()->is_valid());
  context->shutdown("done");
}

TEST(DrakeRos, environment) {
  // The unit testing environment should always be shimmed to have proper
  // environment variables. Check that at least this one test case is shimmed.
  // If yes, it's likely that shimming is correct everywhere else, too.
  const char* const ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
  ASSERT_NE(ament_prefix_path, nullptr);
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
