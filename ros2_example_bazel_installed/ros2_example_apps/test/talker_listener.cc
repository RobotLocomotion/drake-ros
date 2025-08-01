#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "listener.h"
#include "talker.h"

#include "lib/ros_environment/unique.h"

int main(int argc, char* argv[]) {
  bazel_ros2_rules::EnforceUniqueROSEnvironment();

  rclcpp::init(argc, argv);

  auto talker = std::make_shared<Talker>();
  auto listener = std::make_shared<Listener>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(talker);
  exec.add_node(listener);

  auto got_message = listener->NextMessage();

  auto result =
    exec.spin_until_future_complete(got_message, std::chrono::seconds(5));

  rclcpp::shutdown();

  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    return 0;
  }
  return 1;
}
