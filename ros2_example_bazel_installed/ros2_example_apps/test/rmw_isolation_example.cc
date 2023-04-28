#include <iostream>
#include <chrono>
#include <memory>
#include <filesystem>
#include <vector>

#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rmw_isolation/rmw_isolation.h"

// This simple example demonstrates how a publisher and a subscriber
// can be isolated using rmw_isoaltion. We need to create a new temporary directory,
// and supply it to isolate_rmw_by_path() before invoking rclcpp::init().
// This isolation works process wide, and internally it creates a config file for the rmw layer.
// For a complete test, refer to :
// bazel_ros2_rules/ros2/resources/rmw_isolation/test/rmw_isolation_test.cc

using std::placeholders::_1;

class Talker : public rclcpp::Node {
 public:
  Talker()
      : rclcpp::Node("talker_example") {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("chatter", 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.1),
        std::bind(&Talker::TimerCallback, this));
  }

 private:
  void TimerCallback() const {
    std_msgs::msg::Float64 message;
    message.data = 1.0;
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class Listener : public rclcpp::Node {
 public:
  Listener()
      : rclcpp::Node("listener_example") {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "chatter", 10, std::bind(&Listener::TopicCallback, this, _1));
  }

 private:
  void TopicCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    ++messages_count_;
    if (messages_count_ >= 2) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  size_t messages_count_{0u};
};

// Launch a process for the talker or the listener.
pid_t LaunchNode(int argc, char* argv[], const std::string& node_type="talker"){
  auto process_id = fork();
  if (process_id != 0){
    // We are in the parent process. Return the child process id.
    return process_id;
  }

  rclcpp::init(argc, argv);
  if (node_type == "talker"){
    rclcpp::spin(std::make_shared<Talker>());
  } else {
    rclcpp::spin(std::make_shared<Listener>());
  }
  rclcpp::shutdown();
  return 0;
}

int main(int argc, char* argv[]){
  // Generate a temporary directory which will hold the config file for RMW isoaltion.
  auto directory_path = std::filesystem::current_path() / std::string("test_node_pair");
  if (!std::filesystem::exists(directory_path)){
    std::filesystem::create_directory(directory_path);
  }

  // The talker and listener processes are supplied a common directory_path
  // for rmw isoaltion, and hence will be able to talk to each other but will be isolated
  // from rest of the system. For e.g, if one were to run a new subscriber on the /chatter topic,
  // the data published by the talker would not be visible.
  // Note that you can also execute a separate program using functions like `system()` if you so choose.
  ros2::isolate_rmw_by_path(argv[0], directory_path);
  auto talker_process = LaunchNode(argc, argv, "talker");
  auto listener_process = LaunchNode(argc, argv, "listener");

  // Wait for the listener to exit, then kill the talker.
  waitpid(listener_process, NULL, 0);
  kill(talker_process, SIGINT);

  return 0;
}
