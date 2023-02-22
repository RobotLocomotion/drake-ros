#include <iostream>
#include <chrono>
#include <memory>
#include <filesystem>
#include <array>

#include <unistd.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rmw_isolation/rmw_isolation.h"

using std::placeholders::_1;

class Talker : public rclcpp::Node {
 public:
  Talker(const float& id)
      : rclcpp::Node("isolated_talker"), id_(id) {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("id", 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.1),
        std::bind(&Talker::timerCallback, this));
  }

 private:
  void timerCallback() const {
    std_msgs::msg::Float64 message;
    message.data = id_;
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float id_;
};

class Listener : public rclcpp::Node {
 public:
  Listener(const float& id)
      : rclcpp::Node("isolated_listener"), id_(id) {
    double timeout = this->declare_parameter("timeout", 1.0);
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "id", 10, std::bind(&Listener::topicCallback, this, _1));
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timeout),
        std::bind(&Listener::timerCallback, this));
  }

 private:
  void topicCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (msg->data != id_) {
      throw std::runtime_error("I heard '" + std::to_string(msg->data) + "' yet " +
                               "I was expecting '" + std::to_string(id_) + "'!");
    }
    ++expectedMessagesCount_;
  }

  void timerCallback() const {
    if (0u == expectedMessagesCount_) {
      throw std::runtime_error("I did not hear '" + std::to_string(id_) + "' even once!");
    }
    rclcpp::shutdown();
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t expectedMessagesCount_{0u};
  float id_;
};

// Launch a process for the talker or the listener.
pid_t launch_node(int argc, char* argv[], int id, const std::string& nodeType="talker"){
  // Launch a new process.
  auto processId = fork();
  // Return to the parent process.
  if (processId != 0){
    return processId;
  }

  // Create an isolated environment.
  auto logDirectory = std::filesystem::temp_directory_path().string().c_str();
  setenv("ROS_LOG_DIR", logDirectory, 1);
  auto directoryPath = std::filesystem::current_path() / std::to_string(id);
  if (!std::filesystem::exists(directoryPath)){
    std::filesystem::create_directory(directoryPath);
  }

  ros2::isolate_rmw_by_path(argv[0], directoryPath);

  rclcpp::init(argc, argv);
  if (nodeType == "talker"){
    rclcpp::spin(std::make_shared<Talker>(id));
  } else {
    rclcpp::spin(std::make_shared<Listener>(id));
  }
  rclcpp::shutdown();
  return 0;
}

int main(int argc, char* argv[]) {
  // Number of isolated talker-listener pairs.
  const int numberOfIsoaltedPairs = 3;

  // Start the processes.
  std::array<int, numberOfIsoaltedPairs> talkerProcesses;
  std::array<int, numberOfIsoaltedPairs> listenerProcesses;
  // Start the talkers.
  for (int i = 0; i < numberOfIsoaltedPairs; i++){
    talkerProcesses[i] = launch_node(argc, argv, i, "talker");
  }

  // Wait for the talkers to start.
  sleep(1.0);

  // Start the listeners.
  for (int i = 0; i < numberOfIsoaltedPairs; i++){
    listenerProcesses[i] = launch_node(argc, argv, i, "listener");
  }

  sleep(2.0);

  // Kill the talkers and listeners.
  for (int i = 0; i < numberOfIsoaltedPairs; i++){
    kill(talkerProcesses[i], SIGTERM);
    kill(listenerProcesses[i], SIGTERM);
  }

  return 0;
}
