#include <iostream>
#include <chrono>
#include <memory>
#include <filesystem>
#include <vector>

#include <unistd.h>
#include <signal.h>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rmw_isolation/rmw_isolation.h"

using std::placeholders::_1;

DEFINE_int32(number_of_isolated_pairs, 5, "Number of isolated talker-listener pairs");

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
      throw std::runtime_error(fmt::format("I heard '{0}' yet "
                               "I was expecting '{1}'!", msg->data, id_));
    }
    ++expectedMessagesCount_;
  }

  void timerCallback() const {
    if (0u == expectedMessagesCount_) {
      throw std::runtime_error(fmt::format("I did not hear '{0}' even once!", id_));
    }
    rclcpp::shutdown();
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t expectedMessagesCount_{0u};
  float id_;
};

// Launch a process for the talker or the listener.
pid_t LaunchNode(int argc, char* argv[], int id, const std::string& node_type="talker"){
  // Launch a new process.
  auto process_id = fork();
  if (process_id != 0){
    // We are in the parent process. Return the child process id.
    return process_id;
  }

  // Create an isolated environment.
  auto log_directory = std::filesystem::temp_directory_path().string().c_str();
  setenv("ROS_LOG_DIR", log_directory, 1);
  auto directory_path = std::filesystem::current_path() / std::to_string(id);
  if (!std::filesystem::exists(directory_path)){
    std::filesystem::create_directory(directory_path);
  }

  ros2::isolate_rmw_by_path(argv[0], directory_path);

  rclcpp::init(argc, argv);
  if (node_type == "talker"){
    rclcpp::spin(std::make_shared<Talker>(id));
  } else {
    rclcpp::spin(std::make_shared<Listener>(id));
  }
  rclcpp::shutdown();
  return 0;
}

int main(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Start the processes.
  std::vector<int> talker_processes;
  std::vector<int> listener_processes;
  // Start the talkers.
  for (int i = 0; i < FLAGS_number_of_isolated_pairs; i++){
    talker_processes.push_back(LaunchNode(argc, argv, i, "talker"));
  }

  // Wait for the talkers to start.
  sleep(1.0);

  // Start the listeners.
  for (int i = 0; i < FLAGS_number_of_isolated_pairs; i++){
    listener_processes.push_back(LaunchNode(argc, argv, i, "listener"));
  }

  sleep(2.0);

  // Kill the talkers and listeners.
  for (int i = 0; i < FLAGS_number_of_isolated_pairs; i++){
    kill(talker_processes[i], SIGTERM);
    kill(listener_processes[i], SIGTERM);
  }

  return 0;
}
