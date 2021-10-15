#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "tools/ros2/rmw_isolation.h"

using std::placeholders::_1;

class IsolatedListener : public rclcpp::Node
{
  public:
    IsolatedListener()
    : rclcpp::Node("isolated_listener")
    {
      double timeout = this->declare_parameter("timeout", 2.0);
      subscription_ = this->create_subscription<std_msgs::msg::String>(
          "topic", 10, std::bind(&IsolatedListener::topic_callback, this, _1));
      timer_ = this->create_wall_timer(
          std::chrono::duration<double>(timeout),
          std::bind(&IsolatedListener::timer_callback, this));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      throw std::runtime_error("I heard '" + msg->data + "'");
    }

    void timer_callback() const
    {
      rclcpp::shutdown();
    }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  const char * TEST_TMPDIR = std::getenv("TEST_TMPDIR");
  if (TEST_TMPDIR != nullptr) {
    ros2::isolate_rmw_by_path(argv[0], TEST_TMPDIR);
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IsolatedListener>());
  rclcpp::shutdown();
  return 0;
}
