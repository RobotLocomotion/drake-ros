#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include "common_msgs/msg/status.hpp"

using namespace std::chrono_literals;

namespace apps {

class Oracle : public rclcpp::Node {
public:
  Oracle() : Node("oracle") {
    status_pub_ = this->create_publisher<common_msgs::msg::Status>(
      "status", rclcpp::QoS(rclcpp::KeepLast(1)));

    status_timer_ = this->create_wall_timer(
      1s, std::bind(&Oracle::publish_status, this));
  }

private:
  void publish_status() {
    common_msgs::msg::Status msg;
    msg.stamp = this->get_clock()->now();
    msg.sequence_id = sequence_id_++;
    msg.message = "All good!";
    status_pub_->publish(msg);
  }

  uint64_t sequence_id_{0u};
  std::shared_ptr<rclcpp::TimerBase> status_timer_;
  std::shared_ptr<rclcpp::Publisher<common_msgs::msg::Status>> status_pub_;
};

}  // namespace apps

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<apps::Oracle>());

  rclcpp::shutdown();
}
