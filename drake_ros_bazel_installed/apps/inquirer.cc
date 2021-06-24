#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "common_msgs/msg/status.hpp"

using namespace std::chrono_literals;

namespace apps {

class Inquirer : public rclcpp::Node {
public:
  Inquirer() : Node("inquirer") {
    using namespace std::placeholders;
    status_sub_ =
      this->create_subscription<common_msgs::msg::Status>(
        "status", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&Inquirer::on_status, this, _1));
  }

private:
  void on_status(const common_msgs::msg::Status & msg) {
    RCLCPP_INFO(
      get_logger(), "(%lu) %s",
      msg.sequence_id, msg.message.c_str());
  }

  std::shared_ptr<rclcpp::Subscription<common_msgs::msg::Status>> status_sub_;
};

}  // namespace apps

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<apps::Inquirer>());

  rclcpp::shutdown();
}
