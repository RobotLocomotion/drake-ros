#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  explicit MinimalSubscriber(size_t max_count = 10)
      : Node("minimal_subscriber"), max_count_(max_count), count_(0) {
    auto qos = rclcpp::QoS{max_count};
    qos.transient_local().reliable();
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", qos, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

  bool is_done() const { return count_ >= max_count_; }

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    ++count_;
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  size_t max_count_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Use explicit `spin_some` so we can manually check for completion.
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // Use large enough non-zero max duration to ensure we do not block.
  const std::chrono::microseconds spin_max_duration{50};

  while (rclcpp::ok() && !node->is_done()) {
    executor.spin_some(spin_max_duration);
  }

  return 0;
}
