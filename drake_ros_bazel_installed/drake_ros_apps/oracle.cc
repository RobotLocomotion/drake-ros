#include <chrono>
#include <cstdint>
#include <memory>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>

#include "drake_ros_apps_msgs/msg/status.hpp"
#include "drake_ros_common_msgs/action/do.hpp"
#include "drake_ros_common_msgs/srv/query.hpp"

using namespace std::chrono_literals;

namespace drake_ros_apps {

class Oracle : public rclcpp::Node {
public:
  Oracle() : Node("oracle")
  {
    using namespace std::placeholders;

    status_pub_ = this->create_publisher<drake_ros_apps_msgs::msg::Status>(
      "status", rclcpp::QoS(rclcpp::KeepLast(1)));

    query_server_ = this->create_service<drake_ros_common_msgs::srv::Query>(
      "query", std::bind(&Oracle::handle_query, this, _1, _2));

    action_server_ = rclcpp_action::create_server<drake_ros_common_msgs::action::Do>(
        this, "do", std::bind(&Oracle::handle_action_goal, this, _1, _2),
        std::bind(&Oracle::handle_cancelled_action, this, _1),
        std::bind(&Oracle::handle_accepted_action, this, _1));

    status_timer_ = this->create_wall_timer(
      1s, std::bind(&Oracle::publish_status, this));
  }

private:

  rclcpp_action::GoalResponse
  handle_action_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const drake_ros_common_msgs::action::Do::Goal> goal)
  {
    if (goal->action != "rite")
    {
      RCLCPP_WARN(this->get_logger(), "Don't know how to %s", goal->action.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  using GoalHandle = rclcpp_action::ServerGoalHandle<drake_ros_common_msgs::action::Do>;

  rclcpp_action::CancelResponse handle_cancelled_action(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_action(const std::shared_ptr<GoalHandle> handle)
  {
    action_start_time_ = this->get_clock()->now();
    action_loop_ = this->create_wall_timer(
      rclcpp::Duration{handle->get_goal()->period}.to_chrono<std::chrono::nanoseconds>(),
      [this, handle]() { this->handle_rite_action(handle); });
  }

  void handle_rite_action(const std::shared_ptr<GoalHandle> handle)
  {
    if (handle->is_canceling())
    {
      auto result = std::make_shared<drake_ros_common_msgs::action::Do::Result>();
      handle->canceled(result);
      action_loop_->cancel();
      return;
    }

    auto current_time = this->get_clock()->now();
    if (current_time - action_start_time_ > handle->get_goal()->timeout)
    {
      auto result = std::make_shared<drake_ros_common_msgs::action::Do::Result>();
      result->reason = "timeout";
      handle->abort(result);
      action_loop_->cancel();
      return;
    }

    if (is_rite_complete_(gen_))
    {
      auto result = std::make_shared<drake_ros_common_msgs::action::Do::Result>();
      handle->succeed(result);
      action_loop_->cancel();
      return;
    }

    auto feedback = std::make_shared<drake_ros_common_msgs::action::Do::Feedback>();
    feedback->message = "chanting";
    handle->publish_feedback(feedback);
  }

  void handle_query(
    const std::shared_ptr<drake_ros_common_msgs::srv::Query::Request> request,
    std::shared_ptr<drake_ros_common_msgs::srv::Query::Response> response) const
  {
    if (request->query == "how's it going?") {
      response->reply = "all good!";
    } else {
      response->reply = "don't know";
    }
  }

  void publish_status() {
    drake_ros_apps_msgs::msg::Status msg;
    msg.stamp = this->get_clock()->now();
    msg.status.sequence_id = sequence_id_++;
    msg.status.message = "OK";
    msg.origin = "oracle";
    status_pub_->publish(msg);
  }

  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::bernoulli_distribution is_rite_complete_{0.1};

  uint64_t sequence_id_{0u};
  rclcpp::Time action_start_time_;
  std::shared_ptr<rclcpp::TimerBase> action_loop_;
  std::shared_ptr<rclcpp::TimerBase> status_timer_;
  std::shared_ptr<rclcpp::Publisher<drake_ros_apps_msgs::msg::Status>> status_pub_;
  std::shared_ptr<rclcpp::Service<drake_ros_common_msgs::srv::Query>> query_server_;
  std::shared_ptr<rclcpp_action::Server<drake_ros_common_msgs::action::Do>> action_server_;
};

}  // namespace drake_ros_apps

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<drake_ros_apps::Oracle>());

  rclcpp::shutdown();
}
