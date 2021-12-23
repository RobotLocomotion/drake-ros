#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>

#include "drake_ros_apps_msgs/msg/status.hpp"
#include "drake_ros_common_msgs/action/do.hpp"
#include "drake_ros_common_msgs/srv/query.hpp"

using namespace std::chrono_literals;

namespace drake_ros_apps {

class Inquirer : public rclcpp::Node {
 public:
  Inquirer() : Node("inquirer") {
    using namespace std::placeholders;
    status_sub_ = this->create_subscription<drake_ros_apps_msgs::msg::Status>(
        "status", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&Inquirer::on_status, this, _1));

    query_client_ =
        this->create_client<drake_ros_common_msgs::srv::Query>("query");

    action_client_ =
        rclcpp_action::create_client<drake_ros_common_msgs::action::Do>(this,
                                                                        "do");

    inquire_timer_ =
        this->create_wall_timer(5s, std::bind(&Inquirer::inquire, this));
  }

 private:
  using QueryClient = rclcpp::Client<drake_ros_common_msgs::srv::Query>;

  void handle_reply(QueryClient::SharedFuture future) const {
    RCLCPP_INFO(this->get_logger(), "oracle said: %s",
                future.get()->reply.c_str());
  }

  using ActionGoalHandle =
      rclcpp_action::ClientGoalHandle<drake_ros_common_msgs::action::Do>;
  void handle_rite_request_response(
      const std::shared_ptr<ActionGoalHandle> handle) const {
    if (!handle) {
      RCLCPP_ERROR(this->get_logger(), "oracle rejected rite request");
    } else {
      RCLCPP_INFO(this->get_logger(), "oracle rite in progress");
    }
  }

  void handle_rite_feedback(
      std::shared_ptr<ActionGoalHandle> handle,
      const std::shared_ptr<const ActionGoalHandle::Feedback> feedback) const {
    RCLCPP_INFO(this->get_logger(), "oracle is %s", feedback->message.c_str());
  }

  void handle_rite_result(const ActionGoalHandle::WrappedResult& result) const {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "oracle rite is complete");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "oracle rite aborted due to %s",
                     result.result->reason.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "oracle rite was cancelled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "oracle rite status unknown");
        break;
    }
  }

  void inquire() const {
    using namespace std::placeholders;

    if (query_client_->service_is_ready()) {
      auto request =
          std::make_shared<drake_ros_common_msgs::srv::Query::Request>();
      request->query = "how's it going?";
      RCLCPP_INFO(this->get_logger(), "oracle, %s", request->query.c_str());
      query_client_->async_send_request(
          request, std::bind(&Inquirer::handle_reply, this, _1));
    } else {
      RCLCPP_WARN(this->get_logger(), "oracle not available for queries");
    }

    if (action_client_->action_server_is_ready()) {
      rclcpp_action::Client<drake_ros_common_msgs::action::Do>::SendGoalOptions
          options;
      options.goal_response_callback =
          std::bind(&Inquirer::handle_rite_request_response, this, _1);
      options.feedback_callback =
          std::bind(&Inquirer::handle_rite_feedback, this, _1, _2);
      options.result_callback =
          std::bind(&Inquirer::handle_rite_result, this, _1);
      drake_ros_common_msgs::action::Do::Goal goal;
      goal.action = "rite";
      goal.period = rclcpp::Duration::from_seconds(0.1);
      goal.timeout = rclcpp::Duration::from_seconds(1.0);
      action_client_->async_send_goal(goal, options);
    } else {
      RCLCPP_WARN(this->get_logger(), "oracle not available for actions");
    }
  }

  void on_status(const drake_ros_apps_msgs::msg::Status& msg) const {
    RCLCPP_INFO(get_logger(), "%s status (%lu): %s", msg.origin.c_str(),
                msg.status.sequence_id, msg.status.message.c_str());
  }

  std::shared_ptr<rclcpp::Subscription<drake_ros_apps_msgs::msg::Status>>
      status_sub_;
  std::shared_ptr<rclcpp::Client<drake_ros_common_msgs::srv::Query>>
      query_client_;
  std::shared_ptr<rclcpp_action::Client<drake_ros_common_msgs::action::Do>>
      action_client_;
  std::shared_ptr<rclcpp::TimerBase> inquire_timer_;
};

}  // namespace drake_ros_apps

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<drake_ros_apps::Inquirer>());

  rclcpp::shutdown();
}
