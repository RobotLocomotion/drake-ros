#include "lib_with_ros.h"

#include <rclcpp/rclcpp.hpp>

void some_function()
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("lib_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
