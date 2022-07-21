find_package(std_msgs REQUIRED)

add_executable(sine sine.cpp)
target_link_libraries(sine
  drake::drake
  drake_ros_introspection::drake_ros_introspection
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

install(
  TARGETS sine
  DESTINATION lib/${PROJECT_NAME}
)
