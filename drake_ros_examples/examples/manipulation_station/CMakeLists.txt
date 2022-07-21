find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(manipulation_station manipulation_station.cpp)
target_link_libraries(manipulation_station
  drake::drake
  drake_ros_core::drake_ros_core
  drake_ros_viz::drake_ros_viz
  rclcpp::rclcpp
  drake_ros_introspection::drake_ros_introspection
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

install(
  TARGETS manipulation_station
  DESTINATION lib/${PROJECT_NAME}
)
