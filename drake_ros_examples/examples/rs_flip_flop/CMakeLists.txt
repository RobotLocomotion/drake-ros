find_package(std_msgs REQUIRED)

add_executable(rs_flip_flop rs_flip_flop.cpp)
target_link_libraries(rs_flip_flop
  drake::drake
  drake_ros::drake_ros_core
  ${std_msgs_TARGETS}
  gflags_shared
)

install(
  PROGRAMS rs_flip_flop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS rs_flip_flop
  DESTINATION lib/${PROJECT_NAME}
)
