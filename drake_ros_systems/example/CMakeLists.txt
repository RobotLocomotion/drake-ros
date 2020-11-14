find_package(std_msgs REQUIRED)

add_executable(rs_flip_flop rs_flip_flop.cpp)
target_link_libraries(rs_flip_flop
  drake::drake
  drake_ros_systems
  ${std_msgs_TARGETS}
)
