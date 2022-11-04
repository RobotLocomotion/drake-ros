find_package(drake REQUIRED)
find_package(drake_ros_core REQUIRED)
find_package(drake_ros_tf2 REQUIRED)
find_package(drake_ros_viz REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ufo ufo.cc)
target_link_libraries(ufo PRIVATE
  drake::drake
  drake_ros_core::drake_ros_core
  drake_ros_tf2::drake_ros_tf2
  drake_ros_viz::drake_ros_viz
  ${geometry_msgs_TARGETS}
)

install(
  TARGETS
    ufo
  DESTINATION lib/${PROJECT_NAME}
)
