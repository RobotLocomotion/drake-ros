add_executable(multirobot multirobot.cc)
target_link_libraries(multirobot
  drake::drake
  drake_ros::drake_ros_core
  drake_ros::drake_ros_tf2
  drake_ros::drake_ros_viz
  gflags_shared
)

install(
  TARGETS multirobot
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS multirobot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES multirobot.rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*_launch.py"
)
