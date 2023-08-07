add_executable(iiwa_manipulator iiwa_manipulator.cpp)
target_link_libraries(iiwa_manipulator
  drake::drake
  drake_ros::drake_ros_core
  drake_ros::drake_ros_viz
  gflags_shared
)

install(
  PROGRAMS iiwa_manipulator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    iiwa_manipulator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES iiwa_manipulator.rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*_launch.py"
)
