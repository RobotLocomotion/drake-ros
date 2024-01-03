add_executable(hydroelastic
  hydroelastic.cc
)
target_link_libraries(hydroelastic
  drake::drake
  drake_ros::drake_ros_viz
  gflags
  ament_index_cpp::ament_index_cpp
)
install(
  TARGETS
    hydroelastic
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  hydroelastic.sdf
  hydroelastic.rviz
  DESTINATION share/${PROJECT_NAME}/hydroelastic/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*_launch.py"
)
