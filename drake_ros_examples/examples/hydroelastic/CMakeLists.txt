add_executable(hydroelastic
  hydroelastic.cc
)
target_link_libraries(hydroelastic
  drake::drake
  drake_ros::drake_ros_viz
  gflags
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
