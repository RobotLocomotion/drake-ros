add_executable(collisions
  collisions.cc
)
target_link_libraries(collisions
  drake::drake
  drake_ros_viz::drake_ros_viz
  gflags
)
install(
  TARGETS
    collisions
  DESTINATION lib/${PROJECT_NAME}
)
