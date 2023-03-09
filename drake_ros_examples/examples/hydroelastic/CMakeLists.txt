add_executable(hydroelastic
  hydroelastic.cc
)
target_link_libraries(collisions
  drake::drake
  drake_ros_viz::drake_ros_viz
  gflags
)
install(
  TARGETS
    hydroelastic
  DESTINATION lib/${PROJECT_NAME}
)
