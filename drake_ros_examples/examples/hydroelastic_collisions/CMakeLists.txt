add_executable(hydroelastic_collisions
  hydroelastic_collisions.cc
)
target_link_libraries(hydroelastic_collisions
  drake::drake
  drake_ros_viz::drake_ros_viz
  gflags
)
install(
  TARGETS
    hydroelastic_collisions
  DESTINATION lib/${PROJECT_NAME}
)
