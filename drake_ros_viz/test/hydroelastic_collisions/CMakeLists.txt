add_executable(hydroelastic_collisions hydroelastic_collisions.cpp)

target_link_libraries(hydroelastic_collisions
  drake::drake
  drake_ros_systems::drake_ros_systems
  gflags
)

install(
  TARGETS
    hydroelastic_collisions
  DESTINATION lib/${PROJECT_NAME}
)
