find_package(example_interfaces REQUIRED)

add_executable(introspection introspection.cpp)
include_directories( ${drake_ros_introspection_INCLUDE_DIRS})
target_link_libraries(introspection
  drake::drake
  drake_ros_core::drake_ros_core
  drake_ros_viz::drake_ros_viz
  #drake_ros_introspection::drake_ros_introspection
  ${example_interfaces_TARGETS}
)

install(
  PROGRAMS introspection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS introspection
  DESTINATION lib/${PROJECT_NAME}
)
