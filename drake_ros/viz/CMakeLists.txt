find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

set(HEADERS
  "drake_ros_viz_pybind.h"
  "name_conventions.h"
  "rviz_visualizer.h"
  "scene_markers_system.h"
)

# Mock install headers so include paths match installed paths
set(mock_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include")
file(MAKE_DIRECTORY "${mock_include_dir}/drake_ros_viz")
foreach(hdr ${HEADERS})
  configure_file("${hdr}" "${mock_include_dir}/drake_ros_viz/${hdr}" COPYONLY)
endforeach()

add_library(viz
  name_conventions.cc
  rviz_visualizer.cc
  scene_markers_system.cc
)
set_target_properties(viz PROPERTIES OUTPUT_NAME "drake-ros-viz")

target_link_libraries(viz PUBLIC
    drake::drake
    core
    tf2
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

target_include_directories(viz
  PUBLIC
    "$<BUILD_INTERFACE:${mock_include_dir}>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS viz
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    ${HEADERS}
  DESTINATION include/drake_ros_viz
)

###
# Python bindings
###
pybind11_add_module(py_drake_ros_viz SHARED
  module_py.cc
)
set_target_properties(py_drake_ros_viz PROPERTIES OUTPUT_NAME "drake_ros_viz")
target_link_libraries(py_drake_ros_viz PRIVATE viz)

# Sets PYTHON_INSTALL_DIR
ament_get_python_install_dir(python_install_dir)

install(
  TARGETS py_drake_ros_viz
  DESTINATION "${python_install_dir}"
)
### End Python bindings

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gtest(test_scene_markers test/test_scene_markers.cc)
  target_link_libraries(test_scene_markers
    drake::drake
    viz
    ${visualization_msgs_TARGETS}
  )

  ament_add_gtest(test_viz_name_conventions test/test_name_conventions.cc)
  target_include_directories(test_viz_name_conventions
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  )
  target_link_libraries(test_viz_name_conventions
    drake::drake
    viz
  )

  ament_add_pytest_test(test_python_bindings test/test_python_bindings.py)
endif()