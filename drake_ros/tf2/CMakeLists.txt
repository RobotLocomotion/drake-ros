find_package(geometry_msgs REQUIRED)
find_package(drake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

set(HEADERS
  "drake_ros_tf2_pybind.h"
  "name_conventions.h"
  "scene_tf_broadcaster_system.h"
  "scene_tf_system.h"
)

# Mock install headers so include paths match installed paths
set(mock_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include")
file(MAKE_DIRECTORY "${mock_include_dir}/drake_ros/tf2")
foreach(hdr ${HEADERS})
  configure_file("${hdr}" "${mock_include_dir}/drake_ros/tf2/${hdr}" COPYONLY)
endforeach()

add_library(drake_ros_tf2
  name_conventions.cc
  scene_tf_broadcaster_system.cc
  scene_tf_system.cc
)

target_link_libraries(drake_ros_tf2 PUBLIC
  drake_ros_core
  drake::drake
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
)

target_include_directories(drake_ros_tf2
  PUBLIC
    "$<BUILD_INTERFACE:${mock_include_dir}>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS drake_ros_tf2
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    ${HEADERS}
  DESTINATION include/drake_ros/tf2
)

###
# Python bindings
###
pybind11_add_module(py_drake_ros_tf2 SHARED
  cc_py.cc
)
set_target_properties(py_drake_ros_tf2
  PROPERTIES
    OUTPUT_NAME "_cc"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/py/drake_ros/tf2/")
target_link_libraries(py_drake_ros_tf2 PRIVATE drake_ros_tf2)

# Sets PYTHON_INSTALL_DIR
ament_get_python_install_dir(python_install_dir)

install(
  TARGETS py_drake_ros_tf2
  DESTINATION "${python_install_dir}/drake_ros/tf2/"
)
### End Python bindings

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gtest(test_tf_broadcaster test/test_tf_broadcaster.cc)
  target_link_libraries(test_tf_broadcaster
    drake::drake
    rclcpp::rclcpp
    drake_ros_tf2
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
  )
  target_compile_definitions(test_tf_broadcaster
    PRIVATE
    # We do not expose `rmw_isoliation` via CMake.
    _TEST_DISABLE_RMW_ISOLATION
  )

  ament_add_pytest_test(test_tf_broadcaster_py test/test_tf_broadcaster.py
    # Let Python import mock-install structure in build folder
    APPEND_ENV
      "PYTHONPATH=${PROJECT_BINARY_DIR}/py/")

  ament_add_gtest(test_tf2_name_conventions test/test_name_conventions.cc)
  target_include_directories(test_tf2_name_conventions
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  )
  target_link_libraries(test_tf2_name_conventions
    drake::drake
    drake_ros_tf2
  )
endif()
