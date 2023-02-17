
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)


set(HEADERS
  "drake_ros_core_pybind.h"
  "drake_ros.h"
  "geometry_conversions.h"
  "publisher.h"
  "qos_pybind.h"
  "ros_interface_system.h"
  "ros_publisher_system.h"
  "ros_subscriber_system.h"
  "serializer.h"
  "serializer_interface.h"
)

# Mock install headers so include paths match installed paths
set(mock_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include")
file(MAKE_DIRECTORY "${mock_include_dir}/drake_ros/core")
foreach(hdr ${HEADERS})
  configure_file("${hdr}" "${mock_include_dir}/drake_ros/core/${hdr}" COPYONLY)
endforeach()

add_library(drake_ros_core
  drake_ros.cc
  geometry_conversions.cc
  publisher.cc
  ros_interface_system.cc
  ros_publisher_system.cc
  ros_subscriber_system.cc
  subscription.cc
)

target_link_libraries(drake_ros_core PUBLIC
  Eigen3::Eigen
  drake::drake
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rosidl_runtime_c::rosidl_runtime_c
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)

target_include_directories(drake_ros_core
  PUBLIC
    "$<BUILD_INTERFACE:${mock_include_dir}>"
    "$<INSTALL_INTERFACE:include>"
)

# Nominal ament install directives
install(TARGETS drake_ros_core EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    ${HEADERS}
  DESTINATION include/drake_ros/core
)

###
# Python bindings
###
pybind11_add_module(drake_ros_core_py SHARED
  module_py.cc
)
target_link_libraries(drake_ros_core_py PRIVATE drake_ros_core)
set_target_properties(drake_ros_core_py PROPERTIES OUTPUT_NAME "_drake_ros_core")
target_include_directories(drake_ros_core_py
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)

ament_get_python_install_dir(python_install_dir)

install(
  TARGETS drake_ros_core_py
  DESTINATION "${python_install_dir}"
)
### End Python bindings

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gtest(test_pub_sub test/test_pub_sub.cc)

  target_compile_definitions(test_pub_sub
    PRIVATE
    # We do not expose `rmw_isoliation` via CMake.
    _TEST_DISABLE_RMW_ISOLATION
  )
  target_link_libraries(test_pub_sub
    drake::drake
    drake_ros_core
    ${test_msgs_TARGETS}
  )

  # TODO(eric, shane): Add environment variable to disable rmw_isolation.
  ament_add_pytest_test(test_pub_sub_py test/test_pub_sub.py)

  ament_add_gtest(test_drake_ros test/test_drake_ros.cc)
  target_compile_definitions(test_drake_ros
    PRIVATE
    # We do not expose `rmw_isoliation` via CMake.
    _TEST_DISABLE_RMW_ISOLATION
  )
  target_link_libraries(test_drake_ros
    drake_ros_core
  )

  ament_add_gtest(test_geometry_conversions test/test_geometry_conversions.cc)
  target_link_libraries(test_geometry_conversions drake_ros_core)
endif()
