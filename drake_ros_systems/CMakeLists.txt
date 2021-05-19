cmake_minimum_required(VERSION 3.10)
project(drake_ros_systems)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drake REQUIRED)
# Must use Drake's fork of Pybind11
find_package(pybind11 REQUIRED HINTS "${drake_DIR}/../pybind11" NO_DEFAULT_PATH)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(drake_ros_systems
  src/drake_ros.cpp
  src/publisher.cpp
  src/ros_interface_system.cpp
  src/ros_publisher_system.cpp
  src/ros_subscriber_system.cpp
  src/rviz_visualizer.cpp
  src/scene_markers_system.cpp
  src/scene_tf_system.cpp
  src/subscription.cpp
  src/scene_tf_broadcaster_system.cpp
  src/utilities/name_conventions.cpp
  src/utilities/type_conversion.cpp
)

target_link_libraries(drake_ros_systems PUBLIC
    drake::drake
    rclcpp::rclcpp
    rosidl_runtime_c::rosidl_runtime_c
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

target_include_directories(drake_ros_systems
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<INSTALL_INTERFACE:include>"
)

ament_export_include_directories(include)
ament_export_libraries(drake_ros_systems)
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(drake)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(visualization_msgs)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

###
# Python bindings
###
pybind11_add_module(py_drake_ros_systems SHARED
  src/python_bindings/module_drake_ros_systems.cpp
)
set_target_properties(py_drake_ros_systems PROPERTIES OUTPUT_NAME "drake_ros_systems")
target_link_libraries(py_drake_ros_systems PRIVATE
  drake_ros_systems
)
target_include_directories(drake_ros_systems
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/python_bindings>"
)

# Sets PYTHON_INSTALL_DIR
_ament_cmake_python_get_python_install_dir()

install(
  TARGETS py_drake_ros_systems
  DESTINATION "${PYTHON_INSTALL_DIR}"
)
### End Python bindings

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_pub_sub test/test_pub_sub.cpp)

  # TODO(sloretz) Why isn't pybind11::embed including python libs?
  find_package(PythonLibs REQUIRED)

  target_link_libraries(test_pub_sub
    drake::drake
    drake_ros_systems
    ${test_msgs_TARGETS}
    pybind11::embed
    # TODO(sloretz) Remove when this is included in pybind11::embed
    ${PYTHON_LIBRARIES}
  )

  ament_add_gtest(test_drake_ros test/test_drake_ros.cpp)
  target_link_libraries(test_drake_ros
    drake_ros_systems
  )

  ament_add_gtest(test_tf_broadcaster test/test_tf_broadcaster.cpp)
  target_link_libraries(test_tf_broadcaster
    drake::drake
    drake_ros_systems
    ${geometry_msgs_TARGETS}
    rclcpp::rclcpp
    tf2_ros::tf2_ros
  )

  ament_add_pytest_test(test_tf_broadcaster_py test/test_tf_broadcaster.py)

  ament_add_gtest(test_scene_markers test/test_scene_markers.cpp)
  target_link_libraries(test_scene_markers
    drake::drake
    drake_ros_systems
    ${visualization_msgs_TARGETS}
  )

  ament_add_gtest(test_type_conversion test/test_type_conversion.cpp)
  target_link_libraries(test_type_conversion
    drake::drake
    drake_ros_systems
  )
endif()

ament_package()
