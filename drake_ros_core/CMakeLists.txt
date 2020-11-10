cmake_minimum_required(VERSION 3.10)
project(drake_ros_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(drake REQUIRED)
# Must use Drake's fork of Pybind11
find_package(pybind11 REQUIRED HINTS "${drake_DIR}/../pybind11" NO_DEFAULT_PATH)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

add_library(drake_ros_core
  src/drake_ros.cpp
  src/publisher.cpp
  src/ros_interface_system.cpp
  src/ros_publisher_system.cpp
  src/ros_subscriber_system.cpp
  src/subscription.cpp
)

target_link_libraries(drake_ros_core PUBLIC
    drake::drake
    rclcpp::rclcpp
    rosidl_runtime_c::rosidl_runtime_c
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
)

target_include_directories(drake_ros_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<INSTALL_INTERFACE:include>"
)

ament_export_include_directories(include)
ament_export_libraries(drake_ros_core)
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(drake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_runtime_c)
ament_export_dependencies(rosidl_typesupport_cpp)

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
pybind11_add_module(py_drake_ros_core SHARED
  src/python_bindings/module_drake_ros_core.cpp
)
set_target_properties(py_drake_ros_core PROPERTIES OUTPUT_NAME "drake_ros_core")
target_link_libraries(py_drake_ros_core PRIVATE
  drake_ros_core
)
target_include_directories(py_drake_ros_core
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/python_bindings>"
)

# Sets PYTHON_INSTALL_DIR
_ament_cmake_python_get_python_install_dir()

install(
  TARGETS py_drake_ros_core
  DESTINATION "${PYTHON_INSTALL_DIR}"
)
### End Python bindings

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_clang_format(CONFIG_FILE .clang-format)
  ament_cpplint()

  ament_add_gtest(test_pub_sub test/test_pub_sub.cpp)

  target_link_libraries(test_pub_sub
    drake::drake
    drake_ros_core
    ${test_msgs_TARGETS}
  )

  ament_add_pytest_test(test_pub_sub_py test/test_pub_sub.py)

  ament_add_gtest(test_drake_ros test/test_drake_ros.cpp)
  target_link_libraries(test_drake_ros
    drake_ros_core
  )
endif()

ament_package()
