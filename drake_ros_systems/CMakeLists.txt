cmake_minimum_required(VERSION 3.10)
project(drake_ros_systems)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(drake REQUIRED)
# find_package(pybind11_vendor)
# find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

add_library(drake_ros_systems
  src/drake_ros.cpp
  src/publisher.cpp
  src/ros_interface_system.cpp
  src/ros_publisher_system.cpp
  src/ros_subscriber_system.cpp
  src/subscription.cpp
)
target_link_libraries(drake_ros_systems PUBLIC
  drake::drake
  rclcpp::rclcpp
  rosidl_runtime_c::rosidl_runtime_c
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)
target_include_directories(drake_ros_systems
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<INSTALL_INTERFACE:include>"
)

add_subdirectory(example)

ament_export_dependencies(drake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_generator_c)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# pybind11_add_module(rccl SHARED
#   use_ros.cpp
# )
# target_link_libraries(rccl PRIVATE
#   rclcpp::rclcpp
#   ${std_msgs_TARGETS}
# )
# 
# # Sets PYTHON_INSTALL_DIR
# _ament_cmake_python_get_python_install_dir()
# 
# install(
#   TARGETS rccl
#   DESTINATION "${PYTHON_INSTALL_DIR}"
# )

ament_package()

