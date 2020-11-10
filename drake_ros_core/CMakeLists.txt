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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_pub_sub test/test_pub_sub.cpp)

  target_link_libraries(test_pub_sub
    drake::drake
    drake_ros_core
    ${test_msgs_TARGETS}
  )

  ament_add_gtest(test_drake_ros test/test_drake_ros.cpp)
  target_link_libraries(test_drake_ros
    drake_ros_core
  )
endif()

ament_package()
