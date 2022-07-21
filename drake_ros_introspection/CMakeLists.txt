cmake_minimum_required(VERSION 3.10)
project(drake_ros_introspection)

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
find_package(drake_ros_core REQUIRED)
find_package(drake_ros_viz REQUIRED)

add_library(drake_ros_introspection INTERFACE)
target_include_directories(drake_ros_introspection
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS drake_ros_introspection
  EXPORT drake_ros_introspection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(drake_ros_introspection)
ament_export_dependencies(drake)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  ament_clang_format(CONFIG_FILE .clang-format)
  ament_cpplint()
endif()

ament_package()
