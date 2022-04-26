cmake_minimum_required(VERSION 3.10)
project(drake_ros_viz)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drake REQUIRED)

find_package(drake_ros_core REQUIRED)
find_package(drake_ros_tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(drake_ros_viz
  src/rviz_visualizer.cc
  src/scene_markers_system.cc
  src/utilities/name_conventions.cc
  src/utilities/type_conversion.cc
)

target_link_libraries(drake_ros_viz PUBLIC
    drake::drake
    drake_ros_core::drake_ros_core
    drake_ros_tf2::drake_ros_tf2
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

target_include_directories(drake_ros_viz
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<INSTALL_INTERFACE:include>"
)

ament_export_include_directories(include)
ament_export_libraries(drake_ros_viz)
ament_export_targets(drake_ros_viz)

ament_export_dependencies(drake)
ament_export_dependencies(drake_ros_core)
ament_export_dependencies(drake_ros_tf2)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(visualization_msgs)

install(
  TARGETS drake_ros_viz
  EXPORT drake_ros_viz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  ament_clang_format(CONFIG_FILE .clang-format)
  ament_cpplint()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gtest(test_scene_markers test/test_scene_markers.cc)
  target_link_libraries(test_scene_markers
    drake::drake
    drake_ros_viz
    ${visualization_msgs_TARGETS}
  )

  ament_add_gtest(test_type_conversion test/test_type_conversion.cc)
  target_link_libraries(test_type_conversion
    drake::drake
    drake_ros_viz
  )

  ament_add_gtest(test_name_conventions test/test_name_conventions.cc)
  target_link_libraries(test_name_conventions
    drake::drake
    drake_ros_viz
  )
endif()

ament_package()
