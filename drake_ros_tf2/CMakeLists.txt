cmake_minimum_required(VERSION 3.10)
project(drake_ros_tf2)

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
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(drake_ros_tf2
  src/scene_tf_system.cc
  src/scene_tf_broadcaster_system.cc
  src/utilities/name_conventions.cc
  src/utilities/type_conversion.cc
)

target_link_libraries(drake_ros_tf2 PUBLIC
    drake::drake
    drake_ros_core::drake_ros_core
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
)

target_include_directories(drake_ros_tf2
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<INSTALL_INTERFACE:include>"
)

ament_export_targets(drake_ros_tf2)

ament_export_dependencies(drake)
ament_export_dependencies(drake_ros_core)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_ros)

install(
  TARGETS drake_ros_tf2
  EXPORT drake_ros_tf2
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

  ament_add_gtest(test_tf_broadcaster test/test_tf_broadcaster.cc)
  target_link_libraries(test_tf_broadcaster
    drake_ros_tf2
    drake::drake
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
  )

  ament_add_gtest(test_type_conversion test/test_type_conversion.cc)
  target_link_libraries(test_type_conversion
    drake_ros_tf2
    drake::drake
  )

  ament_add_gtest(test_name_conventions test/test_name_conventions.cc)
  target_link_libraries(test_name_conventions
    drake_ros_tf2
    drake::drake
  )
  target_include_directories(drake_ros_tf2
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  )
endif()

ament_package()
