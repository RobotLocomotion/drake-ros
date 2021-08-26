cmake_minimum_required(VERSION 3.10)
project(drake_ros_viz)

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
find_package(visualization_msgs REQUIRED)

add_library(drake_ros_viz
  src/utilities/name_conventions.cpp
  src/utilities/type_conversion.cpp
)

target_link_libraries(drake_ros_viz PUBLIC
    drake::drake
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
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(drake)
ament_export_dependencies(geometry_msgs)
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

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  ament_clang_format(CONFIG_FILE .clang-format)
  ament_cpplint()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_type_conversion test/test_type_conversion.cpp)
  target_link_libraries(test_type_conversion
    drake::drake
    drake_ros_viz
  )
endif()

ament_package()
