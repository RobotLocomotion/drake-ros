set(HEADERS
  "contact_markers_system.h"
  "defaults.h"
  "name_conventions.h"
  "rviz_visualizer.h"
  "scene_markers_system.h"
)

# Mock install headers so include paths match installed paths
set(mock_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include")
file(MAKE_DIRECTORY "${mock_include_dir}/drake_ros/viz")
foreach(hdr ${HEADERS})
  configure_file("${hdr}" "${mock_include_dir}/drake_ros/viz/${hdr}" COPYONLY)
endforeach()

add_library(drake_ros_viz
  heatmap_png.inc
  name_conventions.cc
  rviz_visualizer.cc
  scene_markers_system.cc
  contact_markers_system.cc
)

target_link_libraries(drake_ros_viz PUBLIC
    drake::drake
    drake_ros_core
    drake_ros_tf2
    rclcpp::rclcpp
    tf2_eigen::tf2_eigen
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

target_include_directories(drake_ros_viz
  PUBLIC
    "$<BUILD_INTERFACE:${mock_include_dir}>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS drake_ros_viz
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    ${HEADERS}
  DESTINATION include/drake_ros/viz
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gtest(test_scene_markers test/test_scene_markers.cc)
  target_link_libraries(test_scene_markers
    drake::drake
    drake_ros_viz
    ${visualization_msgs_TARGETS}
  )

  ament_add_gtest(test_viz_name_conventions test/test_name_conventions.cc)
  target_include_directories(test_viz_name_conventions
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  )
  target_link_libraries(test_viz_name_conventions
    drake::drake
    drake_ros_viz
  )
endif()
