pybind11_add_module(py_drake_ros_core SHARED
  "cc_py.cc"
  "core/cc_pybind.cc"
  "tf2/cc_pybind.cc"
  "viz/cc_pybind.cc"
)
target_link_libraries(py_drake_ros_core PRIVATE
    drake_ros_core
    drake_ros_tf2
    drake_ros_viz)
set_target_properties(py_drake_ros_core
  PROPERTIES
    OUTPUT_NAME "_cc"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/py/drake_ros/")
target_include_directories(py_drake_ros_core
  PRIVATE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)

ament_get_python_install_dir(python_install_dir)

install(
  TARGETS py_drake_ros_core
  DESTINATION "${python_install_dir}/drake_ros/"
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)

  # TODO(eric.cousineau): I am not sure how to place a test-only Python library
  # within a non-test ament package. To that end, we make this an independent
  # Python package.
  pybind11_add_module(py_test_pub_and_sub_cc SHARED
    "test/test_pub_and_sub_cc_py.cc"
  )
  target_link_libraries(py_test_pub_and_sub_cc PRIVATE
      drake_ros_core)
  set_target_properties(py_test_pub_and_sub_cc
    PROPERTIES
      OUTPUT_NAME "drake_ros_test_pub_and_sub_cc"
      LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/py/")
  target_include_directories(py_test_pub_and_sub_cc
    PRIVATE
      "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
  )

  macro(add_python_test test_name test_file)
    # Invoke Pytest in a way that prevents source space from
    # Getting added to `sys.path`, so that CPython extensions
    # are importable.
    # TODO(eric, shane): Add environment variable to disable rmw_isolation.
    ament_add_test("${test_name}"
      COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}"
        "--junit-xml=${AMENT_TEST_RESULTS_DIR}/drake_ros/${test_name}.xml"
        "--junit-prefix=drake_ros"
      # Let Python import mock-install structure in build folder
      WORKING_DIRECTORY
        "${PROJECT_BINARY_DIR}/py/"
      APPEND_ENV
        "PYTHONPATH=${PROJECT_BINARY_DIR}/py/")
  endmacro()

  add_python_test(core_test_py test/core_test.py)
  add_python_test(tf2_test_py test/tf2_test.py)
  add_python_test(viz_test_py test/viz_test.py)
  add_python_test(geometry_conversions_py test/geometry_conversions_test.py)
endif()
# End python bindings
