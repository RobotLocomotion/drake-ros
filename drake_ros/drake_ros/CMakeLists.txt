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
  find_package(Python3 COMPONENTS Interpreter)
  find_package(ament_cmake_test REQUIRED)

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
        "${PROJECT_BINARY_DIR}/py/")
  endmacro()

  add_python_test(core_test_py test/core_test.py)
  add_python_test(tf2_test_py test/tf2_test.py)
  add_python_test(viz_test_py test/viz_test.py)
endif()
# End python bindings
