cmake_minimum_required(VERSION 3.10)
project(drake_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-error=deprecated-declarations)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(drake REQUIRED)
# Must use Drake's fork of Pybind11
find_package(pybind11 REQUIRED HINTS "${drake_DIR}/../pybind11" NO_DEFAULT_PATH)

add_subdirectory(core)
add_subdirectory(tf2)
add_subdirectory(viz)

# TODO (sloretz) - just copy the tests elsewhere to prevent pytest from adding the "basedir" to sys.path
# TODO or use unittest
###
# Python bindings
###
pybind11_add_module(py_drake_ros_core SHARED
  "drake_ros/cc_py.cc"
  "drake_ros/core/cc_pybind.cc"
  "drake_ros/tf2/cc_pybind.cc"
  "drake_ros/viz/cc_pybind.cc"
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

  add_python_test(core_test_py drake_ros/test/core_test.py)
  add_python_test(tf2_test_py drake_ros/test/tf2_test.py)
  add_python_test(viz_test_py drake_ros/test/viz_test.py)
endif()
# End python bindings

# TODO exclude drake_ros/test folder
ament_python_install_package(drake_ros)
# Generate same python package in build directory for use in tests
file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/py/")
file(COPY "drake_ros"
  DESTINATION "${PROJECT_BINARY_DIR}/py/"
  PATTERN "drake_ros/test" EXCLUDE)


if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_pycodestyle REQUIRED)

    ament_clang_format(CONFIG_FILE .clang-format)
    ament_cpplint()
    ament_pycodestyle(--config pycodestyle.ini)
endif()

### Drake library path environment hook
get_target_property(DRAKE_LIBRARY drake::drake LOCATION)
get_filename_component(DRAKE_LIBRARY_DIR ${DRAKE_LIBRARY} DIRECTORY)
if(APPLE)
  set(LIBRARY_PATH_ENV_VAR "DYLD_LIBRARY_PATH")
else()
  set(LIBRARY_PATH_ENV_VAR "LD_LIBRARY_PATH")
endif()
set(
  AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_drake_library_path
  "prepend-non-duplicate;${LIBRARY_PATH_ENV_VAR};${DRAKE_LIBRARY_DIR}")
ament_environment_hooks("env-hooks/drake_library_path.sh.in")

### Python module path environment hook for pydrake
set(
  AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_pydrake_pythonpath
  "prepend-non-duplicate;PYTHONPATH;${drake_PYTHON_DIR}")
ament_environment_hooks("env-hooks/pydrake_pythonpath.sh.in")


ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(drake)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_runtime_c)
ament_export_dependencies(rosidl_typesupport_cpp)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(visualization_msgs)

ament_package()
