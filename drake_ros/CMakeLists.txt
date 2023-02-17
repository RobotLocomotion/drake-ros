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

# Install core python file because ament function can't be used in CMake subdirectory
ament_python_install_module(core/drake_ros_core.py)

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