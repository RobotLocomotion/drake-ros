cmake_minimum_required(VERSION 3.10)
project(drake_ros_examples)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wno-error=deprecated-declarations)
endif()

message(WARNING "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(WARNING "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")

find_package(Python3 COMPONENTS Interpreter REQUIRED)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sys, json; print(json.dumps(sys.path))"
  OUTPUT_VARIABLE _PY_SYS_PATH
)
message(WARNING "python sys.path: ${_PY_SYS_PATH}")
message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")

find_package(ament_cmake_ros REQUIRED)
find_package(drake REQUIRED)
find_package(drake_ros REQUIRED)
find_package(gflags REQUIRED COMPONENTS shared)

add_subdirectory(examples/hydroelastic)
add_subdirectory(examples/multirobot)
add_subdirectory(examples/rs_flip_flop)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  ament_clang_format(CONFIG_FILE .clang-format)
  ament_cpplint()
endif()

ament_package()
