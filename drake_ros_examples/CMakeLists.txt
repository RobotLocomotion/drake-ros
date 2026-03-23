cmake_minimum_required(VERSION 3.10)
project(drake_ros_examples)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wno-error=deprecated-declarations)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(drake REQUIRED)
find_package(drake_ros REQUIRED)
find_package(gflags REQUIRED COMPONENTS shared)

add_subdirectory(examples/hydroelastic)
add_subdirectory(examples/multirobot)
add_subdirectory(examples/rs_flip_flop)

ament_package()
