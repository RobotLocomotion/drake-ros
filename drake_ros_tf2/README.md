# Drake ROS TF2

This package provides abstractions to simplify the use of tf2 ROS functionality from within Drake.

## Building

This package has been built and tested on Ubuntu Focal with ROS Rolling, using a Drake nightly from April 2021.
It may work on other versions of ROS and Drake, but it hasn't been tested.

To build it:

1. [Install ROS Rolling](https://index.ros.org/doc/ros2/Installation/Rolling/)
1. Source your ROS installation `. /opt/ros/rolling/setup.bash`
1. [Download Drake binary](https://drake.mit.edu/from_binary.html), nightly or any stable releases after 14 Jan 2022.
1. [Use this Python virtualenv trick](https://drake.mit.edu/from_binary.html), under "Use as a Python Library", to extract the Drake binary installation, install its prerequisites.
1. Activate the Drake virtual environment.
1. Build it using Colcon, or using CMake directly.

    **Colcon**
    1. Make a workspace `mkdir -p ./ws/src`
    1. `cd ./ws/src`
    1. Get this code `git clone https://github.com/RobotLocomotion/drake-ros.git`
    1. `cd ..`
    1. Build this package and its dependencies `colcon build --packages-up-to drake_ros_tf2`

    **CMake**
    1. Get this code `git clone https://github.com/RobotLocomotion/drake-ros.git`
    1. Build the [`drake_ros_core`](../drake_ros_core/README.md#building) package using CMake first
    1. Manually set `CMAKE_PREFIX_PATH`: `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(pwd)/drake-ros/drake_ros_core/install`
    1. `cd drake-ros/drake_ros_tf2`
    1. Make a build and install folder to avoid installing to the whole system `mkdir build install`
    1. `cd build`
    1. Configure the project `cmake -DCMAKE_INSTALL_PREFIX=$(pwd)/../install ..`
    1. Build the project `make && make install`
