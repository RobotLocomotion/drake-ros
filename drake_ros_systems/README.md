# Drake ROS Systems

This is a ROS 2 prototype of a solution to [robotlocomotion/Drake#9500](https://github.com/RobotLocomotion/drake/issues/9500).
It is similar to this ROS 1 prototype [`gizatt/drake_ros_systems`](https://github.com/gizatt/drake_ros_systems).
It explores ROS 2 equivalents of `LcmInterfaceSystem`, `LcmPublisherSystem`, `LcmSubscriberSystem`, and `DrakeVisualizer`.

## Building

This package has been built and tested on Ubuntu Focal with ROS Rolling, using a Drake nightly from April 2021.
It may work on other versions of ROS and Drake, but it hasn't been tested.

To build it:

1. [Install ROS Rolling](https://index.ros.org/doc/ros2/Installation/Rolling/)
1. Source your ROS installation `. /opt/ros/rolling/setup.bash`
1. [Download April-ish 2021 Drake binary](https://drake.mit.edu/from_binary.html)
1. Extract the Drake binary installation, install it's prerequisites, and [use this Python virutalenv trick](https://drake.mit.edu/python_bindings.html#inside-virtualenv).
1. Activate the drake virtual environment
1. Build it using Colcon, or using CMake directly
    
    **Colcon**
    1. Make a workspace `mkdir -p ./ws/src`
    1. `cd ./ws/src`
    1. Get this code `git clone https://github.com/RobotLocomotion/drake-ros.git`
    1. `cd ..`
    1. Build this package `colcon build --packages-select drake_ros_systems`
    
    **CMake**
    1. Manually set `CMAKE_PREFIX_PATH`: `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$AMENT_PREFIX_PATH`
    1. Get this code `git clone https://github.com/RobotLocomotion/drake-ros.git`
    1. `cd drake-ros/drake_ros_systems`
    1. Make a build and install folder to avoid installing to the whole system `mkdir build install`
    1. `cd build`
    1. Configure the project `cmake -DCMAKE_INSTALL_PREFIX=$(pwd)/../install ..`
    1. Build the project `make && make install`
