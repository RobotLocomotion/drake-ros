# Drake ROS Systems

This is a ROS 2 prototype of a solution to [robotlocomotion/Drake#9500](https://github.com/RobotLocomotion/drake/issues/9500).
It is similar to this ROS 1 prototype [`gizatt/drake_ros_systems`](https://github.com/gizatt/drake_ros_systems).
It explores ROS 2 equivalents of `LcmInterfaceSystem`, `LcmPublisherSystem`, and `LcmSubscriberSystem`.

# Building

This package has been built and tested on Ubuntu Focal with ROS Rolling, using a Drake nightly from November 2020.
It may work on other versions of ROS and Drake, but it hasn't been tested.

To build it:

1. [Install ROS Rolling](https://index.ros.org/doc/ros2/Installation/Rolling/)
1. Source your ROS installation `. /opt/ros/rolling/setup.bash`
* [Install drake from November-ish 2020](https://drake.mit.edu/from_binary.html)
1. Extract the Drake binary installation, install it's prerequisites, and [use this Python virutalenv trick](https://drake.mit.edu/python_bindings.html#inside-virtualenv).
1. Activate the drake virtual environment
1. Build it using Colcon, or using CMake directly
    
    **Colcon**
    1. Make a workspace `mkdir -p ./ws/src`
    1. `cd ./ws/src`
    1. Get this code `git clone https://github.com/sloretz/drake_ros2_demos.git`
    1. `cd ..`
    1. Build this package `colcon build --packages-select drake_ros_systems`
    
    **CMake**
    1. Manually set `CMAKE_PREFIX_PATH`: `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$AMENT_PREFIX_PATH`
    1. Get this code `git clone https://github.com/sloretz/drake_ros2_demos.git`
    1. `cd drake_ros2_demos/drake_ros_systems`
    1. Make a build and install folder to avoid installing to the whole system `mkdir build install`
    1. `cd build`
    1. Configure the project `cmake -DCMAKE_INSTALL_PREFIX=$(pwd)/../install ..`
    1. Build the project `make && make install`

# Running the Example

An example of using these systems is given in the [`example`](./example) folder in two languages: Python and C++.

If you built with `colcon`, then source your workspace.

```
. ./ws/install/setup.bash
# Also make sure to activate drake virtual environment
```

If you built with plain CMake, then source the ROS workspace and set these variables.

```
. /opt/ros/rolling/setup.bash
# Also make sure to activate drake virtual environment
# CD to folder containing `build` and `install` created earlier, commands below use PWD to find correct path
export LD_LIBRARY_PATH=$(pwd)/install/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$(pwd)/install/lib/$(python -c 'import sys; print(f"python{sys.version_info[0]}.{sys.version_info[1]}")')/site-packages:$PYTHONPATH
```

Now you can run the C++ example from the build folder

If built with **colcon** using an isolated build (default)

```
./build/drake_ros_systems/example/rs_flip_flop
```

If built with **cmake** or a non-isolated build of **colcon**

```
./build/example/rs_flip_flop
```

The Python example can be run from the source folder in either case.

```
python3 ./example/rs_flip_flop.py
```
