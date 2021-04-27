# Drake ROS Systems

This is a ROS 2 prototype of a solution to [robotlocomotion/Drake#9500](https://github.com/RobotLocomotion/drake/issues/9500).
It is similar to this ROS 1 prototype [`gizatt/drake_ros_systems`](https://github.com/gizatt/drake_ros_systems).
It explores ROS 2 equivalents of `LcmInterfaceSystem`, `LcmPublisherSystem`, `LcmSubscriberSystem`, and `DrakeVisualizer`.

# Building

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

# Running

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

Now you can run C++ examples from the build folder and Python examples from the source folder.

# Examples

Some examples of using these systems are given in the [`example`](./example) folder in two languages: Python and C++.

## RS flip flop

### Overview

Both `rs_flip_flop` and `rs_flip_flop.py` implement an RS flip flop using NOR gates.
They subscribe to the following topics:

* `/R`
* `/S`

And publish to the following topics

* `/Q`
* `/Q_not`

### How To

If built with **colcon** using an isolated build (default), run the C++ example as follows:

```
./build/drake_ros_systems/example/rs_flip_flop
```

If built with **cmake** or a non-isolated build of **colcon**, run the C++ example as follows:

```
./build/example/rs_flip_flop
```


The Python example can always be run from the source folder as follows:

```
python3 ./example/rs_flip_flop.py
```

Run these commands in different terminals with your ROS installation sourced to echo the output topics:

```
ros2 topic echo /Q
```

```
ros2 topic echo /Q_not
```

Run these commands in different terminals with your ROS installation sourced to play with the input topics.

```
ros2 topic pub /S std_msgs/msg/Bool "data: false"
ros2 topic pub /S std_msgs/msg/Bool "data: true"
```

```
ros2 topic pub /R std_msgs/msg/Bool "data: false"
ros2 topic pub /R std_msgs/msg/Bool "data: true"
```

## IIWA Manipulator

### Overview

Both `iiwa_manipulator` and `iiwa_manipulator.py` enable RViz visualization of a static `ManipulationStation` example.
They publish the following topics:

* `/tf` (all scene frames)
* `/scene_markers` (all scene geometries, including the robot model)

### How To

If built with **colcon** using an isolated build (default), run the C++ example as follows:

```
./build/drake_ros_systems/example/iiwa_manipulator
```

If built with **cmake** or a non-isolated build of **colcon**, run the C++ example as follows:

```
./build/example/iiwa_manipulator
```


The Python example can always be run from the source folder as follows:

```
python3 ./example/iiwa_manipulator.py
```

Run RViz in a different terminal with your ROS installation sourced to visualize the station:

```
ros2 run rviz2 rviz2 -d ./example/iiwa_manipulator.rviz
```
