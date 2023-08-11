# IIWA Manipulator

## Overview

Both `iiwa_manipulator` and `iiwa_manipulator.py` enable RViz visualization of a static [`ManipulationStation`](https://github.com/RobotLocomotion/drake/tree/master/examples/manipulation_station) example.
They publish the following topics:

* `/tf` (all scene frames)
* `/scene_markers` (all scene geometries, including the robot model)

## How To

First, run RViz:

```sh
# Using Colcon/CMake
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix drake_ros_examples)/share/drake_ros_examples/iiwa_manipulator.rviz

# Using bazel
bazel run @ros2//:rviz2 -- -d `pwd`/examples/iiwa_manipulator/iiwa_manipulator.rviz
```

In a separate terminal, run either the C++ or Python executable:

```sh
# Using Colcon/CMake
# C++
ros2 run drake_ros_examples iiwa_manipulator
# Python
ros2 run drake_ros_examples iiwa_manipulator.py

# Using bazel
# C++
bazel run //examples/iiwa_manipulator:iiwa_manipulator
# Python
bazel run //examples/iiwa_manipulator:iiwa_manipulator_py
```
Or
```sh
# Using ROS 2 Launch (C++)
ros2 launch drake_ros_examples iiwa_manipulator_cc_launch.py
# or (Python)
ros2 launch drake_ros_examples iiwa_manipulator_py_launch.py
```
You should see the manipulation station with simple sinusoidal motion.

**Note***: If you restart the simulation but not RViz, you should click RViz's
"Reset" button so that TF does not get tripped up on stale data.

**Note**: All terminals should have their environment setup appropriately. See
`drake_ros_examples` for an example of how to do so.
