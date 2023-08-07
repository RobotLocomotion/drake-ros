# Multirobot

## Overview

The `multirobot.py` script enables RViz visualisation of an array of Kuka LBR iiwa manipulators.
The manipulators are not controlled.
The expected behaviour is for them to "flop" about under the influence of gravity as simulation time progresses.

The simulation publishes the following topics:

* `/tf` (all scene frames)
* `/scene_markers/collision` (collision geometry of all the robots)
* `/scene_markers/visual` (visual representation of all the robots)

## How To

First, run the visualizer by launching RViz:

```sh
# Using Colcon/CMake
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix drake_ros_examples)/share/drake_ros_examples/multirobot.rviz

# Using bazel
bazel run @ros2//:rviz2 -- -d `pwd`/examples/multirobot/multirobot.rviz
```

In a separate terminal, run either the C++ executable or the Python script.

```sh
# Using Colcon/CMake
# C++
ros2 run drake_ros_examples multirobot
# Python
ros2 run drake_ros_examples multirobot.py

# Using bazel
# C++
bazel run //examples/multirobot:multirobot
# Python
bazel run //examples/multirobot:multirobot_py
```
Or use ROS 2 Launch:
```sh
# Using ROS 2 Launch (C++)
ros2 launch drake_ros_examples multirobot_cc_launch.py
# or (Python)
ros2 launch drake_ros_examples multirobot_py_launch.py
``` 

You should observe a 5 x 5 array of manipulators flopping about under the influence of gravity.

**Note***: If you restart the simulation but not RViz, you should click RViz's
"Reset" button so that TF does not get tripped up on stale data.

**Note**: All terminals should have their environment setup appropriately. See
`drake_ros_examples` for an example of how to do so.
