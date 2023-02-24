# Multirobot

## Overview

The `multirobot.py` script enables RViz visualisation of an array of Kuka LBR iiwa manipulators.
The manipulators are not controlled.
The expected behaviour is for them to "flop" about under the influence of gravity as simulation time progresses.

The simulation publishes the following topics:

* `/tf` (all scene frames)
* `/scene_markers/collision` (collision geometry of all the robots)
* `/scene_markers/visual` (visual representation of all the robots)

## How to run the example

Run either the C++ executable or the Python script.
For the C++ version of the example, run the executable.

```
# Using Colcon/CMake
ros2 run drake_ros_examples multirobot

# Using bazel
bazel run //examples/multirobot:multirobot
```

For the Python version of the example, run the Python script.

```
# Using Colcon/CMake
ros2 run drake_ros_examples multirobot.py

# Using bazel
bazel run //examples/multirobot:multirobot_py
```

In a separate terminal, launch RViz and provide the path to the configuration file to visualise the robots.

```
# Using Colcon/CMake
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix drake_ros_examples)/share/drake_ros_examples/multirobot.rviz

# Using bazel
bazel run @ros2//:rviz2 -- -d examples/multirobot/multirobot.rviz
```

You should observe a 10x10 array of manipulators flopping about under the influence of gravity.
