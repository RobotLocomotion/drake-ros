# Hydroelastic

## Overview

This example shows how to visualize both point and hydroelastic contacts in RViz.

It publishes the following topics:

* `/contacts [visualization_msgs/msg/MarkerArray]` - Markers describing contacts
* `/scene_markers/collision [visualization_msgs/msg/MarkerArray]` - all collision geometries
* `/scene_markers/visual [visualization_msgs/msg/MarkerArray]` - all visual geometries
* `/tf [tf2_msgs/msg/TFMessage]` - Frames and transforms

## How To

Run the C++ `hydroelastic` executable in the `drake_ros_examples` package.

```bash
# Using bazel
bazel run //examples/hydroelastic:hydroelastic

# Using Colcon/CMake
ros2 run drake_ros_examples hydroelastic
```

Run RViz in a different terminal with your ROS installation sourced to visualize.

```
# Using bazel
bazel run @ros2//:rviz2 -- -d `pwd`/examples/hydroelastic/hydroelastic.rviz

# Using Colcon/CMake
rviz2 -d collisions.rviz
```
