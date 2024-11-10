# UFO

## Overview

The UFO example shows how to use Drake ROS and the Drake systems framework to
enable controlling a flying object with RViz.

It publishes the following topics:

* `/tf` (all scene frames)
* `/scene_markers/collision` (all collision geometries)
* `/scene_markers/visual` (all visual geometries)

It subscribes to the following topic

* `/goal_pose` (commands where the object should fly to)

## How to run the example

Run the `ufo` executable.

```
ros2 run drake_ros_examples ufo
```

Run RViz in a different terminal with your ROS installation sourced to visualize
the station.

```
ros2 run rviz2 rviz2 -d ufo.rviz
```

Use the `2D Goal Pose` button in RViz to set the location to which the object
should fly to.
