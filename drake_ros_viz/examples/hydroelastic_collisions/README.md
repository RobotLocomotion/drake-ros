# Hydroelastic Collisions

## Overview

Both `iiwa_manipulator` and `iiwa_manipulator.py` enable RViz visualization of a static [`ManipulationStation`](https://github.com/RobotLocomotion/drake/tree/master/examples/manipulation_station) example.
Publishes the following topics:

* `/tf` (all scene frames)
* `/scene_markers` (all scene geometries, including the robot model)
* `/hyrdroelastic_contact/mesh` (contact marker array)

## How To

Run the C++ `hydroelastic_collision` executable as explained [here](../../README.md#running).

Run RViz in a different terminal with your ROS installation sourced to visualize the station:

```
ros2 run rviz2 rviz2 -d hydroelastic_collision.rviz
```


