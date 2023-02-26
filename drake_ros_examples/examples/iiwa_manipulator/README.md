# IIWA Manipulator

## Overview

Both `iiwa_manipulator` and `iiwa_manipulator.py` enable RViz visualization of a static [`ManipulationStation`](https://github.com/RobotLocomotion/drake/tree/master/examples/manipulation_station) example.
They publish the following topics:

* `/tf` (all scene frames)
* `/scene_markers` (all scene geometries, including the robot model)

## How To

First, run RViz:

```sh
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix drake_ros_examples)/share/drake_ros_examples/iiwa_manipulator.rviz
```

In a separate terminal, run either the C++ or Python executable:

```sh
# C++
ros2 run drake_ros_examples iiwa_manipulator

# Python
ros2 run drake_ros_examples iiwa_manipulator.py
```

You should see the manipulation station with simple sinusoidal motion.

**Note***: If you restart the simulation but not RViz, you should click RViz's
"Reset" button so that TF does not get tripped up on stale data.

**Note**: All terminals should have their environment setup appropriately. See
`drake_ros_examples` for an example of how to do so.
