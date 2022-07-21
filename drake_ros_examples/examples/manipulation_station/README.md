# Manipulation station

## Overview

This sample demonstrates the use of the Drake-to-ROS connection for Drake ports.
It uses the manipulation station sample included in Drake to provide a fairly complex system.
The system is introspected by a SimulatorMonitor that creates two ROS topic publishers.
One publisher provides the joint position commanded to the IIWA robot in the simulation.
The other publisher provides the measured joint positions.
These output values are published by the publisheres, making them accessible to ROS nodes.

It publishes the following topics:

* `/_/manipulation_station/iiwa_position_commanded`
* `/_/manipulation_station/iiwa_position_measured`

## How To

Run the C++ `sine` executable script as explained [here](../../README.md#running).

```
ros2 run drake_ros_examples manipulation_station
```

In a separate terminal, echo one of the topics.

```
ros2 topic echo /_/manipulation_station/iiwa_position_measured`
```

You should see the joint positions output in the terminal.

```
---
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name:
- '0'
- '1'
- '2'
- '3'
- '4'
- '5'
- '6'
position:
- -1.487725015191346
- 0.09999999384812874
- -3.55867481091712e-08
- -1.2000000019039663
- 1.7087395726122562e-07
- 1.600000018100928
- 1.5163324676288342e-08
velocity: []
effort: []
```
