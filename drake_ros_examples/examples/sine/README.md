# Sine

## Overview

This sample demonstrates the use of the Drake-to-ROS connection for Drake ports.
It uses a simple system that outputs a sine wave.
The system is introspected by a SimulatorMonitor that creates a ROS topic publisher.
The output value of the system is published by this publisher, making it accessible to ROS nodes.

It publishes the following topics:

* `/introspection_demo/sine/y0`
* `/introspection_demo/sine/y1`
* `/introspection_demo/sine/y2`

## How To

Run the C++ `sine` executable script as explained [here](../../README.md#running).

```
ros2 run drake_ros_examples sine
```

In a separate terminal, echo one of the topics.

```
ros2 topic echo /introspection_demo/sine/y0
```
