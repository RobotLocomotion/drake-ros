# RS flip flop

## Overview

Both `rs_flip_flop` and `rs_flip_flop.py` implement an [RS flip flop using NOR gates](https://en.wikipedia.org/wiki/Flip-flop_(electronics)#SR_NOR_latch).
They subscribe to the following topics:

* `/R`
* `/S`

And publish to the following topics

* `/Q`
* `/Q_not`

## How To

Run either the C++ executable or the Python script.

```sh
# C++
ros2 run drake_ros_examples rs_flip_flop

# Python
ros2 run drake_ros_examples rs_flip_flop.py
```

Run the following commands in different terminals

```sh
ros2 topic echo /Q
```

```sh
ros2 topic echo /Q_not
```

Run these commands in different terminals to play with the input topics. \
The first two commands will cause `Q` to turn true.

```sh
ros2 topic pub /S std_msgs/msg/Bool "data: false"
ros2 topic pub /R std_msgs/msg/Bool "data: true"
```

Run these next two commands to turn `Q` false again.

```sh
ros2 topic pub /R std_msgs/msg/Bool "data: false"
ros2 topic pub /S std_msgs/msg/Bool "data: true"
```

**Note**: All terminals should have their environment setup appropriately. See
`drake_ros_examples` for an example of how to do so.
