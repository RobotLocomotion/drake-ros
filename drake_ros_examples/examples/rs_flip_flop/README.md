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
# Using Colcon/CMake
# C++
ros2 run drake_ros_examples rs_flip_flop
# Python
ros2 run drake_ros_examples rs_flip_flop.py

# Using bazel
# C++
bazel run //examples/rs_flip_flop:rs_flip_flop
# Python
bazel run //examples/rs_flip_flop:rs_flip_flop_py
```

Run the following commands in different terminals

```sh
# Using Colcon/CMake
ros2 topic echo /Q

# Using bazel
bazel run @ros2//:ros2 topic echo /Q
```

```sh
# Using Colcon/CMake
ros2 topic echo /Q_not
# Using bazel
bazel run @ros2//:ros2 topic echo /Q_not
```

Run these commands in different terminals to play with the input topics. \
The first two commands will cause `Q` to turn true.

```sh
# Using Colcon/CMake
ros2 topic pub /S std_msgs/msg/Bool "data: false"
ros2 topic pub /R std_msgs/msg/Bool "data: true"
# Using bazel
bazel run @ros2//:ros2 topic pub /S std_msgs/msg/Bool "data: false"
bazel run @ros2//:ros2 topic pub /R std_msgs/msg/Bool "data: true"
```

Run these next two commands to turn `Q` false again.

```sh
# Using Colcon/CMake
ros2 topic pub /R std_msgs/msg/Bool "data: false"
ros2 topic pub /S std_msgs/msg/Bool "data: true"
# Using bazel
bazel run @ros2//:ros2 topic pub /R std_msgs/msg/Bool "data: false"
bazel run @ros2//:ros2 topic pub /S std_msgs/msg/Bool "data: true"
```

**Note**: All terminals should have their environment setup appropriately. See
`drake_ros_examples` for an example of how to do so.
