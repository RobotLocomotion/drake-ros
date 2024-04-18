## Example ROS 2 apps

This package exercises `rosidl` interface generation and usage in C++ and Python, with cross-package interface dependencies.

To check that the resulting interfaces are functional, it includes a demo applications in C++ and Python. Namely:

- Equivalent `oracle_(cc|py)` applications that publish `ros2_example_apps_msgs/msg/Status` messages, serve a `ros2_example_common_msgs/srv/Query` service, and provide a `ros2_example_common_msgs/action/Do` action.
- Equivalent `inquirer_(cc|py)` applications that subscribe to `ros2_example_apps_msgs/msg/Status` messages, make `ros2_example_common_msgs/srv/Query` service requests, and invoke the `ros2_example_common_msgs/action/Do` action.

You may run a C++ oracle against a Python inquirer, and vice versa:

```sh
bazel run //ros2_example_apps:oracle_py
```

```sh
bazel run //ros2_example_apps:inquirer_cc
```

### Example Launch Files

See `bazel_ros2_rules/ros2/README.md`, Launch Files, for notes on
features and limitations.

Example Python launch file:

```sh
bazel run //ros2_example_apps:roslaunch_eg_py

# Or
bazel build //ros2_example_apps:roslaunch_eg_py
./bazel-bin/ros2_example_apps/roslaunch_eg_py
```

Example XML launch file:

```sh
bazel run //ros2_example_apps:roslaunch_eg_xml
```

WARNING: Per notes in `bazel_ros2_rules/ros2/README.md`, the XML launch file
will not resolve paths correctly.
