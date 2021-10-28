## Demo `drake_ros` apps

This package exercises `rosidl` interface generation and usage in C++ and Python, with cross-package interface dependencies.

To check that the resulting interfaces are functional, it includes a demo applications in C++ and Python. Namely:

- Equivalent `oracle_(cc|py)` applications that publish `drake_ros_apps_msgs/msg/Status` messages, serve a `drake_ros_common_msgs/srv/Query`service, and provide a `drake_ros_common_msgs/action/Do` action.
- Equivalent `inquirer_(cc|py)` applications that subscribe to `drake_ros_apps_msgs/msg/Status` messages, make `drake_ros_common_msgs/srv/Query` service requests, and invoke the `drake_ros_common_msgs/action/Do` action.

You may run a C++ oracle against a Python inquirer, and viceversa:

```sh
bazel run //drake_ros_apps:oracle_py
```

```sh
bazel run //drake_ros_apps:inquirer_cc
```
