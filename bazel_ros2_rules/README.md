# ROS 2 rules for Bazel

This project provides rules to build and run against a ROS 2 distribution from Bazel.

Please be sure to review [Alternatives](./ros2#alternatives) for Bazel tooling
for use with the ROS 2 ecosystem.

## Features

- Automatic ROS 2 overlay scraping, `symlink` or `merge`-installed
- ROS 2 aware C++/Python binaries (e.g. `dload` capable via shims)
- ROS 2 interface generation within Bazel. Ament index manifests are
  also generated against any ROS 2 dependencies (`data` or `deps` for `cc` and
  `py` rules).
    - For example, you can generate custom message types, and them as
      dependencies, run `ros2 interface list` under Bazel and see your custom
      generated types.

More detail about these features are listed in [`./ros2`](./ros2).

## Platform support

These rules require ROS 2 Humble distributions on Ubuntu Jammy 22.04 and
onwards.

## Usage

1. Add `bazel_ros2_rules` to your WORKSPACE (e.g. via `http_archive()`).

1. Add `bazel_ros2_rules` dependencies to you WORKSPACE:

```starlark
load("@bazel_ros2_rules//deps:defs.bzl", "add_bazel_ros2_rules_dependencies")
add_bazel_ros2_rules_dependencies()
```

1. Bind a local ROS 2 workspace underlay in your WORKSPACE:

```starlark
load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_local_repository")
ros2_local_repository(
    name = "ros2",
    workspaces = ["/opt/ros/<distro>"],
)
```

For further documentation on available rules, refer to `ros2` package [documentation](ros2/README.md).
