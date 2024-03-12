# Drake ROS

[![drake-ros continuous integration](https://github.com/RobotLocomotion/drake-ros/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/RobotLocomotion/drake-ros/actions/workflows/main.yml?query=branch%3Amain)

## Getting Started

See [`drake_ros_examples`](./drake_ros_examples) for an example of
getting started with both Drake and ROS 2 using `colcon`.

If you are using Bazel to build , please see
[`ros2_example_bazel_installed`](./ros2_example_bazel_installed).

## About

The intended function of this repository is to provide the following ROS 2
capability:

- API for integration between Drake and ROS 2 components. See the
  `drake_ros` package.
- Examples using this API in the `drake_ros_examples` package.
- Bazel Starlark macros and tooling to enable ingesting (already built) ROS 2
  workspaces from either installed locations or tarballs in
  [`bazel_ros2_rules`](./bazel_ros2_rules).
  - Be sure to read about
    [alternative approaches](./bazel_ros2_rules/ros2/#alternatives).
- Examples for using these APIs and Bazel macros in
  [`ros2_example_bazel_installed`](./ros2_example_bazel_installed).

## Supported Configurations:

  - Ubuntu 22.04 + ROS 2 Humble (Recommended)
  - Ubuntu 20.04 + ROS 2 Rolling
  - Mac (only via [Docker](./docker-README.md))
  - Architecture: x86_64 (amd64), arm64 (only via [Docker](./docker-README.md))
  - Bazel >= 5.0

## Docker Support
For users preferring Docker, we offer support for Ubuntu and Macs via Docker. You can build and interact with visualization (`rviz2`) directly on the Docker platform, which is particularly useful for Mac users, including those with Apple Silicon architecture. Please refer to our detailed Docker instructions in the [Docker README](./docker-README.md).

## Usable! But No Stability Commitment

This code is prioritized for use within the TRI Dexterous Manipulation Group.
We do not presently adhere to any stability commitment (e.g., deprecations,
backports, etc.), so please use this code at your own discretion. (It's still
great code, of course!)

Please note that this should be considered *second-party* to Drake; more
specifically:

- Only a small subset of Drake developers are directly involved in this project
  at present.
- This does not go through the same level of review as Drake's code review.
- This project does not yet align with Drake's release cycles or [Drake's
  supported configurations](https://drake.mit.edu/from_source.html#supported-configurations).

## Contributing

See [CONTRIBUTING](./CONTRIBUTING.md) documentation.
