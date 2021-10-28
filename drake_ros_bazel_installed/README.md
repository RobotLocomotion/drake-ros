# Bazel Project with ROS 2 as a Precompiled External

This project pulls in a system installed ROS 2 distribution as a Bazel external repository.

For an introduction to Bazel, refer to [Getting Started with Bazel](https://docs.bazel.build/versions/master/getting-started.html).

## Platform support

This project targets ROS 2 Rolling distributions on Ubuntu Focal 20.04 only.

## Instructions

First, install the required dependencies:

```sh
sudo ./setup/install_prereqs.sh 
```

To build all packages:

```sh
bazel build //...
```

To run binaries directly: 

```sh
bazel run //drake_ros_apps:oracle_cc
```

```sh
bazel run //drake_ros_apps:inquirer_py
```
