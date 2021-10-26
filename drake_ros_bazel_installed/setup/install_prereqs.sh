#!/bin/bash

set -eux pipefail

apt update && apt install -y apt-transport-https curl gnupg lsb-release cmake build-essential

# Install Bazel
curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg
mv bazel.gpg /etc/apt/trusted.gpg.d/
echo "deb [arch=$(dpkg --print-architecture)] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
apt update && apt install bazel

# Install ROS 2 Rolling
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list
apt update && apt install ros-rolling-ros-base ros-rolling-rmw-fastrtps-cpp ros-rolling-rmw-cyclonedds-cpp

# Install Python dependencies
apt install python3 python3-toposort python3-dev python-is-python3

# Install debuggers
apt install gdb lldb
