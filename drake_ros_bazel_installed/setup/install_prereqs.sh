#!/bin/bash

set -eux pipefail

# Install Bazel
apt install -y apt-transport-https curl gnupg
curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg
mv bazel.gpg /etc/apt/trusted.gpg.d/
echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
apt update && apt install bazel

# Install CPython development libraries
apt install -y python3-dev

# Install ROS 2 Rolling
apt install -y ros-rolling-ros-base ros-rolling-rmw-fastrtps-cpp ros-rolling-rmw-cyclonedds-cpp

# Install debuggers
apt install -y gdb lldb
