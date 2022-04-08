#!/bin/bash
set -eux

export DEBIAN_FRONTEND=noninteractive
apt update && yes | apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

cd /opt/drake-ros
cd ros2_example_bazel_installed
yes | ./setup/install_prereqs.sh

cd ../repro
yes | apt install dialog
dpkg -i ./amdgpu-install_21.50.2.50002-1_all.deb

cd ../ros2_example_bazel_installed
bazel build @ros2//:rviz2_rviz2
