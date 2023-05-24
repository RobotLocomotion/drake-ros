#!/bin/bash
set -uo pipefail

apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

apt install software-properties-common
add-apt-repository universe

apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update; apt upgrade

apt install ros-dev-tools ros-humble-rmw-cyclonedds-cpp ros-humble-rviz2 ros-humble-ros2cli-common-extensions

rosdep init
rosdep update

cd "$( dirname -- "$0"; )"
rosdep install --from-paths ../../drake_ros --rosdistro=humble
rosdep install --from-paths ../ --rosdistro=humble

# Required for bazel_ros2_rules
apt install python3 python3-toposort
