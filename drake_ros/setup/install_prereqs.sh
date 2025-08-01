#!/bin/bash
set -eux -o pipefail

SCRIPT_DIRECTORY=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

sudo apt update && sudo apt install curl

DRAKE_COMMIT=$(grep 'DRAKE_COMMIT =' "$SCRIPT_DIRECTORY/../MODULE.bazel" | cut -d '"' -f 2)
curl -L -o /tmp/drake.tar.gz \
  https://github.com/RobotLocomotion/drake/archive/${DRAKE_COMMIT}.tar.gz
mkdir -p /tmp/drake && tar -xf /tmp/drake.tar.gz -C /tmp/drake --strip-components 1

# Install the source prereqs
/tmp/drake/setup/install_prereqs --with-bazel

# Install ROS as described in https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install ros-dev-tools

rosdep init || true
rosdep update

rosdep install -i -y --from-paths $SCRIPT_DIRECTORY/.. --rosdistro=jazzy

sudo apt install ros-jazzy-test-msgs ros-jazzy-tf2-ros-py ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-tf2-py

#apt install --no-install-recommends libgcc-13-dev libstdc++-13-dev libgfortran-13-dev

# Required for bazel_ros2_rules
sudo apt install python3 python3-toposort

# Clear apparmor unprivileged user namespace restrictions.
# Required to create network namespaces for ROS isolation.
sudo sysctl -w kernel.apparmor_restrict_unprivileged_unconfined=0
sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0
