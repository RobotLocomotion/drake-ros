#!/bin/bash
set -eux

export DEBIAN_FRONTEND=noninteractive
# have to still update key?
apt update && yes | apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

cd /opt/drake-ros
cd ros2_example_bazel_installed
yes | ./setup/install_prereqs.sh

cd ../repro

# # Attepmting full install. Very long to download, install, etc.
# yes | apt install dialog
# dpkg -i ./amdgpu-install_21.50.2.50002-1_all.deb
# yes | amdgpu-install --opencl=rocr,legacy --no-32 --accept-eula

# Shane's suggestion (my attempt to automate?)
cat > /etc/ld.so.conf.d/amdgpu-pro.conf <<EOF
/opt/amdgpu-pro/lib/x86_64-linux-gnu/
EOF
mkdir -p /opt/amdgpu-pro/lib/x86_64-linux-gnu/
# Grab a random lib
cp /usr/lib/x86_64-linux-gnu/libz.so.1 /opt/amdgpu-pro/lib/x86_64-linux-gnu/libGL.so.1

cd ../ros2_example_bazel_installed
bazel build @ros2//:rviz2_rviz2
