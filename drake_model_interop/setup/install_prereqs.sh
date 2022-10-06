#!/bin/bash

set -eux -o pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install lsb-release wget gnupg python3 python3-venv python3-pyassimp git ros-noetic-xacro

wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list
apt-get update
apt-get install ignition-fortress
