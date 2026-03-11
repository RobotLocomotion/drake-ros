#!/bin/bash

# Run colcon-based tests for drake_ros packages
# This script must to be run inside the drake-ros container

set -eo pipefail

# Ensure we're in the repository root
cd /drake-ros

echo "==== Sourcing ROS 2 environment ===="
source /opt/ros/jazzy/setup.bash

echo "==== Building drake_ros packages with colcon ===="
colcon build

echo "==== Testing drake_ros packages with colcon ===="
source colcon-install/setup.bash

colcon test --event-handlers=console_cohesion+

echo "==== Displaying test results ===="
colcon test-result --test-result-base colcon-build

echo "==== All colcon tests passed ===="
