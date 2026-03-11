#!/bin/bash

# Run drake_ros and drake_ros_examples bazel tests
# This script must be run inside the drake-ros container

set -eo pipefail

# Ensure we're in the repository root
cd /drake-ros

echo "==== Configuring drake_ros workspace for CI ===="
ln -sf ../.github/ci.bazelrc drake_ros/user.bazelrc

echo "==== Building drake_ros workspace ===="
cd drake_ros
bazel build //...

echo "==== Testing drake_ros workspace ===="
bazel test //...

echo "==== Cleaning drake_ros workspace ===="
bazel clean

echo "==== Configuring drake_ros_examples workspace for CI ===="
cd ../drake_ros_examples
ln -sf ../.github/ci.bazelrc user.bazelrc

echo "==== Building drake_ros_examples workspace ===="
bazel build //...

echo "==== Testing drake_ros_examples workspace ===="
bazel test //...

echo "==== Cleaning drake_ros_examples workspace ===="
bazel clean

echo "==== All drake_ros tests passed ===="
