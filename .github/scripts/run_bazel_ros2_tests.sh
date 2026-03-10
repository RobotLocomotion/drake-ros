#!/bin/bash

# Run bazel_ros2_rules and ros2_example_bazel_installed tests
# This script must to be run inside the drake-ros container

set -eo pipefail

# Ensure we're in the repository root
cd /drake-ros

echo "==== Configuring bazel_ros2_rules workspace for CI ===="
ln -sf ../.github/ci.bazelrc bazel_ros2_rules/user.bazelrc

echo "==== Configuring ros2_example_bazel_installed workspace for CI ===="
ln -sf ../.github/ci.bazelrc ros2_example_bazel_installed/user.bazelrc

echo "==== Building bazel_ros2_rules workspace ===="
cd bazel_ros2_rules
bazel build //...

echo "==== Testing bazel_ros2_rules workspace ===="
bazel test //...

echo "==== Building ros2_example_bazel_installed workspace ===="
cd ../ros2_example_bazel_installed
bazel build //...

echo "==== Testing ros2_example_bazel_installed workspace ===="
bazel test //... @ros2//...

echo "==== All bazel_ros2_rules tests passed ===="
