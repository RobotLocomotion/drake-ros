#!/bin/bash

# Run all tests in this repository, intended to help local developers test
# their changes prior to pushing to GitHub.

# TODO(jwnimmer-tri) This does not yet run colcon-based tests.
# TODO(jwnimmer-tri) This is missing fix_ament_lint. Running that script
# currently requires extra setup steps ("ament_clang_format: No such ...").
echo "NOTE: The aspirational goal of this script is to run all tests," \
     "but at the moment it does NOT run any colcon-based tests."

me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
cd $(dirname "$me")

set -x
(./fix_bazel_lint.sh --test ) && \
(cd bazel_ros2_rules             && bazel build //...) && \
(cd bazel_ros2_rules             && bazel test //...) && \
(cd drake_ros                    && bazel test //...) && \
(cd drake_ros_examples           && bazel test //...) && \
(cd ros2_example_bazel_installed && bazel test //... @ros2//...) && \
(cd ros2_example_bazel_installed && ./setup/runfiles_direct_test.sh) && \
true
