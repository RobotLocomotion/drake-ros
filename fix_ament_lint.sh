#!/bin/bash
set -eux

# Fixes lint errors specific to ament.
# Leave Bazel errors up to Bazel.

cd $(dirname ${BASH_SOURCE})

ament_packages=(
    drake_ros
    drake_ros_examples
)
for ament_package in ${ament_packages[@]}; do
    (
        cd ${ament_package}
        find . -name '*.h' -o -name '*.cc' \
            | xargs ament_clang_format --config .clang-format --reformat
    )
done

# TODO(eric.cousineau): Add Python fixes and CMake fixes.
