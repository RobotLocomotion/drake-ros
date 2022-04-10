#!/bin/bash
set -eux

cd $(dirname ${BASH_SOURCE})

singularity build --fakeroot repro.sif repro.def
singularity exec \
    --fakeroot --writable --no-home --containall \
    --pwd /drake-ros/ros2_example_bazel_installed repro.sif \
    bazel build @ros2//:builtin_interfaces_cc
