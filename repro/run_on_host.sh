#!/bin/bash
set -eux

cd $(dirname ${BASH_SOURCE})

if [[ ! -f ./repro.sif ]]; then
    singularity build --fakeroot repro.sif repro.def
fi

mkdir -p bazel_cache

if [[ ! -f ./overlay.img ]]; then
    dd if=/dev/zero of=overlay.img bs=1M count=500
    mkfs.ext4 overlay.img
fi

# TODO(eric): How to make sandbox persistent?
# I replaced `--writeable` with `--overlay ... --bind ...`, but eww.
singularity exec \
    --fakeroot \
    --overlay ./overlay.img --bind ./bazel_cache:/root/.cache/bazel \
    --no-home --containall \
    --bind ..:/drake-ros \
    --pwd /drake-ros/ros2_example_bazel_installed \
    repro.sif \
    bazel build @ros2//:builtin_interfaces_cc

    # bazel build //ros2_example_apps:oracle_cc
