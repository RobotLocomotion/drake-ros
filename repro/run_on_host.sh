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
    --nv \
    repro.sif \
    bash -eux -c '
        ls -1 /.singularity.d/libs/*.so* | wc -l
        bazel build @ros2//:builtin_interfaces_cc
        bazel run @ros2//:rviz2_rviz2
    '
    # bazel build //ros2_example_apps:oracle_cc
