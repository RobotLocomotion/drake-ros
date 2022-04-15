#!/bin/bash
set -eux

cd $(dirname ${BASH_SOURCE})

build=/tmp/drake-ros-repro-sandbox

if [[ ! -f ${build}.sif ]]; then
    ( cd .. && find . -name 'bazel-*' | xargs rm -f )

    # Reformat.
    sed 's#{{src}}#'${PWD}/..'#g' ./repro.def.in > ./repro.def

    # TODO(eric): How do I tell it ignore files, and permit recursive copying?
    # apptainer build --fakeroot --sandbox ${build} repro.def
    apptainer build --fakeroot ${build}.sif repro.def
fi

if [[ ! -d ${build} ]]; then
    # Convert sif to writeable sandbox.
    apptainer build --fakeroot --sandbox ${build} ${build}.sif
fi

cache=${build}.bazel_cache
mkdir -p ${cache}

apptainer exec \
    --fakeroot \
    --bind ${cache}:/root/.cache/bazel \
    --no-home \
    --containall \
    --bind ..:/drake-ros \
    --pwd /drake-ros/ros2_example_bazel_installed \
    --nv \
    ${build} \
    bash -eux -c '
        ls -1 /.singularity.d/libs/*.so* | wc -l
        bazel build @ros2//:builtin_interfaces_cc
        bazel run @ros2//:rviz2_rviz2
    '
    # bazel build //ros2_example_apps:oracle_cc
