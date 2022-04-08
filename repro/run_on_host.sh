#!/bin/bash
set -eux

cd $(dirname ${BASH_SOURCE})

deb=amdgpu-install_21.50.2.50002-1_all.deb

if [[ ! -f ${deb} ]]; then
    wget https://repo.radeon.com/amdgpu-install/21.50.2/ubuntu/focal/${deb}
fi

docker run -it -v ${PWD}/..:/opt/drake-ros ros:rolling-ros-base-focal bash -c '/opt/drake-ros/repro/run_on_container.sh'
