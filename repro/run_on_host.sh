#!/bin/bash
set -eux

cd $(dirname ${BASH_SOURCE})

deb=amdgpu-install_21.50.2.50002-1_all.deb

if [[ ! -f ${deb} ]]; then
    wget https://repo.radeon.com/amdgpu-install/21.50.2/ubuntu/focal/${deb}
fi

# Stupid gymnastics to ensure the entrypoint for this container isn't our
# provisioning script (so it's easy to start and exec stuff).
docker run -d -v ${PWD}/..:/opt/drake-ros ros:rolling-ros-base-focal sleep 100000
id=$(docker ps -q | head -n 1)

docker exec -it ${id} /opt/drake-ros/repro/run_on_container.sh

# To clean up, can be heavy handed and remove *all* containers:
#   docker ps -a -q | xargs docker rm -f
