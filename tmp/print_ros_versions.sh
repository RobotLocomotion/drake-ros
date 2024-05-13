#!/bin/bash
set -eux

cur_dir=$(cd $(dirname ${BASH_SOURCE}) && pwd)
cd ${cur_dir}

dpkg --list ros-humble-* \
    | grep ^ii \
    | awk '{print $3 "\t" $2}' \
    > ./print_ros_versions.output.txt
