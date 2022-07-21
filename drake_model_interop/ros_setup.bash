#!/bin/bash

if [[ ${0} != ${BASH_SOURCE} ]]; then
    # Sourced in shell / script.
    # N.B.: Passing `-h` or `--help` in argv makes `setup.bash` choke.
    source /opt/ros/noetic/setup.bash
    unset _is_executed
else
    # Executed as binary.
    # Copied from minimal sourcing of `setup.bash`, but fixing all values,
    # and removing the reference(s) to /usr/local.
    export \
        CMAKE_PREFIX_PATH=/opt/ros/noetic \
        LD_LIBRARY_PATH=/opt/ros/noetic/lib \
        PATH=/opt/ros/noetic/bin:/usr/bin:/bin \
        PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig \
        PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
        ROSLISP_PACKAGE_DIRECTORIES= \
        ROS_DISTRO=noetic \
        ROS_ETC_DIR=/opt/ros/noetic/etc/ros \
        ROS_MASTER_URI=http://localhost:11311 \
        ROS_PACKAGE_PATH=/opt/ros/noetic/share \
        ROS_PYTHON_VERSION=3 \
        ROS_ROOT=/opt/ros/noetic/share/ros \
        ROS_VERSION=1
    exec "$@"
fi
