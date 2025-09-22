#!/bin/bash

${MAYBE_SUDO} rosdep init || true
rosdep update

# Clear apparmor unprivileged user namespace restrictions.
# Required to create network namespaces for ROS isolation.
${MAYBE_SUDO} sysctl -w kernel.apparmor_restrict_unprivileged_unconfined=0
${MAYBE_SUDO} sysctl -w kernel.apparmor_restrict_unprivileged_userns=0
