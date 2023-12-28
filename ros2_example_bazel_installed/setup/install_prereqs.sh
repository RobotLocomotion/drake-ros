#!/bin/bash

set -eux pipefail

apt update && apt install apt-transport-https curl gnupg lsb-release cmake build-essential gettext-base coreutils

as-user() {
  if [[ -n "${SUDO_USER:+D}" ]]; then
    sudo -u ${SUDO_USER} "$@"
  else
    "$@"
  fi
}

# Install Bazel (derived from setup/ubuntu/source_distribution/install_prereqs.sh at
# https://github.com/RobotLocomotion/drake/tree/e91e62f524788081a8fd231129b64ff80607c1dd)
function dpkg_install_from_curl() {
  package="$1"
  version="$2"
  url="$3"
  checksum="$4"

  installed_version=$(dpkg-query --showformat='${Version}\n' --show "${package}" 2>/dev/null || true)
  if [[ "${installed_version}" != "" ]]; then
      # Skip the install if we're already at the exact version.
      if [[ "${installed_version}" == "${version}" ]]; then
          echo "${package} is already at the desired version ${version}"
          return
      fi

      # If installing our desired version would be a downgrade or an upgrade, ask the user first.
      echo "This system has ${package} version ${installed_version} installed."
      action="upgrade"  # Assume an upgrade
      if dpkg --compare-versions "${installed_version}" gt "${version}"; then
          action="downgrade"  # Switch to a downgrade
      fi
      echo "Drake ROS intends to ${action} to version ${version}, the supported version."
      read -r -p "Do you want to ${action}? [Y/n] " reply
      if [[ ! "${reply}" =~ ^([yY][eE][sS]|[yY])*$ ]]; then
          echo "Skipping ${package} ${version} installation."
          return
      fi
  fi
  # Download and verify.
  tmpdeb="/tmp/${package}_${version}-amd64.deb"
  curl -sSL "${url}" -o "${tmpdeb}"
  if echo "${checksum} ${tmpdeb}" | sha256sum -c -; then
    echo  # Blank line between checkout output and dpkg output.
  else
    echo "ERROR: The ${package} deb does NOT have the expected SHA256. Not installing." >&2
    exit 2
  fi

  # Install.
  dpkg -i "${tmpdeb}"
  rm "${tmpdeb}"
}

apt install g++ unzip zlib1g-dev

# TODO(sloretz) Make sure the version of bazel is exactly the same as the one used by Drake
dpkg_install_from_curl \
  bazel 6.4.0 \
  https://github.com/bazelbuild/bazel/releases/download/6.4.0/bazel_6.4.0-linux-x86_64.deb \
  9276a1e11f03e9f7492f009803c95bddc307993c9ab3c463721c9f6cdaa2ccc1

# If the user did not explicitly specify their installation prefix, install
# ROS 2 Humble.
if [[ -z "${ROS2_DISTRO_PREFIX:-}" ]]; then
  apt update && apt install locales
  locale-gen en_US en_US.UTF-8
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  apt install software-properties-common
  add-apt-repository universe

  # apt update && apt install curl -y
  # curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key --keyring /usr/share/keyrings/ros-archive-keyring.gpg add -

  # echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

  apt update

  apt install ros-humble-desktop
  apt install ros-humble-rmw-cyclonedds-cpp
  apt install ros-dev-tools
fi

# Install Python dependencies
apt install python3 python3-toposort python3-dev python-is-python3

# Install debuggers
apt install gdb lldb
