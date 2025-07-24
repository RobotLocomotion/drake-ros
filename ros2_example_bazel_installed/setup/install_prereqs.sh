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

dpkg_install_from_curl \
  bazelisk 1.25.0 \
  https://github.com/bazelbuild/bazelisk/releases/download/v1.25.0/bazelisk-amd64.deb \
  f16dc348190990eb2e8950e773bc91dcdc7632517e5b63bdc4dd58f90062920c

# If the user did not explicitly specify their installation prefix, install
# ROS 2 Jazzy.
if [[ -z "${ROS2_DISTRO_PREFIX:-}" ]]; then
  apt update && apt install locales
  locale-gen en_US en_US.UTF-8
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  apt install software-properties-common
  add-apt-repository universe

# Start - ROS Installation
# Install ROS as described in https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

apt install software-properties-common
add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

# End - ROS Installation

fi

# Ensure a full ROS 2 desktop install
set +u
source ${ROS2_DISTRO_PREFIX}/setup.bash
apt install ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-dev-tools

# Install Python dependencies
apt install python3 python3-toposort python3-dev python-is-python3

# Install debuggers
apt install gdb lldb

# Clear apparmor unprivileged user namespace restrictions.
# Required to create network namespaces for ROS isolation.
sudo sysctl -w kernel.apparmor_restrict_unprivileged_unconfined=0
sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0
