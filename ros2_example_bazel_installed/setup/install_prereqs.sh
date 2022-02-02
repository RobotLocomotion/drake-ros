#!/bin/bash

set -eux pipefail

apt update && apt install apt-transport-https curl gnupg lsb-release cmake build-essential gettext-base coreutils

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
  bazel 4.2.1 \
  https://releases.bazel.build/4.2.1/release/bazel_4.2.1-linux-x86_64.deb \
  67447658b8313316295cd98323dfda2a27683456a237f7a3226b68c9c6c81b3a

# Install ROS 2 Rolling tarball
## This installation step always reinstalls as both traceability and
## persistence for ROS 2 Rolling on Focal tarballs are still in the works.
## As this procedure is carried out primarily on CI, we can afford degraded
## UX for the time being.
rm -rf /opt/ros/rolling-focal /tmp/ros2-rolling-linux-focal-amd64-ci.tar.bz2
(cd /tmp && curl -sSL -O http://repo.ros2.org/ci_archives/rolling-on-focal/ros2-rolling-linux-focal-amd64-ci.tar.bz2)
mkdir -p /opt/ros/rolling-focal
tar xf /tmp/ros2-rolling-linux-focal-amd64-ci.tar.bz2 --strip-components=1 -C /opt/ros/rolling-focal
sed -i 's|COLCON_CURRENT_PREFIX="/opt/ros/rolling"||g' /opt/ros/rolling-focal/setup.sh  # Fix wrong chained prefix, if any
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
ROS2_APT_SOURCE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
if ! grep "${ROS2_APT_SOURCE}" /etc/apt/sources.list.d/ros2.list; then
  echo ${ROS2_APT_SOURCE} | tee /etc/apt/sources.list.d/ros2.list
fi
# Install ROS 2 Rolling dependencies
apt update && apt install python3-rosdep libssl-dev
[[ -d /etc/ros/rosdep ]] || rosdep init
rosdep update
rosdep install --from-paths /opt/ros/rolling-focal --ignore-src -y \
  --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# Install Python dependencies
apt install python3 python3-toposort python3-dev python-is-python3

# Install debuggers
apt install gdb lldb
