# Dockerfile for drake-ros

FROM ubuntu:noble

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

# Disable interactive prompts.
RUN echo 'APT::Get::Assume-Yes "true";' | tee /etc/apt/apt.conf.d/90yes
RUN echo "debconf debconf/frontend select Noninteractive" | debconf-set-selections

# Prevent upgrades against packages that may interfere with testing.
RUN apt update && \
    apt-mark hold grub-efi-$(dpkg --print-architecture)-signed && \
    rm -rf /var/lib/apt/lists/*

# Switch to non-root sudoer user
RUN apt update && \
    apt install -y \
      adduser \
      curl \
      sudo && \
    rm -rf /var/lib/apt/lists/*
RUN adduser ubuntu sudo && echo "ubuntu ALL=NOPASSWD: ALL" >> /etc/sudoers.d/ubuntu

# Install fixuid for UID/GID mapping
RUN USER=ubuntu && \
    GROUP=ubuntu && \
    ARCH=$(dpkg --print-architecture) && \
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-${ARCH}.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

USER ubuntu:ubuntu

# ARM64 workaround: libquadmath0 is x86-only, create a virtual package to
# satisfy Drake's prerequisites on ARM.
RUN if [ "$(dpkg --print-architecture)" != "amd64" ]; then \
      sudo apt-get update && \
      sudo apt-get install -y equivs && \
      printf "Package: libquadmath0\nVersion: 99.0\nArchitecture: all\nDescription: virtual package for ARM64 compat\n" > /tmp/libquadmath0.ctl && \
      cd /tmp && equivs-build libquadmath0.ctl && \
      sudo dpkg -i /tmp/libquadmath0_99.0_all.deb && \
      sudo rm -rf /tmp/libquadmath0* /var/lib/apt/lists/*; \
    fi

# Cache dependencies
COPY setup /tmp/drake-ros/setup
COPY bazel_ros2_rules/setup /tmp/drake-ros/bazel_ros2_rules/setup
COPY ros2_example_bazel_installed/setup /tmp/drake-ros/ros2_example_bazel_installed/setup
COPY drake_ros/setup /tmp/drake-ros/drake_ros/setup
COPY drake_ros/package.xml /tmp/drake-ros/drake_ros/package.xml
COPY drake_ros_examples/setup /tmp/drake-ros/drake_ros_examples/setup
COPY drake_ros_examples/package.xml /tmp/drake-ros/drake_ros_examples/package.xml

# Install necessary dependencies, cleanup afterwards
# Wrap sysctl to avoid failures from missing /proc/sys entries during Docker build.
RUN sudo dpkg-divert --local --rename --add /usr/sbin/sysctl && \
    printf '#!/bin/sh\n/usr/sbin/sysctl.distrib "$@" 2>/dev/null || true\n' | sudo tee /usr/sbin/sysctl > /dev/null && \
    sudo chmod +x /usr/sbin/sysctl

# Use --with-bazel only; Drake is fetched by Bazel at build time so no
# need for the slow from-source build that --with-colcon triggers on ARM.
RUN yes | /tmp/drake-ros/setup/install_prereqs \
     -C /tmp/throwaway --with-bazel && \
   sudo rm -rf /tmp/throwaway /var/lib/apt/lists/*

# Restore original sysctl (best-effort; if install scripts replaced our
# wrapper we still have a working sysctl binary, so just clean up the divert).
RUN sudo rm -f /usr/sbin/sysctl && \
    sudo dpkg-divert --local --rename --remove /usr/sbin/sysctl 2>/dev/null; true

# Set up fixuid as entrypoint for runtime UID/GID mapping
ENTRYPOINT ["fixuid"]
CMD ["/bin/bash"]
