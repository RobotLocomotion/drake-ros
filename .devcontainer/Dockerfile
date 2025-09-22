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
RUN apt update && apt install -y adduser sudo && rm -rf /var/lib/apt/lists/*
RUN adduser ubuntu sudo && echo "ubuntu ALL=NOPASSWD: ALL" >> /etc/sudoers.d/ubuntu
USER ubuntu

# Cache dependencies
COPY setup /tmp/drake-ros/setup
COPY bazel_ros2_rules/setup /tmp/drake-ros/bazel_ros2_rules/setup
COPY ros2_example_bazel_installed/setup /tmp/drake-ros/ros2_example_bazel_installed/setup
COPY drake_ros/setup /tmp/drake-ros/drake_ros/setup
COPY drake_ros/package.xml /tmp/drake-ros/drake_ros/package.xml
COPY drake_ros_examples/setup /tmp/drake-ros/drake_ros_examples/setup
COPY drake_ros_examples/package.xml /tmp/drake-ros/drake_ros_examples/package.xml

# Install necessary dependencies, cleanup afterwards
RUN yes | /tmp/drake-ros/setup/install_prereqs \
     -C /tmp/throwaway --with-bazel --with-colcon && \
   sudo rm -rf /tmp/throwaway /var/lib/apt/lists/*
