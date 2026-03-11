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
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

USER ubuntu:ubuntu

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

# Set up fixuid as entrypoint for runtime UID/GID mapping
ENTRYPOINT ["fixuid"]
CMD ["/bin/bash"]
