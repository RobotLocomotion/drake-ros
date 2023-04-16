# Dockerfile for drake-ros

ARG ARCH
FROM ${ARCH}/ros:humble

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

# Update package list, install necessary dependencies and cleanup package list 
RUN apt-get update && \
    apt-get install -y wget unzip curl software-properties-common lsb-release python3-pip && \
    apt-get install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools -y && \
    apt-get install -y tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install Bazel for arm64 systems since install_prereqs.sh doesn't for non x86_64 systems
RUN if [ "$ARCH" = "arm64" ] ; then \
    wget "https://github.com/bazelbuild/bazel/releases/download/6.2.0/bazel-6.2.0-linux-arm64" && \
    chmod 755 bazel-6.2.0-linux-arm64 && \
    mv bazel-6.2.0-linux-arm64 /usr/bin/bazel \
; fi

# Argument to support building Drake from source (only strictly necessary for ARM64 users)
ARG BUILD_DRAKE_FROM_SOURCE=false 

# Install Drake from source or download pre-built binary accordingly
RUN if [ "$BUILD_DRAKE_FROM_SOURCE" = "true" ]; then \
        # Install build dependencies for Drake \
        apt-get update && \
        apt-get install -y build-essential cmake && \
        git clone https://github.com/RobotLocomotion/drake.git && \
        yes | bash drake/setup/ubuntu/install_prereqs.sh && \
        mkdir drake-build && \
        cd drake-build && \
        # Build Drake from source \
        cmake -DCMAKE_INSTALL_PREFIX=/opt/drake ../drake && make -j$(nproc) && \
        make install \
    ; else \
        # Update and add necessary keys and sources for Drake installation
        apt-get update && apt-get install --no-install-recommends \
        ca-certificates gnupg lsb-release wget && \
        wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
        | tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null && \
        echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/drake.list >/dev/null && \
        apt-get update && apt-get install -y --no-install-recommends drake-dev && \
        # Add Drake to the path 
        echo 'export PATH="/opt/drake/bin${PATH:+:${PATH}}"' >> /etc/bash.bashrc && \
        echo 'export PYTHONPATH="/opt/drake/lib/python'$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')'/site-packages${PYTHONPATH:+:${PYTHONPATH}}"' >> /etc/bash.bashrc \
    ; fi

# Clone Drake ROS repository
RUN git clone https://github.com/RobotLocomotion/drake-ros.git
RUN cd drake-ros

# ROS2 workspace setup
RUN mkdir -p drake_ros_ws/src/drake_ros
COPY . /drake_ros_ws/src/drake_ros

RUN source /opt/ros/humble/setup.bash && \
    cd drake_ros_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --symlink-install && \
    colcon test --packages-up-to drake_ros_examples --event-handlers console_cohesion+ && \
    colcon test-result --verbose

WORKDIR '/drake_ros_ws'
# Set the entrypoint to source ROS setup.bash and run a bash shell
ENTRYPOINT ["/bin/bash"]
