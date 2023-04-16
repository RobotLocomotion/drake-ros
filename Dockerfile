# Dockerfile for drake-ros

ARG ARCH
FROM ${ARCH}ros:humble-ros-base-jammy

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

# Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Update and upgrade the system
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y tzdata

# Install necessary dependencies for the script
RUN apt-get install -y wget unzip curl software-properties-common lsb-release python3-pip

# Install Bazelisk using npm as it's straightforward this way
RUN apt-get install -y npm
RUN npm install -g @bazel/bazelisk

# Run Bazelisk to install Bazel
RUN bazelisk

# Install some useful tools for development
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       libeigen3-dev \
                       tmux \
		               zip

RUN apt-get -y dist-upgrade

ARG BUILD_DRAKE_FROM_SOURCE=false
RUN if [ "$BUILD_DRAKE_FROM_SOURCE" = "true" ] ; then \
        # Install build dependencies for Drake \
        apt-get install -y build-essential cmake && \
        # Clone Drake source code \
        git clone https://github.com/RobotLocomotion/drake.git && \
        yes | bash drake/setup/ubuntu/install_prereqs.sh && \
        mkdir drake-build && \
        cd drake-build && \
        # Build Drake from source \
        cmake -DCMAKE_INSTALL_PREFIX=$HOME/drake ../drake && make -j$(nproc) \
        ; \
    else \
        # Download and install Drake dependencies \
        wget -q -O /tmp/drake-setup.zip https://github.com/RobotLocomotion/drake/archive/refs/heads/master.zip && \
    	unzip -q /tmp/drake-setup.zip -d /tmp && \
    	yes | bash /tmp/drake-master/setup/ubuntu/install_prereqs.sh && \
    	rm -rf /tmp/drake-setup.zip /tmp/drake-master && \
        # Download and install pre-built Drake binary for Ubuntu \
        wget https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-jammy.tar.gz && \
        tar -xzf drake-latest-jammy.tar.gz --strip-components=1 -C /opt && \
        rm drake-latest-jammy.tar.gz ; \
    fi

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
    if [ "$BUILD_DRAKE_FROM_SOURCE" = "true" ] ; then \
        colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$HOME/drake \
        ; \
    else \
        colcon build --symlink-install ; \
    fi && \
    colcon test --packages-up-to drake_ros_examples --event-handlers console_cohesion+ && \
    colcon test-result --verbose

WORKDIR '/drake_ros_ws'
# Set the entrypoint to source ROS setup.bash and run a bash shell
ENTRYPOINT ["/bin/bash"]
