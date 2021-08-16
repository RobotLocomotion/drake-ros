ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-ros-base
ARG PLATFORM=focal
ARG PACKAGE_NAME=drake_ros_core

# bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    curl \
    && rm -rf /var/lib/apt/lists/*

# drake nightly binary install
RUN curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-${PLATFORM}.tar.gz \
    && tar -xvzf drake.tar.gz -C /opt \
    && apt-get update \
    && apt-get install --no-install-recommends -y $(cat "/opt/drake/share/drake/setup/packages-${PLATFORM}.txt") \
    && rm -rf /var/lib/apt/lists/*

# make root workspace
ENV HOME=/home/${USERNAME}
ENV WORKSPACE=${HOME}/workspace
RUN mkdir -p ${WORKSPACE}/src \
    && mkdir -p ${HOME}/.ros/log

# copy source code into container
COPY /${PACKAGE_NAME} ${WORKSPACE}/src/${PACKAGE_NAME}

# set workspace working directory
WORKDIR ${WORKSPACE}

# install src Debian dependencies
# TODO: Determine why test-msgs package needs to be installed manually.
RUN apt-get update \
    && apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-test-msgs \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && rm -rf /var/lib/apt/lists/*

# build and test workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
    && colcon test \
    && colcon test-result --all --verbose
