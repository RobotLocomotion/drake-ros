# Drake ROS

[![drake-ros continuous integration](https://github.com/RobotLocomotion/drake-ros/actions/workflows/main.yml/badge.svg?branch=develop)](https://github.com/RobotLocomotion/drake-ros/actions/workflows/main.yml)

## About

The repository currently contains experimental code.

The intended function of this repository:
 - API to utilize ROS from a Drake program
 - Examples of how to use both Drake and ROS together

## Contributing

### Code style

All code in this repository must comply with Drake's [code style guide](https://drake.mit.edu/code_style_guide.html).

## First time setup for drake-ros

### Install Drake
Choose your desired version of Drake to install locally. `drake-nightly` is
used as an example. Fill in the variables for your PLATFORM (Ubuntu operating
system) and desired local directory for Drake:
```
PLATFORM=focal
DRAKE_INSTALL_DIR=/path/to/desired/install/dir
curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-${PLATFORM}.tar.gz
tar -xvzf drake.tar.gz -C $DRAKE_INSTALL_DIR
sudo apt-get update
sudo $DRAKE_INSTALL_DIR/share/drake/setup/install_prereqs.sh
```
### Install ROS2 Rolling
Follow ROS2 instructions for the install:
https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html

It is recommend installing a minimum of the `ros-rolling-desktop` meta package.

### Install drake-ros

Fill in your desired drake-ros workspace folder:
```
DRAKE_ROS_WS=/path/to/drake_ros_ws
mkdir -p $DRAKE_ROS_WS/src
cd $DRAKE_ROS_WS/src
git clone git@github.com:RobotLocomotion/drake-ros.git
```
Copy out the drake-ros-env.sh script to the root of your workspace:
```
cp $DRAKE_ROS_WS/src/drake-ros/drake-ros-env.sh $DRAKE_ROS_WS
```
- Note: Be sure to edit the `drake-ros-env.sh` script with your custom
`DRAKE_INSTALL_DIR`.

Source environment and install ROS dependencies:
```
cd $DRAKE_ROS_WS
source drake-ros-env.sh
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro rolling -y
```

## Building drake-ros

For each build, source the drake-ros-env.sh file if you haven't already in
this terminal, then build using colcon:
```
DRAKE_ROS_WS=/path/to/drake_ros_ws
cd $DRAKE_ROS_WS
source drake-ros-env.sh
colcon build
```
Finally, source the local drake-ros workspace overlay after the build:
```
source $DRAKE_ROS_WS/install/setup.bash
```
